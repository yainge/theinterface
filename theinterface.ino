#include "InterfaceSensor.h"
#include <math.h>

// Open Tools > Serial Plotter at 115200 baud. The sketch emits one labelled
// line every 40 ms for the two independent participant channels.
const uint8_t SENSOR1_SDA = 8;
const uint8_t SENSOR1_SCL = 9;
const uint8_t SENSOR2_SDA = A4;
const uint8_t SENSOR2_SCL = A5;

const uint8_t MOTOR1_PIN = 6;
const uint8_t MOTOR2_PIN = 13;

const long CONTACT_DETECTED_IR = 6000;
const long GAIN_TARGET_LOW = 90000;
const long GAIN_TARGET_HIGH = 225000;
const byte INITIAL_LED_CURRENT = 0x30;
const byte MIN_LED_CURRENT = 0x08;
const byte MAX_LED_CURRENT = 0x3F;
const uint16_t SAMPLE_PERIOD_MS = 40; // MAX30102 samples at 100 Hz, but FIFO_CFG
                                       // averages 4 samples per FIFO push (see
                                       // InterfaceSensor::setupSensor), so a new
                                       // FIFO entry only arrives every 40 ms.
const uint16_t GAIN_ADJUST_INTERVAL_MS = 350;
const uint16_t GAIN_SETTLE_MS = 2500;
const uint16_t MIN_BEAT_INTERVAL_MS = 300;
const uint16_t MAX_BEAT_INTERVAL_MS = 2000;

const long INITIAL_THRESHOLD = 120;   // starting/reset adaptive beat threshold
const long MIN_THRESHOLD_FLOOR = 80;  // threshold never adapts below this

const byte INTERVAL_BUFFER_SIZE = 8; // beat intervals kept for BPM averaging and HRV (RMSSD)

const long SPO2_MIN_AC = 20;             // ignore ratio-of-ratios input this close to the noise floor
const float SPO2_MIN_PLAUSIBLE = 70.0;   // reject outlier estimates outside a plausible
const float SPO2_MAX_PLAUSIBLE = 100.0;  // range rather than smoothing them in
const float SPO2_SMOOTHING = 0.1;        // low-pass filter weight applied to each new estimate

// Simplified empirical SpO2 = a - b*R approximation (R = ratio-of-ratios of the
// Red vs. IR AC/DC levels). Widely used in hobbyist MAX30102 projects; NOT
// clinically calibrated. Good enough as a ratiometric biofeedback cue, not a
// real pulse-oximeter reading -- see DESIGN.md.
const double SPO2_RATIO_INTERCEPT = 110.0;
const double SPO2_RATIO_SLOPE = 25.0;

// Motor pulse envelope: linear ramp up, linear ramp down -- avoids the
// abrupt "click" of just switching the motor on/off. Peak intensity is a
// fixed placeholder until the potentiometer is wired in to control it.
const uint16_t MOTOR_ATTACK_MS = 40;
const uint16_t MOTOR_DECAY_MS = 200;
const byte MOTOR_PEAK_INTENSITY = 180; // 0-255 analogWrite duty cycle

enum ChannelState : byte
{
    NO_CONTACT,
    CALIBRATING,
    TRACKING
};

struct HeartChannel
{
    InterfaceSensor sensor;
    ChannelState state;

    byte irLedCurrent;
    long dcEstimate;
    long wave;
    long positivePeak;

    byte redLedCurrent;
    long redDcEstimate;
    long redWave;
    long redPositivePeak;

    long threshold;
    unsigned long sampleTime;
    unsigned long lastBeatTime;
    unsigned long beatFlashUntil;
    unsigned long lastGainAdjust;
    unsigned long gainStableSince;
    unsigned long motorTriggerTime;

    uint16_t bpm;
    uint16_t hrv;
    float spo2;

    uint16_t intervals[INTERVAL_BUFFER_SIZE];
    byte intervalCount;
    byte intervalIndex;

    HeartChannel(uint8_t sda, uint8_t scl)
        : sensor(sda, scl), state(NO_CONTACT),
          irLedCurrent(INITIAL_LED_CURRENT), dcEstimate(0), wave(0), positivePeak(0),
          redLedCurrent(INITIAL_LED_CURRENT), redDcEstimate(0), redWave(0), redPositivePeak(0),
          threshold(INITIAL_THRESHOLD), sampleTime(0), lastBeatTime(0),
          beatFlashUntil(0), lastGainAdjust(0), gainStableSince(0), motorTriggerTime(0),
          bpm(0), hrv(0), spo2(0),
          intervalCount(0), intervalIndex(0)
    {
    }

    // Fields reset the same way by all three state transitions below.
    // dcEstimate/redDcEstimate, sampleTime, beatFlashUntil, and the gain
    // settle timer differ per transition and are handled separately.
    void resetBeatState()
    {
        wave = 0;
        positivePeak = 0;
        redWave = 0;
        redPositivePeak = 0;
        threshold = INITIAL_THRESHOLD;
        lastBeatTime = 0;
        motorTriggerTime = 0;
        bpm = 0;
        hrv = 0;
        spo2 = 0;
        intervalCount = 0;
        intervalIndex = 0;
    }

    void enterNoContact()
    {
        state = NO_CONTACT;
        dcEstimate = 0;
        redDcEstimate = 0;
        sampleTime = 0;
        beatFlashUntil = 0;
        resetBeatState();
    }

    void enterCalibration(long red, long ir)
    {
        state = CALIBRATING;
        dcEstimate = ir << 4;
        redDcEstimate = red << 4;
        sampleTime = millis();
        beatFlashUntil = 0;
        resetBeatState();
        lastGainAdjust = 0;
        gainStableSince = millis();
    }

    void enterTracking()
    {
        state = TRACKING;
        resetBeatState();
    }

    static byte stepLedCurrent(byte current, long dcLevel)
    {
        byte next = current;
        if (dcLevel < GAIN_TARGET_LOW && current < MAX_LED_CURRENT)
        {
            next = current + 2;
            if (next > MAX_LED_CURRENT)
                next = MAX_LED_CURRENT;
        }
        else if (dcLevel > GAIN_TARGET_HIGH && current > MIN_LED_CURRENT)
        {
            next = current - 2;
            if (next < MIN_LED_CURRENT)
                next = MIN_LED_CURRENT;
        }
        return next;
    }

    void adjustGain()
    {
        if (millis() - lastGainAdjust < GAIN_ADJUST_INTERVAL_MS)
            return;
        lastGainAdjust = millis();

        bool changed = false;

        byte nextIr = stepLedCurrent(irLedCurrent, dcEstimate >> 4);
        if (nextIr != irLedCurrent && sensor.setIRLedAmplitude(nextIr))
        {
            irLedCurrent = nextIr;
            changed = true;
        }

        byte nextRed = stepLedCurrent(redLedCurrent, redDcEstimate >> 4);
        if (nextRed != redLedCurrent && sensor.setRedLedAmplitude(nextRed))
        {
            redLedCurrent = nextRed;
            changed = true;
        }

        if (changed)
            gainStableSince = millis();
    }

    // RMSSD (root mean square of successive differences) over the beat
    // intervals currently held in the ring buffer, walked in chronological
    // order. A standard short-window, real-time-friendly HRV measure.
    uint16_t computeRMSSD() const
    {
        if (intervalCount < 2)
            return 0;

        byte oldestIndex = (intervalCount < INTERVAL_BUFFER_SIZE) ? 0 : intervalIndex;
        long previous = intervals[oldestIndex];
        unsigned long sumSquaredDiff = 0;

        for (byte i = 1; i < intervalCount; ++i)
        {
            byte idx = (oldestIndex + i) % INTERVAL_BUFFER_SIZE;
            long diff = (long)intervals[idx] - previous;
            sumSquaredDiff += (unsigned long)(diff * diff);
            previous = intervals[idx];
        }

        return (uint16_t)sqrt((double)sumSquaredDiff / (intervalCount - 1));
    }

    void recordBeat(unsigned long interval)
    {
        intervals[intervalIndex] = interval;
        intervalIndex = (intervalIndex + 1) % INTERVAL_BUFFER_SIZE;
        if (intervalCount < INTERVAL_BUFFER_SIZE)
            ++intervalCount;

        unsigned long sum = 0;
        for (byte i = 0; i < intervalCount; ++i)
            sum += intervals[i];
        bpm = 60000UL / (sum / intervalCount);

        hrv = computeRMSSD();
        beatFlashUntil = millis() + 120;
        motorTriggerTime = millis();
    }

    // Linear ramp up over MOTOR_ATTACK_MS, then linear ramp down over
    // MOTOR_DECAY_MS. Purely a function of elapsed time since the last
    // triggered beat, so it can be sampled every loop() iteration for a
    // smooth envelope without any extra per-loop bookkeeping.
    byte motorIntensity() const
    {
        if (motorTriggerTime == 0)
            return 0;

        unsigned long elapsed = millis() - motorTriggerTime;

        if (elapsed < MOTOR_ATTACK_MS)
            return (byte)((unsigned long)MOTOR_PEAK_INTENSITY * elapsed / MOTOR_ATTACK_MS);

        elapsed -= MOTOR_ATTACK_MS;
        if (elapsed < MOTOR_DECAY_MS)
            return (byte)((unsigned long)MOTOR_PEAK_INTENSITY * (MOTOR_DECAY_MS - elapsed) / MOTOR_DECAY_MS);

        return 0;
    }

    // Ratio-of-ratios SpO2 estimate, evaluated once per full waveform cycle
    // from the AC amplitude (positivePeak) and DC level of both LED
    // channels. Runs independently of the BPM beat-plausibility check, since
    // SpO2 doesn't need the same interval bounds.
    void updateSpO2()
    {
        long irDc = dcEstimate >> 4;
        long redDc = redDcEstimate >> 4;

        if (positivePeak < SPO2_MIN_AC || redPositivePeak < SPO2_MIN_AC || irDc <= 0 || redDc <= 0)
            return;

        double irRatio = (double)positivePeak / (double)irDc;
        double redRatio = (double)redPositivePeak / (double)redDc;
        double r = redRatio / irRatio;
        double estimate = SPO2_RATIO_INTERCEPT - SPO2_RATIO_SLOPE * r;

        if (estimate < SPO2_MIN_PLAUSIBLE || estimate > SPO2_MAX_PLAUSIBLE)
            return; // discard rather than smoothing in an implausible outlier

        spo2 = (spo2 <= 0) ? estimate : (spo2 * (1.0 - SPO2_SMOOTHING) + estimate * SPO2_SMOOTHING);
    }

    void process(long red, long ir)
    {
        if (ir < CONTACT_DETECTED_IR)
        {
            if (state != NO_CONTACT)
                enterNoContact();
            return;
        }

        if (state == NO_CONTACT)
            enterCalibration(red, ir);

        sampleTime += SAMPLE_PERIOD_MS;

        if (state == CALIBRATING)
        {
            // A relatively fast baseline is useful here because the LED gain
            // changes during calibration. BPM/HRV/SpO2 are intentionally
            // disabled until gain is locked.
            dcEstimate += ((ir << 4) - dcEstimate) >> 4;
            redDcEstimate += ((red << 4) - redDcEstimate) >> 4;
            adjustGain();

            if (millis() - gainStableSince >= GAIN_SETTLE_MS)
                enterTracking();
            return;
        }

        // Tracking locks the gain. The slower baseline removes skin/contact
        // DC level while preserving the small pulsatile AC waveform.
        long previousWave = wave;
        dcEstimate += ((ir << 4) - dcEstimate) >> 6;
        wave = ir - (dcEstimate >> 4);

        redDcEstimate += ((red << 4) - redDcEstimate) >> 6;
        redWave = red - (redDcEstimate >> 4);

        if (wave > positivePeak)
            positivePeak = wave;
        if (redWave > redPositivePeak)
            redPositivePeak = redWave;

        // Evaluate one pulse candidate at each downward zero crossing. The
        // adaptive threshold is per participant and follows pulse amplitude.
        if (previousWave > 0 && wave <= 0)
        {
            if (positivePeak > threshold)
            {
                if (lastBeatTime != 0)
                {
                    unsigned long interval = sampleTime - lastBeatTime;
                    if (interval >= MIN_BEAT_INTERVAL_MS && interval <= MAX_BEAT_INTERVAL_MS)
                        recordBeat(interval);
                }
                lastBeatTime = sampleTime;
            }

            updateSpO2();

            long nextThreshold = positivePeak / 2;
            if (nextThreshold < MIN_THRESHOLD_FLOOR)
                nextThreshold = MIN_THRESHOLD_FLOOR;
            threshold = (threshold * 3 + nextThreshold) / 4;
            positivePeak = 0;
            redPositivePeak = 0;
        }
    }

    int plotState() const
    {
        return state == NO_CONTACT ? 0 : (state == CALIBRATING ? 1 : 2);
    }
};

HeartChannel participant1(SENSOR1_SDA, SENSOR1_SCL);
HeartChannel participant2(SENSOR2_SDA, SENSOR2_SCL);

unsigned long lastPlotMs = 0;

bool startSensor(HeartChannel &channel, const __FlashStringHelper *name)
{
    Serial.print(name);
    Serial.print(F(": "));

    if (!channel.sensor.begin())
    {
        Serial.println(F("not found"));
        return false;
    }
    if (!channel.sensor.setupSensor())
    {
        Serial.println(F("setup failed"));
        return false;
    }
    if (!channel.sensor.setIRLedAmplitude(channel.irLedCurrent))
    {
        Serial.println(F("IR LED setup failed"));
        return false;
    }
    if (!channel.sensor.setRedLedAmplitude(channel.redLedCurrent))
    {
        Serial.println(F("Red LED setup failed"));
        return false;
    }

    Serial.println(F("ready"));
    return true;
}

void readChannel(HeartChannel &channel)
{
    byte count = channel.sensor.getFIFOCount();
    for (byte i = 0; i < count; ++i)
    {
        uint32_t red, ir;
        if (channel.sensor.readFIFO(red, ir))
            channel.process((long)red, (long)ir);
    }
}

// Prints one channel's fields with the given participant suffix ('1' or
// '2'). Field order/text matches the original hand-duplicated version
// exactly, so Serial Plotter label parsing is unaffected.
void printChannel(const HeartChannel &channel, char suffix, byte motor)
{
    Serial.print(F("Wave")); Serial.print(suffix); Serial.print(':'); Serial.print(channel.wave);
    Serial.print(F(" Beat")); Serial.print(suffix); Serial.print(':'); Serial.print(millis() < channel.beatFlashUntil ? 2500 : 0);
    Serial.print(F(" BPM")); Serial.print(suffix); Serial.print(':'); Serial.print(channel.bpm);
    Serial.print(F(" HRV")); Serial.print(suffix); Serial.print(':'); Serial.print(channel.hrv);
    Serial.print(F(" SpO2_")); Serial.print(suffix); Serial.print(':'); Serial.print(channel.spo2, 1);
    Serial.print(F(" Motor")); Serial.print(suffix); Serial.print(':'); Serial.print(motor);
    Serial.print(F(" State")); Serial.print(suffix); Serial.print(':'); Serial.print(channel.plotState());
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    pinMode(MOTOR1_PIN, OUTPUT);
    pinMode(MOTOR2_PIN, OUTPUT);
    analogWrite(MOTOR1_PIN, 0);
    analogWrite(MOTOR2_PIN, 0);

    Serial.println(F("The Interface - HR Visualizer"));
    bool sensor1Ready = startSensor(participant1, F("Sensor 1"));
    bool sensor2Ready = startSensor(participant2, F("Sensor 2"));

    if (!sensor1Ready || !sensor2Ready)
        Serial.println(F("Fix sensor wiring before using the visualizer."));
    else
        Serial.println(F("Open Serial Plotter at 115200 baud."));
}

void loop()
{
    readChannel(participant1);
    readChannel(participant2);

    // Sampled once per loop and reused for both the motor write and the
    // (throttled) print below, so the printed Motor value always matches
    // exactly what was written to the pin this iteration.
    byte motor1 = participant1.motorIntensity();
    byte motor2 = participant2.motorIntensity();

    // Updated every loop() iteration (not gated by the plot throttle below)
    // so the ramp envelope is smooth rather than stepping in 40 ms chunks.
    analogWrite(MOTOR1_PIN, motor1);
    analogWrite(MOTOR2_PIN, motor2);

    if (millis() - lastPlotMs < 40)
        return;
    lastPlotMs = millis();

    // Clean plot output. State: 0 = no contact, 1 = calibrating, 2 = tracking.
    printChannel(participant1, '1', motor1);
    Serial.print(' ');
    printChannel(participant2, '2', motor2);
    Serial.println();
}
