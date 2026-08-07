#include "InterfaceSensor.h"
#include <Adafruit_NeoPixel.h>
#include <math.h>

// Open Tools > Serial Plotter at 115200 baud. The sketch emits one labelled
// line every 40 ms for the two independent participant channels.
const uint8_t SENSOR1_SDA = 8;
const uint8_t SENSOR1_SCL = 9;
const uint8_t SENSOR2_SDA = A4;
const uint8_t SENSOR2_SCL = A5;

const uint8_t LED_PIN_1 = 7;
const uint8_t LED_PIN_2 = 12;
const uint8_t LED_COUNT = 44;

// Beat pulse color (RGB — Adafruit NeoPixel handles GRB byte reordering internally)
const uint8_t BEAT_R = 220;
const uint8_t BEAT_G = 60;
const uint8_t BEAT_B = 0;

const uint8_t LED_AMBIENT_FLOOR = 3;   // minimum per-pixel brightness between beats
const uint8_t LED_SPARKLE_PEAK = 60;   // peak brightness when a new sparkle fires
const uint8_t NUM_SPARKLES = 6;        // independent twinkling points per strip
const uint8_t LED_BEAT_DECAY = 8;      // beat pulse fades by this per 20 ms frame (~640 ms total)
const uint8_t LED_PIXEL_DECAY = 4;     // per-pixel ambient brightness decay per frame
const uint16_t LED_UPDATE_MS = 20;     // 50 fps

// Motor driver wiring is cross-paired with sensor wiring, not matched by
// number: the D8/D9 (participant 1) sensor's beat drives the D13 motor, and
// the SDA/SCL (participant 2) sensor's beat drives the D6 motor. Named by
// pin rather than participant number to keep that pairing explicit here
// instead of implicit in the analogWrite() call sites below.
const uint8_t MOTOR_PIN_D6 = 6;
const uint8_t MOTOR_PIN_D13 = 13;

const long CONTACT_DETECTED_IR = 30000;
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
const long THRESHOLD_MAX_STEP_RATIO = 2; // cap how far a single cycle can push the
                                          // threshold target up, so one noise spike
                                          // (motion artifact, bad I2C sample) can't
                                          // inflate it and suppress real beats for
                                          // several seconds while it decays back down

const byte INTERVAL_BUFFER_SIZE = 8; // beat intervals kept for BPM averaging and HRV (RMSSD)

const long SPO2_MIN_AC = 20;             // ignore ratio-of-ratios input this close to the noise floor
const float SPO2_MIN_PLAUSIBLE = 70.0;   // reject outlier estimates outside a plausible
const float SPO2_MAX_PLAUSIBLE = 100.0;  // range rather than smoothing them in
const float SPO2_SMOOTHING = 0.1;        // low-pass filter weight applied to each new estimate

// Simplified empirical SpO2 = a - b*R approximation (R = ratio-of-ratios of the
// Red vs. IR AC/DC levels). Widely used in hobbyist MAX30102 projects; NOT
// clinically calibrated. Good enough as a ratiometric biofeedback cue, not a
// real pulse-oximeter reading -- see DESIGN.md.
// float, not double: the Uno R4's Cortex-M4 FPU is single-precision only, so
// double math here would silently fall back to slow software emulation for
// an estimate that's approximate by design anyway. Keeping the whole sketch
// float-only (see computeRMSSD()/updateSpO2() below) also lets the linker
// drop the software double-precision library entirely.
const float SPO2_RATIO_INTERCEPT = 110.0f;
const float SPO2_RATIO_SLOPE = 25.0f;

// Motor pulse envelope: two short linear ramp-up/ramp-down lobes per beat
// ("bum-bum") rather than one single pulse, mimicking the lub-dub of a real
// heartbeat. Peak intensity is a fixed placeholder until the potentiometer
// is wired in to control it.
const uint16_t LOBE1_ATTACK_MS = 30;
const uint16_t LOBE1_DECAY_MS = 90;
const uint16_t LOBE_GAP_MS = 120;     // silence between the two lobes
const uint16_t LOBE2_ATTACK_MS = 25;
const uint16_t LOBE2_DECAY_MS = 90;
const float LOBE2_PEAK_SCALE = 0.7;   // second lobe a bit softer than the first, like a real "dub"
const byte MOTOR_PEAK_INTENSITY = 180; // 0-255 analogWrite duty cycle

// EXPERIMENTAL (this branch only): motor feedback starts by following each
// raw detected beat, then after this long in TRACKING switches to a steady
// pulse timed off the rolling BPM average instead of individual beat
// jitter. BPM/HRV/SpO2 computation is unaffected either way -- this only
// changes what schedules the motor's two-tone pulse.
const unsigned long MOTOR_STEADY_TRANSITION_MS = 7000;

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
    unsigned long trackingStartTime;
    unsigned long nextSteadyPulseTime;

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
          trackingStartTime(0), nextSteadyPulseTime(0),
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
        // Starts the raw-feedback phase timer fresh every time tracking
        // (re)starts, e.g. after a contact loss -- see MOTOR_STEADY_TRANSITION_MS.
        trackingStartTime = millis();
        nextSteadyPulseTime = 0;
    }

    // True once MOTOR_STEADY_TRANSITION_MS has elapsed in TRACKING and a
    // BPM estimate exists to time a steady pulse off of.
    bool inSteadyMotorMode() const
    {
        return state == TRACKING && bpm > 0 &&
               (millis() - trackingStartTime) >= MOTOR_STEADY_TRANSITION_MS;
    }

    // EXPERIMENTAL: once in steady mode, fires a motor pulse on a fixed
    // schedule derived from the rolling BPM average instead of individual
    // raw beat detections. Call once per loop() iteration (time-based, not
    // tied to FIFO sample processing).
    void updateMotorSchedule()
    {
        if (state != TRACKING)
            return;

        if (!inSteadyMotorMode())
        {
            nextSteadyPulseTime = 0; // re-arm so the first steady pulse fires promptly on transition
            return;
        }

        unsigned long now = millis();
        if (nextSteadyPulseTime == 0 || now >= nextSteadyPulseTime)
        {
            motorTriggerTime = now;
            nextSteadyPulseTime = now + 60000UL / bpm;
        }
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

        // sqrtf, not sqrt: keeps this on the hardware single-precision FPU
        // instead of pulling in software double-precision math for a value
        // that's already an integer millisecond count either way.
        return (uint16_t)sqrtf((float)sumSquaredDiff / (intervalCount - 1));
    }

    // BPM/HRV bookkeeping only -- felt/visual feedback is triggered
    // separately (see process()) so a statistically-implausible interval
    // doesn't also suppress the tactile pulse for a perfectly real beat.
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
    }

    // Linear ramp up over attackMs, then linear ramp down over decayMs,
    // scaled to peak. Shared shape for both lobes of the two-tone envelope.
    static byte lobeIntensity(unsigned long elapsed, uint16_t attackMs, uint16_t decayMs, byte peak)
    {
        if (elapsed < attackMs)
            return (byte)((unsigned long)peak * elapsed / attackMs);

        elapsed -= attackMs;
        if (elapsed < decayMs)
            return (byte)((unsigned long)peak * (decayMs - elapsed) / decayMs);

        return 0;
    }

    // Two-tone "bum-bum" heartbeat feel: a full-intensity lobe, a short
    // silence, then a softer second lobe -- purely a function of elapsed
    // time since motorTriggerTime, so it can be sampled every loop()
    // iteration for a smooth envelope without extra per-loop bookkeeping.
    // What sets motorTriggerTime (raw beat detection vs. the steady BPM
    // schedule) is decided elsewhere; this only shapes each pulse.
    byte motorIntensity() const
    {
        if (motorTriggerTime == 0)
            return 0;

        unsigned long elapsed = millis() - motorTriggerTime;
        const uint16_t lobe1Duration = LOBE1_ATTACK_MS + LOBE1_DECAY_MS;

        if (elapsed < lobe1Duration)
            return lobeIntensity(elapsed, LOBE1_ATTACK_MS, LOBE1_DECAY_MS, MOTOR_PEAK_INTENSITY);

        elapsed -= lobe1Duration;
        if (elapsed < LOBE_GAP_MS)
            return 0;

        elapsed -= LOBE_GAP_MS;
        byte lobe2Peak = (byte)(MOTOR_PEAK_INTENSITY * LOBE2_PEAK_SCALE);
        return lobeIntensity(elapsed, LOBE2_ATTACK_MS, LOBE2_DECAY_MS, lobe2Peak);
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

        float irRatio = (float)positivePeak / (float)irDc;
        float redRatio = (float)redPositivePeak / (float)redDc;
        float r = redRatio / irRatio;
        float estimate = SPO2_RATIO_INTERCEPT - SPO2_RATIO_SLOPE * r;

        if (estimate < SPO2_MIN_PLAUSIBLE || estimate > SPO2_MAX_PLAUSIBLE)
            return; // discard rather than smoothing in an implausible outlier

        spo2 = (spo2 <= 0) ? estimate : (spo2 * (1.0f - SPO2_SMOOTHING) + estimate * SPO2_SMOOTHING);
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

                // Felt/visual feedback on every real detected peak, even if
                // its interval got rejected above for BPM/HRV purposes --
                // an implausible interval doesn't mean the peak was fake,
                // and a suppressed motor pulse reads as "sometimes nothing
                // happens" even though the sensor saw a real beat.
                beatFlashUntil = millis() + 120;

                // EXPERIMENTAL: once in steady mode, updateMotorSchedule()
                // owns motorTriggerTime instead of raw beat detection.
                if (!inSteadyMotorMode())
                    motorTriggerTime = millis();
            }

            updateSpO2();

            long nextThreshold = positivePeak / 2;
            if (nextThreshold < MIN_THRESHOLD_FLOOR)
                nextThreshold = MIN_THRESHOLD_FLOOR;
            long maxNextThreshold = threshold * THRESHOLD_MAX_STEP_RATIO;
            if (nextThreshold > maxNextThreshold)
                nextThreshold = maxNextThreshold;
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

// ── LED rendering ─────────────────────────────────────────────────────────────
//
// Each strip has:
//   - A per-pixel brightness array that decays slowly toward an ambient floor,
//     with a handful of "sparkles" that pop up at random positions and fade out
//     to create a twinkling baseline.
//   - A beat pulse value (0-255) that jumps to full on each new detected beat
//     and decays over ~420 ms, additively brightening all pixels.
//
// Timing: the strip is re-rendered at LED_UPDATE_MS (20 ms / 50 fps), which is
// independent of the sensor FIFO poll rate (40 ms). NeoPixel show() disables
// interrupts for ~1.3 ms (44 LEDs × 30 µs); SoftWire is purely software-timed
// so they don't interfere — just keep show() out of the middle of readChannel().
struct LedState
{
    uint8_t pxBright[LED_COUNT];
    uint8_t beatPulse;
    unsigned long lastBeatFlash;   // last beatFlashUntil value we've seen; used
                                   // to detect a new beat without re-triggering
                                   // on every loop iteration during the flash window
    unsigned long lastUpdate;

    uint8_t sparklePos[NUM_SPARKLES];
    uint8_t sparkleTimer[NUM_SPARKLES]; // counts down; when 0, pick a new position

    void begin()
    {
        for (uint8_t i = 0; i < LED_COUNT; i++)
            pxBright[i] = LED_AMBIENT_FLOOR;
        beatPulse = 0;
        lastBeatFlash = 0;
        lastUpdate = 0;
        for (uint8_t i = 0; i < NUM_SPARKLES; i++)
        {
            sparklePos[i] = random(LED_COUNT);
            sparkleTimer[i] = random(10, 40);
        }
    }

    // Call once per loop() — detects a newly set beatFlashUntil and arms the pulse.
    // On beat: flood all pixels to full brightness so the flash is impossible to miss.
    void checkBeat(unsigned long beatFlashUntil)
    {
        if (beatFlashUntil != lastBeatFlash && beatFlashUntil > millis())
        {
            beatPulse = 255;
            for (uint8_t i = 0; i < LED_COUNT; i++)
                pxBright[i] = 255;
            lastBeatFlash = beatFlashUntil;
        }
    }

    bool needsUpdate() const { return millis() - lastUpdate >= LED_UPDATE_MS; }

    void update(Adafruit_NeoPixel &strip)
    {
        lastUpdate = millis();

        // Decay the beat pulse
        if (beatPulse > LED_BEAT_DECAY)
            beatPulse -= LED_BEAT_DECAY;
        else
            beatPulse = 0;

        // Decay per-pixel brightness toward the ambient floor
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
            if (pxBright[i] > LED_AMBIENT_FLOOR + LED_PIXEL_DECAY)
                pxBright[i] -= LED_PIXEL_DECAY;
            else
                pxBright[i] = LED_AMBIENT_FLOOR;
        }

        // Advance sparkles only when the beat pulse has fully faded — keeps the
        // flash clean (no new sparkles popping during the bright decay window).
        if (beatPulse == 0)
        {
            for (uint8_t i = 0; i < NUM_SPARKLES; i++)
            {
                if (sparkleTimer[i] == 0)
                {
                    sparklePos[i] = random(LED_COUNT);
                    pxBright[sparklePos[i]] = random(LED_SPARKLE_PEAK / 2, LED_SPARKLE_PEAK);
                    sparkleTimer[i] = random(15, 50);
                }
                else
                {
                    sparkleTimer[i]--;
                }
            }
        }

        // Write pixels: ambient sparkle brightness + additive beat pulse, clamped to 255
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
            uint16_t b = (uint16_t)pxBright[i] + beatPulse;
            if (b > 255) b = 255;
            uint8_t bv = (uint8_t)b;
            strip.setPixelColor(i, strip.Color(
                (uint8_t)((uint16_t)BEAT_R * bv / 255),
                (uint8_t)((uint16_t)BEAT_G * bv / 255),
                (uint8_t)((uint16_t)BEAT_B * bv / 255)
            ));
        }
        strip.show();
    }
};

Adafruit_NeoPixel strip1(LED_COUNT, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED_COUNT, LED_PIN_2, NEO_GRB + NEO_KHZ800);

LedState leds1, leds2;

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
    // EXPERIMENTAL: 0 = following raw beat detection, 1 = steady BPM pulse.
    Serial.print(F(" Mode")); Serial.print(suffix); Serial.print(':'); Serial.print(channel.inSteadyMotorMode() ? 1 : 0);
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    pinMode(MOTOR_PIN_D6, OUTPUT);
    pinMode(MOTOR_PIN_D13, OUTPUT);
    analogWrite(MOTOR_PIN_D6, 0);
    analogWrite(MOTOR_PIN_D13, 0);

    strip1.begin(); strip1.clear(); strip1.show();
    strip2.begin(); strip2.clear(); strip2.show();
    leds1.begin();
    leds2.begin();

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

    // EXPERIMENTAL: advances the steady-BPM pulse schedule once in steady
    // mode. No-op (and harmless) outside TRACKING or during the initial
    // raw-feedback phase -- see MOTOR_STEADY_TRANSITION_MS.
    participant1.updateMotorSchedule();
    participant2.updateMotorSchedule();

    // Sampled once per loop and reused for both the motor write and the
    // (throttled) print below, so the printed Motor value always matches
    // exactly what was written to the pin this iteration.
    byte motor1 = participant1.motorIntensity();
    byte motor2 = participant2.motorIntensity();

    // Updated every loop() iteration (not gated by the plot throttle below)
    // so the ramp envelope is smooth rather than stepping in 40 ms chunks.
    // Cross-paired per the wiring: participant 1 (D8/D9 sensor) -> D13
    // motor, participant 2 (SDA/SCL sensor) -> D6 motor.
    analogWrite(MOTOR_PIN_D13, motor1);
    analogWrite(MOTOR_PIN_D6, motor2);

    // Check for new beats and update LED strips at 50 fps.
    // Called after sensor reads so show() never interrupts a SoftWire transaction.
    leds1.checkBeat(participant1.beatFlashUntil);
    leds2.checkBeat(participant2.beatFlashUntil);
    if (leds1.needsUpdate()) leds1.update(strip1);
    if (leds2.needsUpdate()) leds2.update(strip2);

    if (millis() - lastPlotMs < 40)
        return;
    lastPlotMs = millis();

    // Clean plot output. State: 0 = no contact, 1 = calibrating, 2 = tracking.
    printChannel(participant1, '1', motor1);
    Serial.print(' ');
    printChannel(participant2, '2', motor2);
    Serial.println();
}
