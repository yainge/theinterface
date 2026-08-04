#include "InterfaceSensor.h"

// Open Tools > Serial Plotter at 115200 baud. The sketch emits one labelled
// line every 40 ms for the two independent participant channels.
const uint8_t SENSOR1_SDA = 8;
const uint8_t SENSOR1_SCL = 9;
const uint8_t SENSOR2_SDA = A4;
const uint8_t SENSOR2_SCL = A5;

const long CONTACT_DETECTED_IR = 6000;
const long GAIN_TARGET_LOW_IR = 90000;
const long GAIN_TARGET_HIGH_IR = 225000;
const byte INITIAL_IR_LED_CURRENT = 0x30;
const byte MIN_IR_LED_CURRENT = 0x08;
const byte MAX_IR_LED_CURRENT = 0x3F;
const uint16_t SAMPLE_PERIOD_MS = 10; // MAX30102 is configured for 100 Hz.
const uint16_t GAIN_ADJUST_INTERVAL_MS = 350;
const uint16_t GAIN_SETTLE_MS = 2500;
const uint16_t MIN_BEAT_INTERVAL_MS = 300;
const uint16_t MAX_BEAT_INTERVAL_MS = 2000;

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
    long threshold;
    unsigned long sampleTime;
    unsigned long lastBeatTime;
    unsigned long beatFlashUntil;
    unsigned long lastGainAdjust;
    unsigned long gainStableSince;
    uint16_t bpm;
    uint16_t intervals[4];
    byte intervalCount;
    byte intervalIndex;

    HeartChannel(uint8_t sda, uint8_t scl)
        : sensor(sda, scl), state(NO_CONTACT),
          irLedCurrent(INITIAL_IR_LED_CURRENT), dcEstimate(0), wave(0),
          positivePeak(0), threshold(120), sampleTime(0), lastBeatTime(0),
          beatFlashUntil(0), lastGainAdjust(0), gainStableSince(0), bpm(0),
          intervalCount(0), intervalIndex(0)
    {
    }

    void enterNoContact()
    {
        state = NO_CONTACT;
        dcEstimate = 0;
        wave = 0;
        positivePeak = 0;
        threshold = 120;
        sampleTime = 0;
        lastBeatTime = 0;
        beatFlashUntil = 0;
        bpm = 0;
        intervalCount = 0;
        intervalIndex = 0;
    }

    void enterCalibration(long ir)
    {
        state = CALIBRATING;
        dcEstimate = ir << 4;
        wave = 0;
        positivePeak = 0;
        threshold = 120;
        sampleTime = millis();
        lastBeatTime = 0;
        beatFlashUntil = 0;
        bpm = 0;
        intervalCount = 0;
        intervalIndex = 0;
        lastGainAdjust = 0;
        gainStableSince = millis();
    }

    void enterTracking()
    {
        state = TRACKING;
        wave = 0;
        positivePeak = 0;
        threshold = 120;
        lastBeatTime = 0;
        intervalCount = 0;
        intervalIndex = 0;
        bpm = 0;
    }

    void adjustGain()
    {
        if (millis() - lastGainAdjust < GAIN_ADJUST_INTERVAL_MS)
            return;
        lastGainAdjust = millis();

        long dcLevel = dcEstimate >> 4;
        byte nextCurrent = irLedCurrent;
        if (dcLevel < GAIN_TARGET_LOW_IR && irLedCurrent < MAX_IR_LED_CURRENT)
        {
            nextCurrent = irLedCurrent + 2;
            if (nextCurrent > MAX_IR_LED_CURRENT)
                nextCurrent = MAX_IR_LED_CURRENT;
        }
        else if (dcLevel > GAIN_TARGET_HIGH_IR && irLedCurrent > MIN_IR_LED_CURRENT)
        {
            nextCurrent = irLedCurrent - 2;
            if (nextCurrent < MIN_IR_LED_CURRENT)
                nextCurrent = MIN_IR_LED_CURRENT;
        }

        if (nextCurrent == irLedCurrent)
            return;

        if (sensor.setIRLedAmplitude(nextCurrent))
        {
            irLedCurrent = nextCurrent;
            gainStableSince = millis();
        }
    }

    void recordBeat(unsigned long interval)
    {
        intervals[intervalIndex] = interval;
        intervalIndex = (intervalIndex + 1) % 4;
        if (intervalCount < 4)
            ++intervalCount;

        unsigned long sum = 0;
        for (byte i = 0; i < intervalCount; ++i)
            sum += intervals[i];
        bpm = 60000UL / (sum / intervalCount);
        beatFlashUntil = millis() + 120;
    }

    void process(long ir)
    {
        if (ir < CONTACT_DETECTED_IR)
        {
            if (state != NO_CONTACT)
                enterNoContact();
            return;
        }

        if (state == NO_CONTACT)
            enterCalibration(ir);

        sampleTime += SAMPLE_PERIOD_MS;

        if (state == CALIBRATING)
        {
            // A relatively fast baseline is useful here because the LED gain
            // changes during calibration. BPM is intentionally disabled.
            dcEstimate += ((ir << 4) - dcEstimate) >> 4;
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

        if (wave > positivePeak)
            positivePeak = wave;

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

            long nextThreshold = positivePeak / 2;
            if (nextThreshold < 80)
                nextThreshold = 80;
            threshold = (threshold * 3 + nextThreshold) / 4;
            positivePeak = 0;
        }
    }

    int plotState() const
    {
        return state == NO_CONTACT ? 0 : (state == CALIBRATING ? 1 : 2);
    }
};

HeartChannel participant1(SENSOR1_SDA, SENSOR1_SCL);
HeartChannel participant2(SENSOR2_SDA, SENSOR2_SCL);

long latestIr1 = 0;
long latestIr2 = 0;
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
        Serial.println(F("LED setup failed"));
        return false;
    }

    Serial.println(F("ready"));
    return true;
}

void readChannel(HeartChannel &channel, long &latestIr)
{
    byte count = channel.sensor.getFIFOCount();
    for (byte i = 0; i < count; ++i)
    {
        long ir = channel.sensor.getIR();
        if (ir >= 0)
        {
            latestIr = ir;
            channel.process(ir);
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

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
    readChannel(participant1, latestIr1);
    readChannel(participant2, latestIr2);

    if (millis() - lastPlotMs < 40)
        return;
    lastPlotMs = millis();

    // Clean plot output. State: 0 = no contact, 1 = calibrating, 2 = tracking.
    Serial.print(F("Wave1:")); Serial.print(participant1.wave);
    Serial.print(F(" Beat1:")); Serial.print(millis() < participant1.beatFlashUntil ? 2500 : 0);
    Serial.print(F(" BPM1:")); Serial.print(participant1.bpm);
    Serial.print(F(" State1:")); Serial.print(participant1.plotState());
    Serial.print(F(" Wave2:")); Serial.print(participant2.wave);
    Serial.print(F(" Beat2:")); Serial.print(millis() < participant2.beatFlashUntil ? 2500 : 0);
    Serial.print(F(" BPM2:")); Serial.print(participant2.bpm);
    Serial.print(F(" State2:")); Serial.println(participant2.plotState());
}
