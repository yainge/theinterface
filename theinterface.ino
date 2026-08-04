#include "InterfaceSensor.h"

// Open Tools > Serial Plotter at 115200 baud after uploading. Each label is
// plotted as its own trace: IR is scaled down, Wave is the DC-removed pulse
// signal, Beat briefly rises when a beat is found, and BPM is the latest rate.
const uint8_t SENSOR1_SDA = 8;
const uint8_t SENSOR1_SCL = 9;
const uint8_t SENSOR2_SDA = A4;
const uint8_t SENSOR2_SCL = A5;
const long FINGER_PRESENT_IR = 20000;
const uint16_t SAMPLE_PERIOD_MS = 10; // MAX30102 is configured for 100 Hz.
const uint16_t MIN_BEAT_INTERVAL_MS = 300;
const uint16_t MAX_BEAT_INTERVAL_MS = 2000;

struct HeartChannel
{
    InterfaceSensor sensor;
    long dcEstimate;
    long ac;
    long positivePeak;
    long threshold;
    unsigned long sampleTime;
    unsigned long lastBeatTime;
    unsigned long beatFlashUntil;
    uint16_t bpm;
    bool initialized;

    HeartChannel(uint8_t sda, uint8_t scl)
        : sensor(sda, scl), dcEstimate(0), ac(0), positivePeak(0),
          threshold(120), sampleTime(0), lastBeatTime(0),
          beatFlashUntil(0), bpm(0), initialized(false)
    {
    }

    void clear()
    {
        dcEstimate = 0;
        ac = 0;
        positivePeak = 0;
        threshold = 120;
        sampleTime = 0;
        lastBeatTime = 0;
        beatFlashUntil = 0;
        bpm = 0;
        initialized = false;
    }

    void process(long ir)
    {
        if (ir < FINGER_PRESENT_IR)
        {
            clear();
            return;
        }

        if (!initialized)
        {
            dcEstimate = ir << 4; // Fixed-point value with 4 fractional bits.
            sampleTime = millis();
            initialized = true;
            return;
        }

        sampleTime += SAMPLE_PERIOD_MS;
        long previousAc = ac;
        dcEstimate += ((ir << 4) - dcEstimate) >> 5; // Slow DC baseline.
        ac = ir - (dcEstimate >> 4);

        if (ac > positivePeak)
            positivePeak = ac;

        // A positive lobe ends at the downward zero crossing. Its peak must
        // clear an adaptive threshold and the beat must be physiologically
        // plausible before it contributes to BPM.
        if (previousAc > 0 && ac <= 0)
        {
            if (positivePeak > threshold && lastBeatTime != 0)
            {
                unsigned long interval = sampleTime - lastBeatTime;
                if (interval >= MIN_BEAT_INTERVAL_MS && interval <= MAX_BEAT_INTERVAL_MS)
                {
                    uint16_t instantaneousBpm = 60000UL / interval;
                    bpm = bpm == 0 ? instantaneousBpm : (bpm * 3 + instantaneousBpm) / 4;
                    beatFlashUntil = millis() + 120;
                    lastBeatTime = sampleTime;
                }
            }
            else if (positivePeak > threshold)
            {
                lastBeatTime = sampleTime;
            }

            long nextThreshold = positivePeak / 2;
            if (nextThreshold < 80)
                nextThreshold = 80;
            threshold = (threshold * 3 + nextThreshold) / 4;
            positivePeak = 0;
        }
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

    // 25 samples/s keeps the serial stream readable and the plot responsive.
    if (millis() - lastPlotMs < 40)
        return;
    lastPlotMs = millis();

    Serial.print(F("IR1:")); Serial.print(latestIr1 / 100);
    Serial.print(F(" Wave1:")); Serial.print(participant1.ac);
    Serial.print(F(" Beat1:")); Serial.print(millis() < participant1.beatFlashUntil ? 2500 : 0);
    Serial.print(F(" BPM1:")); Serial.print(participant1.bpm);
    Serial.print(F(" IR2:")); Serial.print(latestIr2 / 100);
    Serial.print(F(" Wave2:")); Serial.print(participant2.ac);
    Serial.print(F(" Beat2:")); Serial.print(millis() < participant2.beatFlashUntil ? 2500 : 0);
    Serial.print(F(" BPM2:")); Serial.println(participant2.bpm);
}
