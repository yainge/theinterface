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

// A0 button: cycles the LED pattern shown on both strips (see CLAUDE.md's
// pin table). Active-LOW, INPUT_PULLUP -- same convention as
// HardwareTest.ino's PIN_BUTTON, but debounced non-blockingly here since
// loop() must keep servicing sensors/motors every iteration.
const uint8_t PIN_BUTTON = A0;

// A2 potentiometer: overall "intensity" knob (see CLAUDE.md's pin table)
// -- scales both motor peak strength (HeartChannel::motorIntensity()) and
// LED brightness (LedState::update()) together, from fully off (0) to
// full strength (1.0). Low-pass filtered the same way SPO2_SMOOTHING
// smooths the SpO2 estimate, so raw ADC jitter doesn't make the motors
// stutter or the LEDs flicker.
const uint8_t PIN_POT = A2;
const float POT_SMOOTHING = 0.1f;
float intensity = 1.0f; // 0.0-1.0, updated once per loop() by updateIntensity() (defined
                         // further down, near startSensor()/readChannel() -- kept off this
                         // early in the file since it'd otherwise become the first free
                         // function in the sketch, which is where Arduino's auto-generated
                         // prototypes get inserted; too early for HeartChannel to exist yet)

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
// heartbeat. MOTOR_PEAK_INTENSITY is the envelope's ceiling at full
// potentiometer position (1.0) -- see HeartChannel::motorIntensity() and
// updateIntensity(), which scale it down live from the A2 pot.
const uint16_t LOBE1_ATTACK_MS = 30;
const uint16_t LOBE1_DECAY_MS = 90;
const uint16_t LOBE_GAP_MS = 120;     // silence between the two lobes
const uint16_t LOBE2_ATTACK_MS = 25;
const uint16_t LOBE2_DECAY_MS = 90;
const float LOBE2_PEAK_SCALE = 0.7;   // second lobe a bit softer than the first, like a real "dub"
const byte MOTOR_PEAK_INTENSITY = 180; // 0-255 analogWrite duty cycle

// EXPERIMENTAL (this branch only): felt/visual feedback -- both the motor's
// pulse and the LED beat reaction -- starts by following each raw detected
// beat, then after this long in TRACKING switches to a steady pulse timed
// off the rolling BPM average instead of individual beat jitter. This is
// what keeps the motor and LEDs pulsing together, in time with each other,
// and immune to an occasional misread/missed raw beat once locked in.
// BPM/HRV/SpO2 computation is unaffected either way -- this only changes
// what schedules pulseTriggerTime (see below), which both the motor's
// two-tone envelope and the LED beat reaction key off of.
const unsigned long PULSE_STEADY_TRANSITION_MS = 7000;

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
    unsigned long pulseTriggerTime; // shared trigger instant for both the motor's envelope and
                                     // the LED beat reaction -- see PULSE_STEADY_TRANSITION_MS
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
          beatFlashUntil(0), lastGainAdjust(0), gainStableSince(0), pulseTriggerTime(0),
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
        pulseTriggerTime = 0;
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
        // (re)starts, e.g. after a contact loss -- see PULSE_STEADY_TRANSITION_MS.
        trackingStartTime = millis();
        nextSteadyPulseTime = 0;
    }

    // True once PULSE_STEADY_TRANSITION_MS has elapsed in TRACKING and a
    // BPM estimate exists to time a steady pulse off of.
    bool inSteadyPulseMode() const
    {
        return state == TRACKING && bpm > 0 &&
               (millis() - trackingStartTime) >= PULSE_STEADY_TRANSITION_MS;
    }

    // EXPERIMENTAL: once in steady mode, fires the shared motor+LED pulse
    // on a fixed schedule derived from the rolling BPM average instead of
    // individual raw beat detections. Call once per loop() iteration
    // (time-based, not tied to FIFO sample processing). Naturally pauses
    // (does nothing, sets no new pulseTriggerTime) whenever this channel
    // isn't TRACKING -- e.g. no finger on the sensor -- so losing contact
    // silences both the motor and the LED beat reaction rather than
    // leaving them pulsing on a stale schedule.
    void updatePulseSchedule()
    {
        if (state != TRACKING)
            return;

        if (!inSteadyPulseMode())
        {
            nextSteadyPulseTime = 0; // re-arm so the first steady pulse fires promptly on transition
            return;
        }

        unsigned long now = millis();
        if (nextSteadyPulseTime == 0 || now >= nextSteadyPulseTime)
        {
            pulseTriggerTime = now;
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
    // time since pulseTriggerTime, so it can be sampled every loop()
    // iteration for a smooth envelope without extra per-loop bookkeeping.
    // What sets pulseTriggerTime (raw beat detection vs. the steady BPM
    // schedule) is decided elsewhere; this only shapes each pulse.
    // `intensity` (0.0-1.0, from the potentiometer -- see updateIntensity())
    // scales MOTOR_PEAK_INTENSITY down uniformly; both lobes stay
    // proportional to each other (LOBE2_PEAK_SCALE applies on top).
    byte motorIntensity(float intensity) const
    {
        if (pulseTriggerTime == 0)
            return 0;

        byte peak = (byte)(MOTOR_PEAK_INTENSITY * intensity);

        unsigned long elapsed = millis() - pulseTriggerTime;
        const uint16_t lobe1Duration = LOBE1_ATTACK_MS + LOBE1_DECAY_MS;

        if (elapsed < lobe1Duration)
            return lobeIntensity(elapsed, LOBE1_ATTACK_MS, LOBE1_DECAY_MS, peak);

        elapsed -= lobe1Duration;
        if (elapsed < LOBE_GAP_MS)
            return 0;

        elapsed -= LOBE_GAP_MS;
        byte lobe2Peak = (byte)(peak * LOBE2_PEAK_SCALE);
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

                // Diagnostic-only marker for the Serial "Beat" field
                // (printChannel()) -- fires on every real detected peak,
                // even one whose interval got rejected above for BPM/HRV
                // purposes, so it reflects raw sensor detection regardless
                // of what's currently driving the felt/visual pulse below.
                beatFlashUntil = millis() + 120;

                // EXPERIMENTAL: once in steady mode, updatePulseSchedule()
                // owns pulseTriggerTime instead of raw beat detection --
                // this is what the motor's envelope and the LED beat
                // reaction actually key off of (see checkBeat()).
                if (!inSteadyPulseMode())
                    pulseTriggerTime = millis();
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
// 10 selectable patterns, ported 1:1 from the LEDSimulator3D.html design
// tool (see that file for the original prototype + design commentary this
// port follows). The A0 button cycles currentPattern for both strips at
// once; each pattern owns its own per-pixel color math and (mostly) its
// own state on LedState below.
//
// Timing: each strip is re-rendered at LED_UPDATE_MS (20 ms / 50 fps),
// independent of the sensor FIFO poll rate (40 ms). NeoPixel show()
// disables interrupts for ~1.3 ms (44 LEDs × 30 µs); SoftWire is purely
// software-timed so they don't interfere — just keep show() out of the
// middle of readChannel().
//
// Float-only, same rule as the rest of this file (see the SPO2_RATIO_*
// comment above and computeRMSSD()'s sqrtf): sinf/cosf/fmodf/fabsf, never
// sin/cos/fmod/fabs, and TWO_PI_F below instead of Arduino's double PI/
// TWO_PI macros, so the linker never has to pull in software double math.
//
// Timestamps (wave/eclipse/ring/flower/plant-fade-out start times) are
// unsigned long millis(), never float: a float's 24-bit mantissa silently
// loses millisecond precision past ~4.66 hours, which would corrupt every
// elapsed-time calculation on a long-running installation. 0 means
// "inactive", matching this file's existing pulseTriggerTime==0 idiom.
// Purely-cosmetic continuous-phase math (hue drift, breathing brightness)
// instead wraps "now" via PHASE_WRAP_MS to stay float-precision-safe
// forever without needing a timestamp at all.

struct RGBf { float r, g, b; };

// Forward declaration -- Arduino's auto-generated function prototypes get
// inserted right before the first free function below (hsvToRgb()), which
// is textually before the real LedState struct is defined further down.
// Without this, every pattern function taking `LedState &` fails to
// compile with "'LedState' was not declared in this scope" because the
// auto-inserted prototype can't see it yet; an incomplete forward
// declaration is enough for a reference/pointer parameter.
struct LedState;

const float TWO_PI_F = 6.283185307f;
const unsigned long PHASE_WRAP_MS = 3600000UL; // 1 hour; an exact multiple of every
                                                // periodic pattern's period below, so
                                                // wrapping "now" through this never
                                                // shifts phase, just keeps the float
                                                // math in its exact-integer range.

// hue: degrees (any range, wrapped), s/v: 0-1 -> r/g/b 0-255. Direct port
// of LEDSimulator3D.html's hsvToRgb().
RGBf hsvToRgb(float h, float s, float v)
{
    h = fmodf(h, 360.0f);
    if (h < 0) h += 360.0f;
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r = 0, g = 0, b = 0;
    if      (h <  60.0f) { r = c; g = x; b = 0; }
    else if (h < 120.0f) { r = x; g = c; b = 0; }
    else if (h < 180.0f) { r = 0; g = c; b = x; }
    else if (h < 240.0f) { r = 0; g = x; b = c; }
    else if (h < 300.0f) { r = x; g = 0; b = c; }
    else                 { r = c; g = 0; b = x; }
    return { (r + m) * 255.0f, (g + m) * 255.0f, (b + m) * 255.0f };
}

float lerpf(float a, float b, float t) { return a + (b - a) * t; }

// Clamp+cast a float 0-255 channel to a byte for strip.setPixelColor().
uint8_t toByte(float v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

float randomFloat01() { return (float)random(0, 10001) / 10000.0f; }
float randomHueInRange(float lo, float hi) { return lo + randomFloat01() * (hi - lo); }
uint16_t bpmDelta(uint16_t a, uint16_t b) { return a > b ? a - b : b - a; }

// Matches LEDSimulator3D.html's patNames order exactly.
enum LedPattern : uint8_t
{
    PATTERN_RED_BLUE = 0, PATTERN_CHASE, PATTERN_RAINBOW_WAVE, PATTERN_LAVA,
    PATTERN_ECLIPSE, PATTERN_RINGS, PATTERN_PLANT, PATTERN_SLOW_RAINBOW,
    PATTERN_SPARKLE, PATTERN_AURORA, NUM_PATTERNS
};
const char *const patternNames[NUM_PATTERNS] = {
    "Red-Blue", "Chase", "Rainbow Wave", "Lava", "Eclipse",
    "Rings", "Plant", "Slow Rainbow", "Sparkle", "Aurora"
};
LedPattern currentPattern = PATTERN_RED_BLUE; // shared by both strips -- the button cycles this

// ── Modes ────────────────────────────────────────────────────────────────
// A separate, higher-level layer from the 10 LED patterns above: each mode
// changes *what drives* the felt/visual pulse (real sensor vs. one of two
// fixed synthetic rates, currently), while the button's single-press
// pattern cycling keeps working underneath it. LED index 0 on both strips
// is reserved to show which mode is active (see LedState::update()) --
// intentionally NOT scaled by `intensity`, so it stays legible even with
// the brightness knob turned low. Triple-pressing the A0 button within
// MODE_TRIPLE_CLICK_WINDOW_MS cycles modes (see pollButton()/cycleMode());
// see pollButton()'s comment for how that's kept from also net-cycling the
// LED pattern.
//
// MODE_YELLOW doesn't have a real name yet -- behavior-wise it's now
// defined (see SYNTHETIC_BPM below), just not named/designed visually
// beyond reusing the same 10-pattern engine Breath Entrainment does.
enum Mode : uint8_t { MODE_NORMAL = 0, MODE_BREATH_ENTRAINMENT, MODE_YELLOW, NUM_MODES };
const char *const modeNames[NUM_MODES] = { "Normal", "Breath Entrainment", "Yellow" };
const RGBf MODE_INDICATOR_COLOR[NUM_MODES] = { {0, 0, 0}, {0, 0, 255}, {255, 255, 0} };
Mode currentMode = MODE_NORMAL; // shared by both strips

// Synthetic-clock modes: an artificial, sensor-independent heartbeat
// shared by both participants at a fixed rate, meant to pull both people
// toward it rather than reflect either of their real heartrates -- see
// updateSyntheticClock() and loop()'s branch on `currentMode != MODE_NORMAL`,
// which skips real sensor reads entirely while either is active. Indexed
// by Mode; MODE_NORMAL's entry is unused (real sensor data drives it
// instead). Breath Entrainment runs slow, meant to draw breathing/heart
// rate down; Yellow runs at a common resting-heartrate pace -- same
// mechanism, just a different target rate.
const uint16_t SYNTHETIC_BPM[NUM_MODES] = { 0, 6, 60 };

// Pattern 0 (Red-Blue): per-pixel hue within each strip's own theme range.
const uint8_t SPARKLE_BEAT_COUNT = 14;
const float SPARKLE_HUE_RANGE_1[2] = {0.0f, 35.0f};    // red -> orange (strip 1)
const float SPARKLE_HUE_RANGE_2[2] = {210.0f, 285.0f}; // blue -> purple (strip 2)

// Pattern 1 (Chase).
const RGBf CHASE_COLOR = {255, 0, 0};
const float CHASE_SPEED_PER_FRAME = 1.2f;
const float CHASE_GLOW_PEAK = 80.0f;
const float CHASE_GLOW_FALLOFF = 12.0f;

// Pattern 2 (Rainbow Wave). WAVE_MAX_ACTIVE is a fixed-capacity stand-in
// for the JS's unbounded waveStarts[] -- 8 concurrent waves is generous
// against WAVE_TRAVEL_MS and realistic beat cadence; a beat landing with
// every slot full is simply dropped.
const uint16_t WAVE_TRAVEL_MS = 900;
const uint8_t WAVE_TAIL_LEN = 10;
const uint8_t WAVE_MAX_ACTIVE = 8;

// Pattern 3 (Lava).
const uint16_t LAVA_UPDATE_MS = 90;
const float LAVA_COOLING = 120.0f;
const uint8_t LAVA_BEAT_BURST = 4;

// Pattern 4 (Eclipse).
const uint16_t ECLIPSE_TRAVEL_MS = 1750;
const uint8_t ECLIPSE_UMBRA_RADIUS = 6;
const uint8_t ECLIPSE_CORONA_WIDTH = 4;
const RGBf ECLIPSE_SUN_COLOR = {70, 90, 140};
const RGBf ECLIPSE_CORONA_COLOR = {255, 255, 255};
const RGBf ECLIPSE_SHADOW_COLOR = {0, 0, 0};
const RGBf ECLIPSE_SYNC_SUN_COLOR = {230, 150, 30};
const uint8_t ECLIPSE_SYNC_THRESHOLD = 4;
const uint16_t ECLIPSE_SYNC_COLOR_MS = 3000;

// Pattern 5 (Rings).
const uint16_t RING_TRAVEL_MS = 700;
const uint8_t RING_WIDTH = 6;
const RGBf RING_COLOR = {255, 255, 255};

// Pattern 6 (Plant).
const uint8_t PLANT_MATURE_BRIGHTNESS = 150;
const float PLANT_DECAY = 6.0f;
const float PLANT_AGE_DECAY = 0.1f;
const uint8_t PLANT_AGED_FLOOR = 25;
const float PLANT_HUE_MIN = 95.0f, PLANT_HUE_MAX = 140.0f;
const RGBf VIOLET_COLOR = {170, 60, 230};
const uint8_t PLANT_SYNC_THRESHOLD = 4;
const uint8_t PLANT_FLOWER_COUNT = 1;
const uint16_t FLOWER_FADE_IN_MS = 300;
const uint16_t FLOWER_HOLD_MS = 600;
const uint16_t FLOWER_FADE_OUT_MS = 900;
const uint16_t PLANT_FADEOUT_MS = 2000;

// Pattern 7 (Slow Rainbow).
const float SLOW_RAINBOW_HUE_SPEED = 20.0f;
const uint16_t SLOW_RAINBOW_REFERENCE_BPM = 70;
const uint16_t SLOW_RAINBOW_FADE_MS = 6000;
const uint8_t SLOW_RAINBOW_MIN_BRIGHTNESS = 20;
const uint8_t SLOW_RAINBOW_MAX_BRIGHTNESS = 180;
const float SLOW_RAINBOW_BEAT_KICK = 150.0f;

// Pattern 8 (Sparkle/Twinkle) -- separate from pattern 0's sparkle state.
// Purely beat-reactive now (see updateTwinkle()) -- no ambient-rate
// constants needed, just the beat-triggered burst's size/color/brightness.
const float TWINKLE_HUE_MIN = 42.0f, TWINKLE_HUE_MAX = 58.0f;
const uint8_t TWINKLE_BEAT_COUNT = 3;
const uint8_t TWINKLE_BEAT_BRIGHTNESS = 255;

// Pattern 9 (Aurora).
const float AURORA_WAVE_SPEED = 0.3f;
const uint8_t AURORA_BASE_BRIGHTNESS = 40;
const uint16_t AURORA_INTENSITY_REFERENCE_BPM = 70;
const uint8_t AURORA_SYNC_THRESHOLD = 4;
const uint16_t AURORA_SYNC_COLOR_MS = 3000;
const RGBf AURORA_SYNC_COLOR = {230, 60, 160};

struct LedState
{
    uint8_t pxBright[LED_COUNT];
    uint8_t beatPulse;
    unsigned long lastPulseTrigger; // last HeartChannel::pulseTriggerTime seen; used
                                     // to detect a new pulse without re-triggering on
                                     // every loop() iteration. This is the SAME shared
                                     // trigger the motor's envelope keys off of (see
                                     // PULSE_STEADY_TRANSITION_MS), not the raw-only
                                     // beatFlashUntil -- so the LED beat reaction stays
                                     // in lockstep with the motor and, once in steady
                                     // mode, survives an occasional misread/missed raw
                                     // beat instead of silently skipping a pulse.
    unsigned long lastUpdate;

    // True when this participant's channel is actively TRACKING a
    // heartbeat (see loop()'s update() call sites). Gates the
    // beat-reactive patterns (everything except Slow Rainbow/Sparkle/
    // Aurora) fully dark in getPixelColor() when there's no signal --
    // those three are ambient/idle by design (see DESIGN.md) and ignore
    // this. Per-pattern update() math still runs either way (cheap, and
    // it's what naturally settles Chase/Lava/etc. back to their own idle
    // state once a signal returns) -- only the rendered color is gated.
    bool hasSignal;

    uint8_t sparklePos[NUM_SPARKLES];
    uint8_t sparkleTimer[NUM_SPARKLES]; // counts down; when 0, pick a new position

    // Per-strip identity for patterns that "own" a floor tint (Rainbow
    // Wave, Lava) instead of a full fixed palette -- replaces the old
    // shared BEAT_R/G/B constant now that color varies by pattern. Set
    // once in setup().
    RGBf themeColor;

    // Pattern 0 (Red-Blue).
    float hue[LED_COUNT];
    float hueLo, hueHi;

    // Pattern 1 (Chase).
    float chaseHead;

    // Pattern 2 (Rainbow Wave). 0 = empty slot, else the beat's millis() start time.
    unsigned long waveStart[WAVE_MAX_ACTIVE];

    // Pattern 3 (Lava).
    float heat[LED_COUNT];
    unsigned long lastLavaUpdate;

    // Pattern 4 (Eclipse). 0 = no shadow in flight.
    unsigned long eclipseStart;

    // Pattern 5 (Rings). 0 = no ring in flight.
    unsigned long ringStart;

    // Pattern 6 (Plant).
    uint8_t plantLength;
    float plantBright[LED_COUNT];
    float plantHue[LED_COUNT];
    unsigned long flowerStart[LED_COUNT]; // 0 = no bloom at this LED
    float plantFadeOutFrom[LED_COUNT];
    unsigned long plantFadeOutStart;      // 0 = not currently fading out

    // Pattern 7 (Slow Rainbow).
    float slowRainbowHue;

    // Pattern 8 (Sparkle/Twinkle) -- own state, independent of pattern 0's
    // sparklePos/sparkleTimer/hue above so the two tune separately. Purely
    // beat-reactive (see updateTwinkle()) -- no ambient-timer state needed.
    float twinkleBright[LED_COUNT];
    float twinkleHue[LED_COUNT];

    // Pattern 9 (Aurora).
    float auroraPhase;
    float auroraIntensity;

    void begin()
    {
        for (uint8_t i = 0; i < LED_COUNT; i++)
            pxBright[i] = LED_AMBIENT_FLOOR;
        beatPulse = 0;
        lastPulseTrigger = 0;
        lastUpdate = 0;
        hasSignal = false;
        for (uint8_t i = 0; i < NUM_SPARKLES; i++)
        {
            sparklePos[i] = random(LED_COUNT);
            sparkleTimer[i] = random(10, 40);
        }

        // Fields resetPattern() deliberately leaves alone -- seeded once
        // here (after the caller sets hueLo/hueHi/themeColor) so a pattern
        // switch away and back doesn't reset them.
        for (uint8_t i = 0; i < LED_COUNT; i++)
            hue[i] = randomHueInRange(hueLo, hueHi);
        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
            twinkleHue[i] = randomHueInRange(TWINKLE_HUE_MIN, TWINKLE_HUE_MAX);
            plantHue[i] = PLANT_HUE_MIN;
        }
        slowRainbowHue = 0;
        auroraPhase = (float)random(0, 1000); // random offset so both strips' curtains don't move in lockstep
        auroraIntensity = 1;

        resetPattern();
    }

    // Mirrors LEDSimulator3D.html's setPattern() reset block exactly --
    // called from begin() and on every button-triggered pattern change.
    // Deliberately does NOT reset hue[]/sparklePos/sparkleTimer/
    // twinkleHue/plantHue/slowRainbowHue/auroraPhase/auroraIntensity --
    // e.g. Aurora's per-strip phase offset must survive switching away
    // and back.
    void resetPattern()
    {
        for (uint8_t i = 0; i < LED_COUNT; i++)
            pxBright[i] = LED_AMBIENT_FLOOR;
        beatPulse = 0;
        chaseHead = 0;
        for (uint8_t i = 0; i < WAVE_MAX_ACTIVE; i++)
            waveStart[i] = 0;
        for (uint8_t i = 0; i < LED_COUNT; i++)
            heat[i] = 0;
        eclipseStart = 0;
        ringStart = 0;
        plantLength = 0;
        for (uint8_t i = 0; i < LED_COUNT; i++)
            plantBright[i] = LED_AMBIENT_FLOOR;
        for (uint8_t i = 0; i < LED_COUNT; i++)
            flowerStart[i] = 0;
        plantFadeOutStart = 0;
        for (uint8_t i = 0; i < LED_COUNT; i++)
            twinkleBright[i] = 0;
    }

    // Call once per loop() — detects a newly set HeartChannel::pulseTriggerTime
    // and dispatches this pattern's own beat reaction (reactToBeat(),
    // defined below once the per-pattern functions it calls are all in
    // scope). pulseTriggerTime == 0 means "no pulse" (no contact, or
    // contact just lost -- see HeartChannel::resetBeatState()), so this
    // naturally stays silent whenever the sensor isn't reading a signal.
    void checkBeat(unsigned long pulseTriggerTime, uint16_t myBpm, uint16_t otherBpm)
    {
        if (pulseTriggerTime != 0 && pulseTriggerTime != lastPulseTrigger)
        {
            beatPulse = 255;
            reactToBeat(myBpm, otherBpm);
            lastPulseTrigger = pulseTriggerTime;
        }
    }

    bool needsUpdate() const { return millis() - lastUpdate >= LED_UPDATE_MS; }

    // `intensity` (0.0-1.0, from the potentiometer -- see updateIntensity())
    // scales every pattern's output uniformly, applied once here at the
    // very end of the render pipeline rather than in each pattern's own
    // color function -- so it covers ambient glow and beat flashes alike,
    // for every pattern, with one multiply instead of touching all ten.
    void update(Adafruit_NeoPixel &strip, uint16_t myBpm, uint16_t otherBpm, bool active, float intensity)
    {
        lastUpdate = millis();
        hasSignal = active;

        if (beatPulse > LED_BEAT_DECAY)
            beatPulse -= LED_BEAT_DECAY;
        else
            beatPulse = 0;

        updatePattern(myBpm, otherBpm);

        for (uint8_t i = 0; i < LED_COUNT; i++)
        {
            RGBf c = getPixelColor(i);
            strip.setPixelColor(i, strip.Color(
                toByte(c.r * intensity), toByte(c.g * intensity), toByte(c.b * intensity)));
        }

        // LED 0 is reserved as the mode indicator -- overwritten after the
        // normal per-pixel loop above so every pattern can keep computing
        // index 0 like any other pixel without needing to special-case it.
        // Not scaled by `intensity` -- see the "Modes" comment block above.
        RGBf modeColor = MODE_INDICATOR_COLOR[currentMode];
        strip.setPixelColor(0, strip.Color(toByte(modeColor.r), toByte(modeColor.g), toByte(modeColor.b)));

        strip.show();
    }

    // Defined out-of-line below, after the per-pattern update/color
    // functions they dispatch to (C++ allows this: a class's own member
    // function bodies can reference other members declared later in the
    // same class).
    void updatePattern(uint16_t myBpm, uint16_t otherBpm);
    RGBf getPixelColor(uint8_t i) const;
    void reactToBeat(uint16_t myBpm, uint16_t otherBpm);
};

Adafruit_NeoPixel strip1(LED_COUNT, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED_COUNT, LED_PIN_2, NEO_GRB + NEO_KHZ800);

LedState leds1, leds2;

// ── Per-pattern update/color functions ──────────────────────────────────────
// One update + one pixel-color function per pattern, named 1:1 from
// LEDSimulator3D.html so the port is traceable line-for-line against it.

// Pattern 0: Red-Blue.
void updateSparkle(LedState &state)
{
    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        if (state.pxBright[i] > LED_AMBIENT_FLOOR + LED_PIXEL_DECAY)
            state.pxBright[i] -= LED_PIXEL_DECAY;
        else
            state.pxBright[i] = LED_AMBIENT_FLOOR;
    }
    // Ambient twinkles pause while a beat is still in flight, so new random
    // pops don't visually compete with the beat's own random assortment.
    if (state.beatPulse == 0)
    {
        for (uint8_t i = 0; i < NUM_SPARKLES; i++)
        {
            if (state.sparkleTimer[i] == 0)
            {
                uint8_t idx = random(LED_COUNT);
                state.sparklePos[i] = idx;
                state.pxBright[idx] = random(LED_SPARKLE_PEAK / 2, LED_SPARKLE_PEAK);
                state.hue[idx] = randomHueInRange(state.hueLo, state.hueHi);
                state.sparkleTimer[i] = random(15, 50);
            }
            else
            {
                state.sparkleTimer[i]--;
            }
        }
    }
}

RGBf sparklePixelColor(const LedState &state, uint8_t i)
{
    return hsvToRgb(state.hue[i], 1.0f, state.pxBright[i] / 255.0f);
}

// Pattern 1: Chase.
void updateChase(LedState &state)
{
    if (state.chaseHead < LED_COUNT)
        state.chaseHead += CHASE_SPEED_PER_FRAME;
}

RGBf chasePixelColor(const LedState &state, uint8_t i)
{
    // No ambient floor -- purely reactive, per the refined "off except
    // for heart pulses" spec. Once chaseHead has swept past a pixel's
    // falloff radius (glow reaches 0), that pixel is fully black rather
    // than resting at LED_AMBIENT_FLOOR.
    float dist = fabsf((float)i - state.chaseHead);
    float glow = max(0.0f, CHASE_GLOW_PEAK - dist * CHASE_GLOW_FALLOFF);
    float b = min(255.0f, glow);
    return { CHASE_COLOR.r * b / 255.0f, CHASE_COLOR.g * b / 255.0f, CHASE_COLOR.b * b / 255.0f };
}

// Pattern 2: Rainbow Wave. Each beat launches its own wavefront at index 0
// that sweeps to LED_COUNT-1 over WAVE_TRAVEL_MS, towing a short rainbow
// tail; multiple beats' waves can be in flight at once (see waveStart[]
// above). hueShift is a pure function of "now" (via PHASE_WRAP_MS), so it's
// computed inline here rather than cached per-strip.
void updateRainbowWave(LedState &state)
{
    for (uint8_t k = 0; k < WAVE_MAX_ACTIVE; k++)
    {
        if (state.waveStart[k] == 0) continue;
        unsigned long elapsed = state.lastUpdate - state.waveStart[k];
        float front = ((float)elapsed / WAVE_TRAVEL_MS) * (LED_COUNT - 1);
        if (front >= (float)(LED_COUNT - 1) + WAVE_TAIL_LEN)
            state.waveStart[k] = 0; // this wave's tail has fully cleared the strip
    }
}

RGBf rainbowPixelColor(const LedState &state, uint8_t i)
{
    // No ambient floor tint -- purely reactive, per the refined "off
    // except for heart pulses" spec. Pixels outside every active wave's
    // tail stay fully black rather than resting at a dim themeColor tint.
    float r = 0, g = 0, b = 0;

    float nowPhase = (float)(state.lastUpdate % PHASE_WRAP_MS);
    float hueShift = fmodf(nowPhase * 0.05f, 360.0f);

    for (uint8_t k = 0; k < WAVE_MAX_ACTIVE; k++)
    {
        if (state.waveStart[k] == 0) continue;
        unsigned long elapsed = state.lastUpdate - state.waveStart[k];
        float front = ((float)elapsed / WAVE_TRAVEL_MS) * (LED_COUNT - 1);
        float dist = front - i;
        if (dist >= 0 && dist < WAVE_TAIL_LEN)
        {
            float brightness = 1.0f - dist / WAVE_TAIL_LEN;
            float hue = fmodf((dist / WAVE_TAIL_LEN) * 360.0f + hueShift, 360.0f);
            RGBf c = hsvToRgb(hue, 1.0f, brightness);
            r = max(r, c.r); g = max(g, c.g); b = max(b, c.b);
        }
    }
    return { r, g, b };
}

// Pattern 3: Lava. Fixed black -> red -> orange -> yellow -> white palette
// driven by heat (0-255); a classic fire-sim heat-diffusion loop. Each beat
// is the only heat source (see reactToBeat()) -- no ambient random
// ignition, so a strip with no recent beats cools to black and stays
// there. Throttled to LAVA_UPDATE_MS so the flow reads as slow-moving
// rather than a fast flicker; rendering still runs every frame.
RGBf lavaColor(float heat)
{
    float t = min(1.0f, heat / 255.0f);
    float hue = 8.0f + t * 42.0f;
    float sat = t < 0.85f ? 1.0f : 1.0f - (t - 0.85f) / 0.15f * 0.6f;
    float val = min(1.0f, t * 1.3f);
    return hsvToRgb(hue, sat, val);
}

void updateLava(LedState &state)
{
    if (state.lastUpdate - state.lastLavaUpdate < LAVA_UPDATE_MS)
        return;
    state.lastLavaUpdate = state.lastUpdate;

    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        float cooldown = randomFloat01() * ((LAVA_COOLING * 10.0f) / LED_COUNT);
        state.heat[i] = max(0.0f, state.heat[i] - cooldown);
    }
    // Heat drifts from the source (index 0) toward the far end: walk from
    // the far end down to the source so each LED blends toward values that
    // still hold last frame's state (same trick Fire2012 uses).
    for (uint8_t i = LED_COUNT - 1; i >= 2; i--)
        state.heat[i] = (state.heat[i - 1] + state.heat[i - 1] + state.heat[i - 2]) / 3.0f;
}

RGBf lavaPixelColor(const LedState &state, uint8_t i)
{
    // No ambient floor tint -- purely reactive, per the refined "off
    // except for heart pulses" spec. Heat that's fully cooled (0) renders
    // as true black instead of a dim themeColor tint.
    return lavaColor(state.heat[i]);
}

// Pattern 4: Eclipse. Idles at full brightness (a hazy sun); each beat
// launches a shadow disc whose center sweeps index 0 -> LED_COUNT-1 over
// ECLIPSE_TRAVEL_MS. The sun's own color slowly shifts toward orange while
// the two hearts are in sync (see updateSharedPatternState()'s
// eclipseSunColorNow).
void updateEclipse(LedState &state)
{
    if (state.eclipseStart == 0) return;
    unsigned long elapsed = state.lastUpdate - state.eclipseStart;
    if (elapsed >= ECLIPSE_TRAVEL_MS)
        state.eclipseStart = 0; // settle back to full sun until next beat
}

RGBf eclipsePixelColor(const LedState &state, uint8_t i)
{
    extern RGBf eclipseSunColorNow;
    float r = eclipseSunColorNow.r, g = eclipseSunColorNow.g, b = eclipseSunColorNow.b;

    if (state.eclipseStart != 0)
    {
        unsigned long elapsed = state.lastUpdate - state.eclipseStart;
        if (elapsed < ECLIPSE_TRAVEL_MS)
        {
            float pos = ((float)elapsed / ECLIPSE_TRAVEL_MS) * (LED_COUNT - 1);
            float dist = fabsf((float)i - pos);
            if (dist < ECLIPSE_UMBRA_RADIUS)
            {
                float t = dist / ECLIPSE_UMBRA_RADIUS;
                float k = t * t;
                r = lerpf(ECLIPSE_SHADOW_COLOR.r, eclipseSunColorNow.r, k);
                g = lerpf(ECLIPSE_SHADOW_COLOR.g, eclipseSunColorNow.g, k);
                b = lerpf(ECLIPSE_SHADOW_COLOR.b, eclipseSunColorNow.b, k);
            }
            else if (dist < ECLIPSE_UMBRA_RADIUS + ECLIPSE_CORONA_WIDTH)
            {
                float coronaT = 1.0f - (dist - ECLIPSE_UMBRA_RADIUS) / ECLIPSE_CORONA_WIDTH;
                r = lerpf(r, ECLIPSE_CORONA_COLOR.r, coronaT);
                g = lerpf(g, ECLIPSE_CORONA_COLOR.g, coronaT);
                b = lerpf(b, ECLIPSE_CORONA_COLOR.b, coronaT);
            }
        }
    }
    return { r, g, b };
}

// Pattern 5: Rings. Each beat sends a symmetric ring of light out from
// index 0, front sweeping to LED_COUNT-1 over RING_TRAVEL_MS -- fades on
// both its leading and trailing edge, unlike Rainbow Wave/Lava's one-sided tail.
void updateRings(LedState &state)
{
    if (state.ringStart == 0) return;
    unsigned long elapsed = state.lastUpdate - state.ringStart;
    if (elapsed >= RING_TRAVEL_MS)
        state.ringStart = 0;
}

RGBf ringsPixelColor(const LedState &state, uint8_t i)
{
    // No ambient floor tint -- purely reactive, per the refined "off
    // except for heart pulses" spec. Outside any active ring's band,
    // pixels stay fully black instead of resting at a dim RING_COLOR tint.
    float r = 0, g = 0, b = 0;

    if (state.ringStart != 0)
    {
        unsigned long elapsed = state.lastUpdate - state.ringStart;
        if (elapsed < RING_TRAVEL_MS)
        {
            float front = ((float)elapsed / RING_TRAVEL_MS) * (LED_COUNT - 1);
            float dist = fabsf((float)i - front);
            if (dist < RING_WIDTH)
            {
                float t = 1.0f - dist / RING_WIDTH;
                r = lerpf(r, RING_COLOR.r, t);
                g = lerpf(g, RING_COLOR.g, t);
                b = lerpf(b, RING_COLOR.b, t);
            }
        }
    }
    return { r, g, b };
}

// Pattern 6: Plant. Growth is cumulative across beats (see reactToBeat()),
// not a transient event -- each beat adds one more grown LED from index 0
// (the "roots") toward LED_COUNT-1. Brightness has two decay phases: a
// fast initial shimmer settle down to PLANT_MATURE_BRIGHTNESS, then a slow
// continuous aging dim down to PLANT_AGED_FLOOR that never stops. Violet
// flowers bloom on the grown stem whenever the two hearts are in sync
// (reactToBeat()), each with its own fade-in/hold/fade-out envelope
// recomputed inline here from flowerStart[i] + lastUpdate. Once fully
// grown, the whole strip eases down to LED_AMBIENT_FLOOR before a fresh
// plant starts.
void updatePlant(LedState &state)
{
    for (uint8_t i = 0; i < state.plantLength; i++)
    {
        if (state.plantBright[i] > PLANT_MATURE_BRIGHTNESS)
            state.plantBright[i] = max((float)PLANT_MATURE_BRIGHTNESS, state.plantBright[i] - PLANT_DECAY);
        else if (state.plantBright[i] > PLANT_AGED_FLOOR)
            state.plantBright[i] = max((float)PLANT_AGED_FLOOR, state.plantBright[i] - PLANT_AGE_DECAY);
    }

    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        if (state.flowerStart[i] == 0) continue;
        unsigned long elapsed = state.lastUpdate - state.flowerStart[i];
        if (elapsed >= (unsigned long)FLOWER_FADE_IN_MS + FLOWER_HOLD_MS + FLOWER_FADE_OUT_MS)
            state.flowerStart[i] = 0; // envelope finished
    }

    if (state.plantLength >= LED_COUNT)
    {
        if (state.plantFadeOutStart == 0)
        {
            state.plantFadeOutStart = state.lastUpdate;
            for (uint8_t i = 0; i < LED_COUNT; i++)
                state.plantFadeOutFrom[i] = state.plantBright[i]; // snapshot -- fade from here
        }
        unsigned long elapsed = state.lastUpdate - state.plantFadeOutStart;
        float t = min(1.0f, (float)elapsed / PLANT_FADEOUT_MS);
        for (uint8_t i = 0; i < LED_COUNT; i++)
            state.plantBright[i] = lerpf(state.plantFadeOutFrom[i], LED_AMBIENT_FLOOR, t);

        if (t >= 1.0f)
        {
            state.plantLength = 0;
            for (uint8_t i = 0; i < LED_COUNT; i++)
                state.plantBright[i] = LED_AMBIENT_FLOOR;
            for (uint8_t i = 0; i < LED_COUNT; i++)
                state.flowerStart[i] = 0;
            state.plantFadeOutStart = 0; // ready to grow a fresh plant on the next beat
        }
    }
}

RGBf plantPixelColor(const LedState &state, uint8_t i)
{
    float bright = (i < state.plantLength) ? state.plantBright[i] : LED_AMBIENT_FLOOR;
    RGBf c = hsvToRgb(state.plantHue[i], 1.0f, bright / 255.0f);

    float r = c.r, g = c.g, b = c.b;
    if (state.flowerStart[i] != 0)
    {
        unsigned long elapsed = state.lastUpdate - state.flowerStart[i];
        float flower = 0;
        if (elapsed < FLOWER_FADE_IN_MS)
            flower = 255.0f * ((float)elapsed / FLOWER_FADE_IN_MS);
        else if (elapsed < (unsigned long)FLOWER_FADE_IN_MS + FLOWER_HOLD_MS)
            flower = 255.0f;
        else if (elapsed < (unsigned long)FLOWER_FADE_IN_MS + FLOWER_HOLD_MS + FLOWER_FADE_OUT_MS)
        {
            unsigned long fadeElapsed = elapsed - FLOWER_FADE_IN_MS - FLOWER_HOLD_MS;
            flower = 255.0f * (1.0f - (float)fadeElapsed / FLOWER_FADE_OUT_MS);
        }
        if (flower > 0)
        {
            float t = flower / 255.0f;
            r = lerpf(r, VIOLET_COLOR.r, t);
            g = lerpf(g, VIOLET_COLOR.g, t);
            b = lerpf(b, VIOLET_COLOR.b, t);
        }
    }
    return { r, g, b };
}

// Pattern 7: Slow Rainbow. A continuously flowing ambient rainbow gradient
// (spatial hue by position) with a slow brightness "breathing" cycle,
// running regardless of beats. slowRainbowHue is a running accumulator
// (speed depends on avgBpm and gets a kick from beatPulse) rather than a
// function of absolute time, so it never jumps when either changes;
// breathe uses PHASE_WRAP_MS since its period is a fixed constant.
void updateSlowRainbow(LedState &state, uint16_t avgBpm)
{
    float baseSpeed = SLOW_RAINBOW_HUE_SPEED * ((float)avgBpm / SLOW_RAINBOW_REFERENCE_BPM);
    float kick = ((float)state.beatPulse / 255.0f) * SLOW_RAINBOW_BEAT_KICK;
    state.slowRainbowHue = fmodf(state.slowRainbowHue + (baseSpeed + kick) * LED_UPDATE_MS / 1000.0f, 360.0f);
}

RGBf slowRainbowPixelColor(const LedState &state, uint8_t i)
{
    float nowPhase = (float)(state.lastUpdate % PHASE_WRAP_MS);
    float breathe = (sinf((nowPhase / SLOW_RAINBOW_FADE_MS) * TWO_PI_F) + 1.0f) / 2.0f;
    float brightness = SLOW_RAINBOW_MIN_BRIGHTNESS + breathe * (SLOW_RAINBOW_MAX_BRIGHTNESS - SLOW_RAINBOW_MIN_BRIGHTNESS);
    float hue = fmodf(state.slowRainbowHue + ((float)i / LED_COUNT) * 360.0f, 360.0f);
    return hsvToRgb(hue, 1.0f, brightness / 255.0f);
}

// Pattern 8: Sparkle/Twinkle. Purely reactive, per the refined "off except
// for heart pulses" spec -- no ambient auto-pop timer anymore (that used
// to pop sparkles on a BPM-scaled timer independent of real detected
// beats). Every sparkle now comes from reactToBeat()'s
// TWINKLE_BEAT_COUNT burst on an actual beat; this just decays those
// pixels back to true black (not LED_AMBIENT_FLOOR) between beats.
void updateTwinkle(LedState &state)
{
    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        if (state.twinkleBright[i] > LED_PIXEL_DECAY)
            state.twinkleBright[i] -= LED_PIXEL_DECAY;
        else
            state.twinkleBright[i] = 0;
    }
}

RGBf twinklePixelColor(const LedState &state, uint8_t i)
{
    return hsvToRgb(state.twinkleHue[i], 1.0f, state.twinkleBright[i] / 255.0f);
}

// Pattern 9: Aurora. Three overlapping sine layers at different spatial
// frequencies/phase speeds combine into an organic drifting "curtain".
// This strip's own BPM scales its curtain's intensity; a shared pink
// shimmer (auroraSyncT, see updateSharedPatternState()) fades in wherever
// the curtain is already bright when the two hearts are in sync.
void updateAurora(LedState &state, uint16_t myBpm)
{
    state.auroraPhase += AURORA_WAVE_SPEED * (LED_UPDATE_MS / 1000.0f);
    float ratio = (float)myBpm / AURORA_INTENSITY_REFERENCE_BPM;
    state.auroraIntensity = max(0.3f, min(2.0f, ratio));
}

RGBf auroraPixelColor(const LedState &state, uint8_t i)
{
    extern float auroraSyncT;
    float t = (float)i / (LED_COUNT - 1);
    float w1 = sinf(t * TWO_PI_F * 1.3f + state.auroraPhase);
    float w2 = sinf(t * TWO_PI_F * 2.7f + state.auroraPhase * 0.6f + 2.1f);
    float w3 = sinf(t * TWO_PI_F * 4.1f + state.auroraPhase * 1.4f + 4.7f);
    float wave = (w1 + w2 * 0.6f + w3 * 0.4f) / 2.0f;
    float curtain = max(0.0f, wave);

    float brightness = min(255.0f, (AURORA_BASE_BRIGHTNESS + curtain * 180.0f) * state.auroraIntensity);
    float hue = 100.0f + (1.0f - curtain) * 70.0f;

    float lowBias = max(0.0f, 1.0f - t / 0.35f);
    float lowHue = 210.0f + lowBias * 65.0f;
    hue = lerpf(hue, lowHue, lowBias);

    float highBias = max(0.0f, (t - 0.85f) / 0.15f);
    hue = lerpf(hue, 355.0f, highBias);

    RGBf c = hsvToRgb(hue, 0.85f, brightness / 255.0f);
    float r = c.r, g = c.g, b = c.b;

    if (auroraSyncT > 0)
    {
        float pinkT = auroraSyncT * curtain;
        r = lerpf(r, AURORA_SYNC_COLOR.r, pinkT);
        g = lerpf(g, AURORA_SYNC_COLOR.g, pinkT);
        b = lerpf(b, AURORA_SYNC_COLOR.b, pinkT);
    }
    return { r, g, b };
}

// ── Shared (not per-strip) pattern state ────────────────────────────────────
// Eclipse's sun color and Aurora's pink shimmer are properties of *both*
// hearts together, not either strip alone -- advanced once per loop() at
// LED_UPDATE_MS cadence (their increment math assumes exactly that), not
// once per raw loop() iteration.
float eclipseSyncT = 0;             // 0 = fully blue, 1 = fully orange-yellow
RGBf eclipseSunColorNow = ECLIPSE_SUN_COLOR;
float auroraSyncT = 0;              // 0 = no shimmer, 1 = full shimmer
unsigned long lastSharedLedUpdate = 0;

// Direct port of updateEclipseSyncColor()/updateAuroraSyncColor() from
// LEDSimulator3D.html.
void updateSharedPatternState(uint16_t bpm1, uint16_t bpm2)
{
    float step;

    bool eclipseSynced = bpmDelta(bpm1, bpm2) <= ECLIPSE_SYNC_THRESHOLD;
    step = (float)LED_UPDATE_MS / ECLIPSE_SYNC_COLOR_MS;
    eclipseSyncT = eclipseSynced ? min(1.0f, eclipseSyncT + step) : max(0.0f, eclipseSyncT - step);
    eclipseSunColorNow.r = lerpf(ECLIPSE_SUN_COLOR.r, ECLIPSE_SYNC_SUN_COLOR.r, eclipseSyncT);
    eclipseSunColorNow.g = lerpf(ECLIPSE_SUN_COLOR.g, ECLIPSE_SYNC_SUN_COLOR.g, eclipseSyncT);
    eclipseSunColorNow.b = lerpf(ECLIPSE_SUN_COLOR.b, ECLIPSE_SYNC_SUN_COLOR.b, eclipseSyncT);

    bool auroraSynced = bpmDelta(bpm1, bpm2) <= AURORA_SYNC_THRESHOLD;
    step = (float)LED_UPDATE_MS / AURORA_SYNC_COLOR_MS;
    auroraSyncT = auroraSynced ? min(1.0f, auroraSyncT + step) : max(0.0f, auroraSyncT - step);
}

// ── LedState dispatch methods ────────────────────────────────────────────
// Out-of-line now that every per-pattern update/color function above is in scope.
void LedState::updatePattern(uint16_t myBpm, uint16_t otherBpm)
{
    switch (currentPattern)
    {
    case PATTERN_RED_BLUE:     updateSparkle(*this); break;
    case PATTERN_CHASE:        updateChase(*this); break;
    case PATTERN_RAINBOW_WAVE: updateRainbowWave(*this); break;
    case PATTERN_LAVA:         updateLava(*this); break;
    case PATTERN_ECLIPSE:      updateEclipse(*this); break;
    case PATTERN_RINGS:        updateRings(*this); break;
    case PATTERN_PLANT:        updatePlant(*this); break;
    case PATTERN_SLOW_RAINBOW: updateSlowRainbow(*this, (uint16_t)(((uint32_t)myBpm + otherBpm) / 2)); break;
    case PATTERN_SPARKLE:      updateTwinkle(*this); break;
    case PATTERN_AURORA:       updateAurora(*this, myBpm); break;
    default: break;
    }
}

RGBf LedState::getPixelColor(uint8_t i) const
{
    // Slow Rainbow/Sparkle/Aurora are ambient/idle by design (DESIGN.md's
    // "Idle / no reading -> slow twinkling ambient light") and always
    // render, signal or not. Every other pattern is beat-reactive and
    // stays fully dark without a live heartbeat, per the refined spec --
    // rather than showing their own idle drift/decay.
    switch (currentPattern)
    {
    case PATTERN_SLOW_RAINBOW: return slowRainbowPixelColor(*this, i);
    case PATTERN_SPARKLE:      return twinklePixelColor(*this, i);
    case PATTERN_AURORA:       return auroraPixelColor(*this, i);
    default: break;
    }
    if (!hasSignal)
        return {0, 0, 0};

    switch (currentPattern)
    {
    case PATTERN_CHASE:        return chasePixelColor(*this, i);
    case PATTERN_RAINBOW_WAVE: return rainbowPixelColor(*this, i);
    case PATTERN_LAVA:         return lavaPixelColor(*this, i);
    case PATTERN_ECLIPSE:      return eclipsePixelColor(*this, i);
    case PATTERN_RINGS:        return ringsPixelColor(*this, i);
    case PATTERN_PLANT:        return plantPixelColor(*this, i);
    default:                   return sparklePixelColor(*this, i); // PATTERN_RED_BLUE
    }
}

// Direct port of triggerBeat() from LEDSimulator3D.html -- each pattern's
// own reaction to a freshly detected beat. Patterns 7 (Slow Rainbow) and 9
// (Aurora) do nothing beyond the beatPulse=255 checkBeat() already set --
// they're ambient-only, driven by their own continuous update function.
void LedState::reactToBeat(uint16_t myBpm, uint16_t otherBpm)
{
    unsigned long now = millis();
    switch (currentPattern)
    {
    case PATTERN_RED_BLUE:
        for (uint8_t n = 0; n < SPARKLE_BEAT_COUNT; n++)
        {
            uint8_t idx = random(LED_COUNT);
            pxBright[idx] = 255;
            hue[idx] = randomHueInRange(hueLo, hueHi);
        }
        break;

    case PATTERN_CHASE:
        chaseHead = 0;
        break;

    case PATTERN_RAINBOW_WAVE:
        for (uint8_t k = 0; k < WAVE_MAX_ACTIVE; k++)
        {
            if (waveStart[k] == 0)
            {
                waveStart[k] = now;
                break; // one free slot claimed; if none free, this beat's wave is dropped
            }
        }
        break;

    case PATTERN_LAVA:
        for (uint8_t k = 0; k < LAVA_BEAT_BURST && k < LED_COUNT; k++)
            heat[k] = 255;
        break;

    case PATTERN_ECLIPSE:
        if (eclipseStart == 0)
            eclipseStart = now; // only launch a fresh shadow if the last one already finished
        break;

    case PATTERN_RINGS:
        ringStart = now;
        break;

    case PATTERN_PLANT:
        if (plantLength < LED_COUNT)
        {
            uint8_t idx = plantLength;
            plantHue[idx] = randomHueInRange(PLANT_HUE_MIN, PLANT_HUE_MAX);
            plantBright[idx] = 255;
            plantLength++;
        }
        if (plantLength > 1 && bpmDelta(myBpm, otherBpm) <= PLANT_SYNC_THRESHOLD)
        {
            for (uint8_t f = 0; f < PLANT_FLOWER_COUNT; f++)
            {
                uint8_t idx = random(plantLength - 1); // never the growth tip
                flowerStart[idx] = now;
            }
        }
        break;

    case PATTERN_SPARKLE:
        for (uint8_t n = 0; n < TWINKLE_BEAT_COUNT; n++)
        {
            uint8_t idx = random(LED_COUNT);
            twinkleBright[idx] = TWINKLE_BEAT_BRIGHTNESS;
            twinkleHue[idx] = randomHueInRange(TWINKLE_HUE_MIN, TWINKLE_HUE_MAX);
        }
        break;

    default:
        break; // PATTERN_SLOW_RAINBOW / PATTERN_AURORA: no beat reaction beyond beatPulse=255
    }
}

// ── A0 button: single press cycles the LED pattern, triple-click (within
// MODE_TRIPLE_CLICK_WINDOW_MS) cycles the mode instead ──────────────────────
const uint16_t BUTTON_DEBOUNCE_MS = 30;
bool buttonLastRaw = HIGH;       // HIGH = released (INPUT_PULLUP, active-LOW)
bool buttonStableState = HIGH;
unsigned long buttonLastChangeMs = 0;

const uint16_t MODE_TRIPLE_CLICK_WINDOW_MS = 2000;
uint8_t clickCount = 0;
unsigned long clickWindowStart = 0;         // 0 = no click window currently open
LedPattern patternBeforeClickWindow;        // snapshot to restore from if a triple-click is confirmed

void cyclePattern();
void cycleMode();

// Non-blocking debounce: track the last raw level change, commit a new
// stable state only after it's held for BUTTON_DEBOUNCE_MS. Unlike
// HardwareTest.ino's testButton() (blocking, while(digitalRead())), loop()
// here must keep servicing sensors/motors every iteration -- same
// millis()-driven idiom as HeartChannel::updatePulseSchedule().
//
// Every press cycles the pattern immediately (cyclePattern() below), same
// as always -- deliberately NOT deferred/held to see whether a triple-click
// is coming, since that would add up to the full 2s window's worth of lag
// to every single press, which is by far the common case. Instead, a
// confirmed triple-click (3rd press within the window) restores the
// pattern to whatever it was before the 1st press of that window -- so a
// triple ends net-unchanged on the pattern and switches mode instead --
// at the cost of a brief flicker through 2 patterns during the gesture
// itself, which only happens on a deliberate triple-click.
void pollButton()
{
    bool raw = digitalRead(PIN_BUTTON);
    if (raw != buttonLastRaw)
    {
        buttonLastChangeMs = millis();
        buttonLastRaw = raw;
    }
    if (millis() - buttonLastChangeMs >= BUTTON_DEBOUNCE_MS && raw != buttonStableState)
    {
        buttonStableState = raw;
        if (buttonStableState == LOW) // falling edge = press
        {
            unsigned long now = millis();
            bool newWindow = (clickWindowStart == 0 || now - clickWindowStart > MODE_TRIPLE_CLICK_WINDOW_MS);
            if (newWindow)
            {
                clickWindowStart = now;
                clickCount = 0;
                patternBeforeClickWindow = currentPattern; // before this window's 1st cyclePattern() below
            }
            clickCount++;

            cyclePattern();

            if (clickCount >= 3)
            {
                // Confirmed triple-click: undo the 3 pattern advances
                // above (net zero change) and switch mode instead.
                currentPattern = patternBeforeClickWindow;
                leds1.resetPattern();
                leds2.resetPattern();
                Serial.print(F("LED pattern: "));
                Serial.println(patternNames[currentPattern]);

                cycleMode();
                clickCount = 0;
                clickWindowStart = 0; // re-arm: next press opens a fresh window
            }
        }
    }
}

// Advances to the next LED pattern, wrapping after the last one, and
// resets both strips' pattern-specific state -- mirrors
// LEDSimulator3D.html's cyclePattern()/setPattern().
void cyclePattern()
{
    currentPattern = (LedPattern)((currentPattern + 1) % NUM_PATTERNS);
    leds1.resetPattern();
    leds2.resetPattern();
    Serial.print(F("LED pattern: "));
    Serial.println(patternNames[currentPattern]);
}

HeartChannel participant1(SENSOR1_SDA, SENSOR1_SCL);
HeartChannel participant2(SENSOR2_SDA, SENSOR2_SCL);

unsigned long lastPlotMs = 0;

unsigned long syntheticNextBeat = 0; // 0 = not yet armed; see updateSyntheticClock()

// Advances to the next mode, wrapping after the last one. Resets the
// synthetic clock and clears any pulseTriggerTime left over from it, so
// leaving/entering a synthetic-clock mode never leaves a stale pulse
// behind for the motor/LED code to react to on the next iteration. Lives
// here (not next to pollButton()/cyclePattern()) since it needs
// participant1/participant2, declared just above.
void cycleMode()
{
    currentMode = (Mode)((currentMode + 1) % NUM_MODES);
    syntheticNextBeat = 0;
    participant1.pulseTriggerTime = 0;
    participant2.pulseTriggerTime = 0;
    Serial.print(F("Mode: "));
    Serial.println(modeNames[currentMode]);
}

// Synthetic-clock modes (Breath Entrainment, Yellow): a fixed-rate,
// sensor-independent heartbeat shared by both participants, driving the
// exact same pulseTriggerTime-based motor envelope and LED beat reaction
// real beats use (HeartChannel::motorIntensity()/LedState::checkBeat())
// -- so the felt/visual pulse looks and feels identical either way, just
// decoupled from any real physiological signal. Both channels fire in
// perfect unison (same `now` written to both). Call once per loop()
// iteration whenever currentMode != MODE_NORMAL, with that mode's own
// rate (see SYNTHETIC_BPM); loop() skips real sensor reads entirely while
// any synthetic-clock mode is active, so this is the only thing setting
// pulseTriggerTime during that time.
void updateSyntheticClock(uint16_t bpm)
{
    unsigned long now = millis();
    if (syntheticNextBeat == 0 || now >= syntheticNextBeat)
    {
        participant1.pulseTriggerTime = now;
        participant2.pulseTriggerTime = now;
        syntheticNextBeat = now + 60000UL / bpm;
    }
}

// Smooths a fresh A2 reading into the shared `intensity` global -- see its
// declaration near PIN_POT above for why this function lives down here
// instead of next to that declaration. Inverted (1023 - raw) so turning
// the knob toward its wired-high end is the *low*-intensity direction --
// physical "intense" end is the other one.
void updateIntensity()
{
    float raw = (1023 - analogRead(PIN_POT)) / 1023.0f;
    intensity += (raw - intensity) * POT_SMOOTHING;
}

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
    Serial.print(F(" Mode")); Serial.print(suffix); Serial.print(':'); Serial.print(channel.inSteadyPulseMode() ? 1 : 0);
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    pinMode(MOTOR_PIN_D6, OUTPUT);
    pinMode(MOTOR_PIN_D13, OUTPUT);
    analogWrite(MOTOR_PIN_D6, 0);
    analogWrite(MOTOR_PIN_D13, 0);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    // Seed with a real (unsmoothed) reading so playback starts at the
    // knob's actual position instead of easing up/down from the 1.0
    // default over the first several updateIntensity() calls. Inverted --
    // see updateIntensity()'s comment.
    intensity = (1023 - analogRead(PIN_POT)) / 1023.0f;

    strip1.begin(); strip1.clear(); strip1.show();
    strip2.begin(); strip2.clear(); strip2.show();

    // Per-strip identity, set before begin() so its one-time random seeding
    // (pattern 0's hue[]) uses the right range -- see LedState::begin().
    leds1.hueLo = SPARKLE_HUE_RANGE_1[0]; leds1.hueHi = SPARKLE_HUE_RANGE_1[1];
    leds1.themeColor = {220, 60, 0};   // orange
    leds1.begin();
    leds2.hueLo = SPARKLE_HUE_RANGE_2[0]; leds2.hueHi = SPARKLE_HUE_RANGE_2[1];
    leds2.themeColor = {0, 144, 192};  // blue
    leds2.begin();

    Serial.println(F("The Interface - HR Visualizer"));
    bool sensor1Ready = startSensor(participant1, F("Sensor 1"));
    bool sensor2Ready = startSensor(participant2, F("Sensor 2"));

    if (!sensor1Ready || !sensor2Ready)
        Serial.println(F("Fix sensor wiring before using the visualizer."));
    else
        Serial.println(F("Open Serial Plotter at 115200 baud."));

    Serial.print(F("LED pattern: "));
    Serial.println(patternNames[currentPattern]);
    Serial.print(F("Mode: "));
    Serial.println(modeNames[currentMode]);
}

void loop()
{
    bool syntheticMode = (currentMode != MODE_NORMAL);

    if (syntheticMode)
    {
        // Breath Entrainment / Yellow: no real sensor I/O at all, per spec
        // -- a synthetic clock at this mode's own rate drives both
        // channels' shared pulseTriggerTime instead.
        updateSyntheticClock(SYNTHETIC_BPM[currentMode]);
    }
    else
    {
        readChannel(participant1);
        readChannel(participant2);

        // EXPERIMENTAL: advances the steady-BPM pulse schedule once in steady
        // mode. No-op (and harmless) outside TRACKING or during the initial
        // raw-feedback phase -- see PULSE_STEADY_TRANSITION_MS.
        participant1.updatePulseSchedule();
        participant2.updatePulseSchedule();
    }

    // Overall strength/brightness knob (A2) -- smoothed against ADC
    // jitter, shared by the motor writes and LED render below. Still
    // live in every mode.
    updateIntensity();

    // Sampled once per loop and reused for both the motor write and the
    // (throttled) print below, so the printed Motor value always matches
    // exactly what was written to the pin this iteration.
    byte motor1 = participant1.motorIntensity(intensity);
    byte motor2 = participant2.motorIntensity(intensity);

    // Updated every loop() iteration (not gated by the plot throttle below)
    // so the ramp envelope is smooth rather than stepping in 40 ms chunks.
    // Cross-paired per the wiring: participant 1 (D8/D9 sensor) -> D13
    // motor, participant 2 (SDA/SCL sensor) -> D6 motor.
    analogWrite(MOTOR_PIN_D13, motor1);
    analogWrite(MOTOR_PIN_D6, motor2);

    pollButton();

    // In a synthetic-clock mode, participant.bpm/.state are frozen
    // (process() isn't running) -- feed the LED/pattern code that mode's
    // fixed rate and force "has signal" on instead, so beat-reactive
    // patterns render normally off the artificial pulse rather than
    // reading a stale real value.
    uint16_t bpm1 = syntheticMode ? SYNTHETIC_BPM[currentMode] : participant1.bpm;
    uint16_t bpm2 = syntheticMode ? SYNTHETIC_BPM[currentMode] : participant2.bpm;
    bool active1 = syntheticMode ? true : (participant1.state == TRACKING);
    bool active2 = syntheticMode ? true : (participant2.state == TRACKING);

    // Shared pattern state (Eclipse/Aurora sync shimmer) advances at most
    // once per LED_UPDATE_MS tick -- its increment math assumes exactly
    // that cadence, see updateSharedPatternState(). In a synthetic-clock
    // mode, bpm1==bpm2 always, so both strips' sync shimmer stays fully engaged.
    if (millis() - lastSharedLedUpdate >= LED_UPDATE_MS)
    {
        lastSharedLedUpdate = millis();
        updateSharedPatternState(bpm1, bpm2);
    }

    // Check for new pulses (the same shared trigger the motor envelope
    // above just used) and update LED strips at 50 fps. Called after
    // sensor reads so show() never interrupts a SoftWire transaction.
    leds1.checkBeat(participant1.pulseTriggerTime, bpm1, bpm2);
    leds2.checkBeat(participant2.pulseTriggerTime, bpm2, bpm1);
    if (leds1.needsUpdate()) leds1.update(strip1, bpm1, bpm2, active1, intensity);
    if (leds2.needsUpdate()) leds2.update(strip2, bpm2, bpm1, active2, intensity);

    if (millis() - lastPlotMs < 40)
        return;
    lastPlotMs = millis();

    // Clean plot output. State: 0 = no contact, 1 = calibrating, 2 = tracking.
    printChannel(participant1, '1', motor1);
    Serial.print(' ');
    printChannel(participant2, '2', motor2);
    Serial.println();
}
