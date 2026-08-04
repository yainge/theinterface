# The Interface — Design Document

A two-participant biofeedback installation. Each person wears a heart rate sensor.
The system reads both heartrates and uses light and vibration to create a shared sensory
experience. A later phase nudges both participants toward a shared "flow state" rhythm —
that part is deferred and not in scope for the current build.

---

## Hardware overview

One Arduino UNO R4 Minima runs everything. Each participant has:

- **Heart sensor** (MAX30102) — reads pulse via infrared light through skin
- **Vibration motor** (DRV8833 driver + rumble pack) — fires on each heartbeat
- **LED strip** — 44 WS2812 lights that respond to that participant's heartrate

Shared controls:
- **Button** (A0) — cycles through LED patterns
- **Potentiometer** (A2) — tunes motor intensity

---

## What the system does now (in-scope)

Each LED strip pulses in brightness on every detected heartbeat.
The motor fires a short vibration on each beat — smooth onset, natural decay.
Both strips and motors run independently, one per participant.

---

## What is deferred

- Sync logic: detecting when two heartrates come within 5–10 BPM and nudging them toward a shared rhythm.
- Multi-participant LED coherence (strips echoing each other).

These are intentionally out of scope until the single-participant heartbeat → LED + motor pulse is solid.

---

## LED behaviour

| State | What the LEDs do |
|---|---|
| Idle / no sensor reading | Slow twinkling ambient light |
| Heartbeat detected | Strip brightness pulses up on each beat |
| Near-sync (5–10 BPM delta) | Deferred |
| Fully synced | Deferred |

Each strip is independent. Strip 1 reflects participant 1's heartrate; strip 2 reflects participant 2's.

---

## Motor behaviour

The vibration motor fires on each detected heartbeat. The feel is tunable:
- Onset: PWM ramps up quickly (sharp but not harsh)
- Decay: coasts to zero naturally
- Intensity: controlled by the A2 potentiometer

---

## Build order

1. **Run HardwareTest** — flash the test harness, verify every component responds correctly before writing production code.

2. **Sensor soak test** — run the raw IR readout for ~5 minutes with a finger on the sensor. Confirm the IR values are stable (not frozen). This resolves a known ambiguity in the sensor FIFO configuration.

3. **IR → BPM** — the raw sensor outputs 18-bit infrared samples at 100 Hz. A peak detector finds the pulse peaks, measures the time between them, and produces a smoothed BPM number. Everything downstream depends on this being reliable. Validate against a phone heart rate app.

4. **Motor pulse** — fire the motor on each detected beat. Tune onset and intensity.

5. **LED pulse** — pulse each strip's brightness on each detected beat.

6. **Dual sensor** — add the second sensor and run both channels simultaneously.

7. **Sync logic** — deferred.

---

## Technical notes (brief)

- Arduino UNO R4 Minima: one program, one loop. All sensor, LED, and motor logic shares the same `loop()`. Keep it non-blocking.
- Both heart sensors have the same I2C address. They run on separate bit-bang (SoftWire) buses to avoid conflicts. Hardware I2C (Wire library) was tried and abandoned — it causes intermittent lockups.
- NeoPixel `show()` and SoftWire both need careful timing. Do not call them at the same instant.

---

## Files

| File | What it is |
|---|---|
| `InterfaceSensor.h/.cpp` | Heart sensor library — done |
| `theinterface.ino` | Main sketch — currently a bare IR readout loop; needs full implementation |
| `HardwareTest/HardwareTest.ino` | Test harness — flash this first to verify hardware before writing production code |
| `CLAUDE.md` | Full technical reference (board, pins, locked decisions, task order) |
