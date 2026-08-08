# Changelog

All notable changes to The Interface are documented here.

## Unreleased

### Added

- LED beat reaction now keys off the same shared `pulseTriggerTime` the motor envelope already used (renamed from `motorTriggerTime`, along with `inSteadyMotorMode()`/`updateMotorSchedule()`/`MOTOR_STEADY_TRANSITION_MS` -> `inSteadyPulseMode()`/`updatePulseSchedule()`/`PULSE_STEADY_TRANSITION_MS`), instead of the raw-only `beatFlashUntil`. This puts the motor and LEDs in lockstep, and once past the steady-mode transition, both keep pulsing on the rolling BPM average even if an individual raw beat is misread or missed -- and both fall silent together the moment a channel loses contact (`pulseTriggerTime` resets to 0 in `resetBeatState()`). `beatFlashUntil` still exists but is now diagnostic-only (drives the Serial `Beat1`/`Beat2` field).
- 10 selectable LED patterns (Red-Blue, Chase, Rainbow Wave, Lava, Eclipse, Rings, Plant, Slow Rainbow, Sparkle, Aurora), ported from the `LEDSimulator3D.html` design tool into `theinterface.ino`, cycled live on both strips by the A0 pushbutton (non-blocking debounce) with the active pattern name printed to Serial on each press. Eclipse, Plant, and Aurora each add a cross-strip "hearts in sync" reaction (BPM delta at/under a per-pattern threshold) on top of their own per-strip animation.
- Two-participant serial heart-rate visualizer in `theinterface.ino`.
- Per-participant runtime states: no contact, calibration, and tracking.
- Per-participant automatic IR LED-current calibration with bounded gain and settling time.
- DC-removed waveforms, adaptive beat thresholds, beat markers, and rolling eight-interval BPM estimates.
- HRV (RMSSD) computed over the same rolling beat-interval buffer.
- SpO2 estimate via ratio-of-ratios of Red/IR AC-DC levels, with outlier rejection and low-pass smoothing (uncalibrated, non-medical approximation).
- Per-participant automatic Red LED-current calibration (mirrors the existing IR calibration) so the SpO2 ratio isn't corrupted by a clipped or noise-floor Red channel.
- Clean Arduino Serial Plotter fields: `Wave1`, `Wave2`, `Beat1`, `Beat2`, `BPM1`, `BPM2`, `HRV1`, `HRV2`, `SpO2_1`, `SpO2_2`, `State1`, and `State2`.
- `InterfaceSensor::setIRLedAmplitude()` / `setRedLedAmplitude()` for runtime LED-current control.
- Non-blocking motor pulse envelope (`HeartChannel::motorIntensity()`): linear ramp up (40 ms) / ramp down (200 ms) on each detected beat, driven via `analogWrite` on D6/D13. Peak intensity is a fixed constant for now; potentiometer control is a follow-up. `Motor1`/`Motor2` added to the Serial Plotter output for tuning.

### Changed

- Replaced the original single-sensor IR print loop with a two-person visualizer.
- Replaced continuous gain changes during BPM detection with a calibration phase that locks gain before tracking.
- `readChannel()` now reads both Red and IR from `readFIFO()` instead of discarding Red via `getIR()`.
- Updated the design and development documentation to describe the current sensor workflow.

### Fixed

- Switched MAX30102 reads to SoftWire's buffered transaction API, fixing register and FIFO reads on the Arduino UNO R4 Minima.
- Corrected `SAMPLE_PERIOD_MS` from 10 ms to 40 ms to match the actual FIFO push rate (100 Hz sampling with 4x FIFO averaging = 25 Hz FIFO entries). The old value understated real beat intervals ~4x, causing valid beats to fail the plausibility check and BPM to read wrong or not at all.
- Removed automatic FIFO-overflow-triggered recalibration. It relied on `OVF_COUNTER` (register 0x05) clearing after being read or rewritten, which testing showed does not happen on this hardware -- the register appears read-only, so a single overflow event latched it nonzero permanently and forced every channel back into `NO_CONTACT`/`CALIBRATING` on every loop iteration, making it impossible to ever reach `TRACKING`. `getOverflowCount()` is kept for manual debug inspection only.
- Corrected motor/sensor pairing to match the physical wiring: the D8/D9 (participant 1) sensor now drives the D13 motor, and the SDA/SCL (participant 2) sensor now drives the D6 motor. The original motor modulation implementation paired them by matching number (participant 1 -> D6, participant 2 -> D13), which didn't match how the boards are actually wired.
- Decoupled motor/Beat-marker feedback from the BPM interval-plausibility check. Previously a real detected peak whose timing was rejected for BPM purposes (e.g. right after a stronger beat, or the first beat after tracking starts) also silently suppressed the motor pulse, which read as inconsistent vibration even though the sensor did see the beat.
- Capped how far a single cycle can push the adaptive beat threshold upward (`THRESHOLD_MAX_STEP_RATIO`). Confirmed via live serial capture that an occasional large-amplitude outlier sample (motion artifact or bad I2C read) was feeding straight into the threshold via `nextThreshold = positivePeak / 2`, inflating it several times over and then suppressing real, normal-amplitude beats for several seconds while it slowly decayed back down (75/25 blend). This was the main cause of "sometimes vibrating, sometimes not" -- live counters showed only ~55-60% of zero-crossing cycles passing the threshold before the fix, ~87% after.
