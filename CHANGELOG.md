# Changelog

All notable changes to The Interface are documented here.

## Unreleased

### Added

- Two-participant serial heart-rate visualizer in `theinterface.ino`.
- Per-participant runtime states: no contact, calibration, and tracking.
- Per-participant automatic IR LED-current calibration with bounded gain and settling time.
- DC-removed waveforms, adaptive beat thresholds, beat markers, and rolling eight-interval BPM estimates.
- HRV (RMSSD) computed over the same rolling beat-interval buffer.
- SpO2 estimate via ratio-of-ratios of Red/IR AC-DC levels, with outlier rejection and low-pass smoothing (uncalibrated, non-medical approximation).
- Per-participant automatic Red LED-current calibration (mirrors the existing IR calibration) so the SpO2 ratio isn't corrupted by a clipped or noise-floor Red channel.
- FIFO overflow detection (`OVF_COUNTER`) that forces a clean recalibration instead of computing BPM/HRV/SpO2 against desynced timing after dropped samples.
- Clean Arduino Serial Plotter fields: `Wave1`, `Wave2`, `Beat1`, `Beat2`, `BPM1`, `BPM2`, `HRV1`, `HRV2`, `SpO2_1`, `SpO2_2`, `State1`, and `State2`.
- `InterfaceSensor::setIRLedAmplitude()` / `setRedLedAmplitude()` for runtime LED-current control, and `getOverflowCount()` for FIFO overflow detection.
- Non-blocking motor pulse envelope (`HeartChannel::motorIntensity()`): linear ramp up (40 ms) / ramp down (200 ms) on each detected beat, driven via `analogWrite` on D6/D13. Peak intensity is a fixed constant for now; potentiometer control is a follow-up. `Motor1`/`Motor2` added to the Serial Plotter output for tuning.

### Changed

- Replaced the original single-sensor IR print loop with a two-person visualizer.
- Replaced continuous gain changes during BPM detection with a calibration phase that locks gain before tracking.
- `readChannel()` now reads both Red and IR from `readFIFO()` instead of discarding Red via `getIR()`.
- Updated the design and development documentation to describe the current sensor workflow.

### Fixed

- Switched MAX30102 reads to SoftWire's buffered transaction API, fixing register and FIFO reads on the Arduino UNO R4 Minima.
- Corrected `SAMPLE_PERIOD_MS` from 10 ms to 40 ms to match the actual FIFO push rate (100 Hz sampling with 4x FIFO averaging = 25 Hz FIFO entries). The old value understated real beat intervals ~4x, causing valid beats to fail the plausibility check and BPM to read wrong or not at all.
- `InterfaceSensor::getOverflowCount()` now explicitly clears `OVF_COUNTER` after reading a nonzero value instead of assuming the chip auto-resets it. Without this, a single overflow event latched the register nonzero permanently, which forced every channel back into calibration on every loop iteration and made it impossible to ever reach `TRACKING` — BPM/HRV/SpO2 stayed at 0 and `State` never advanced past 1.
