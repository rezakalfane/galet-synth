# Changelog

All notable changes to GaletSynth are documented here.

## [Unreleased]

---

## 2026-04-07

### Added — `feat: add QUANTIZE_ENABLED flag to disable pitch quantization` (`21ae1df`)
- New `QUANTIZE_ENABLED` constant (`true` by default) at the top of `src/main.cpp`
- Set to `false` for fully continuous pitch from first touch — no snapping to scale
- When `true`, touch-down still snaps to the nearest chromatic note as before

### Added — `feat: per-electrode pressure calibration and smooth pitch sliding` (`5b72465`)
- **Per-electrode pressure arrays**: `PRESSURE_MAX_REF[12]` and `PRESSURE_DEAD_ZONE[12]`
  — edge channels 0 and 11 have higher dead zones to compensate for their naturally
  greater sensitivity through glass
- `pressure_pct()` now takes a channel index for per-electrode mapping
- `MIN_FINGER_SEP` raised from 3 to 4 to suppress false second-finger detection
- **Re-quantize guard**: pitch only snaps to scale on a genuinely fresh touch
  (finger off for >200 ms), preventing unintended re-quantization during brief
  threshold dropouts while sliding
- `SLEW_FREQ` increased from 0.9970 to 0.9985 for smoother pitch glide (~14 ms)
- Removed duplicate `LDFLAGS` from `Makefile` that caused `--specs` linker error

### Added — `added: calibration tool` (`f88324d`)
- `tools/mpr121_calibrate.cpp`: full AFE + pressure calibration tool
  - Phase 1a: CDC × CDT sweep (charge current and charge time)
  - Phase 1b: FFI × ESI sweep (first filter iterations and sample interval)
  - Phase 2: per-electrode light/hard press capture to derive `PRESSURE_MIN_REF`
    and `PRESSURE_MAX_REF` arrays
  - Waits for actual touch before recording each measurement window
  - Outputs register values and array constants ready to paste into `src/main.cpp`

---

## 2026-04-04

### Changed — `chore: update` (`7e80166`)
- Incremental tuning and fixes during early hardware bring-up

### Added — `Initial commit` (`06d2b81`)
- Glass touch bass synth for Daisy Seed + MPR121 capacitive sensor
- Bit-bang I2C driver (SDA=D12, SCL=D11)
- Two-finger tracking with centroid position and pressure
- Finger 1: position → exponential pitch (50–300 Hz), pressure → filter cutoff + vibrato
- Finger 2: position → detune, pressure → bitcrush + ring mod
- Touch-down quantization to chromatic scale, free continuous pitch on slide
- Auto-rebaseline after 2 s of idle
- Calibration tools: `mpr121_glass_tuner`, `mpr121_glass_tuner2`, `mpr121_reader`,
  `i2c_scanner`, `osc_test`
