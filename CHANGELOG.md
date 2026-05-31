# Changelog

All notable changes to GaletSynth are documented here.

## [Unreleased]

### Added — `feat: per-voice envelope/glide/keytracking and new effect colors`
- Amp **attack/release** and pitch **glide** are now per-voice (`attack_ms`,
  `release_ms`, `glide_ms`), specified in milliseconds and converted to slew
  coefficients once at startup (`ms_to_coeff`). Previously global constants
  (`SLEW_FREQ`/`SLEW_AMP_A`/`SLEW_AMP_R`) shared by every voice — so plucks,
  pads and stabs now feel distinct
- **Filter keytracking** per voice (`keytrack`, 0..1) — how much the cutoff base
  follows pitch; <1 keeps the tone consistent across the range
- New per-voice effect parameters:
  - `noise_level` — white noise (xorshift32) mixed into the stack for breath /
    chiff / hiss
  - `ringmod_ratio` — ring-mod carrier pitch ratio; non-integer values give
    inharmonic bell/metallic tones (was hardcoded to unison)
  - `ringmod_max` / `fold_max` — per-voice ceilings for the finger-2 effects
- Preset voicing applied: e.g. `ORGAN` gains breath noise + fast on/off,
  `SCREAM` gains an inharmonic 2.5× metallic ring-mod + snappy attack,
  `BASS_CLOSED` gains a soft attack, long tail and reduced keytracking
- The `noise` and `keytrack == 1` paths fold away at compile time when unused

### Added — `feat: Voice abstraction with switchable presets and per-oscillator waveforms`
- New `Voice` struct bundles a sound's full definition: oscillator stack (per-osc
  pitch ratio, mix level, and waveform), filter character (base cutoff, sweep
  depth, resonance), drive amount, and pitch range
- New `Waveform` enum (`WAVE_TRI`, `WAVE_SINE`, `WAVE_SQUARE`, `WAVE_SAW`) set
  per oscillator, so one voice can mix shapes (e.g. saw root + square fifth +
  sine sub). Sine uses a parabolic approximation; square/saw are naive
- Seven presets:
  - `VOICE_LEAD` — original glass-Moog lead (triangle, finger-2 detune)
  - `VOICE_BASS` — round & dark power chord (osc2 = fixed perfect fifth, so every
    note sounds as root + 5th + octave + sub)
  - `VOICE_BASS_OPEN` — same chord stack, brighter base + wider pressure sweep
  - `VOICE_BASS_RICH` — mixed waveforms + dark base + huge 13-octave pressure
    sweep with high resonance for an acid-style filter "wow"
  - `VOICE_ORGAN` — clean all-sine organ/flute in a melodic register, with a
    1.005× detuned unison for slow chorus shimmer
  - `VOICE_SCREAM` — aggressive detuned-saw lead, high register, heavy drive and
    near-self-oscillation resonance
  - `VOICE_BASS_CLOSED` — deep, muffled sub bass with the filter near the
    fundamental and almost no pressure sweep
- Switch sounds by editing a single line: `static constexpr Voice VOICE = ...`
  — compile-time selection, zero runtime cost (unused branches fold away)
- Pitch range, cutoff scaling, drive, oscillator tuning, and waveform all now
  read from the active voice in both the audio callback and the touch loop
- Boot banner prints the active voice name and range
- Replaces the previously-dead `FREQ_LOW`/`FREQ_HIGH` constants

### Fixed — `fix: smooth cutoff response under pressure`
- Cutoff opening slew slowed from ~1 ms to ~35 ms (`SLEW_CUT` 0.9800 → 0.9994).
  The cutoff target is only recomputed at the ~12-16 Hz touch-loop rate and
  pressure is quantized to integer %, so the previous fast slew let each coarse
  step through as audible zipper/stair-stepping. The slower slew interpolates
  across the steps for a smooth filter sweep while still feeling responsive.

---

## 2026-04-07

### Added — `feat: add test-slider diagnostic tool` (`3eabfd1`)
- New `tools/test-slider.cpp`: sensor display tool with all audio stripped out
- Shows live delta bars for all 12 electrodes, two-finger position bar, and per-finger pressure
- Refreshes at 10 Hz to avoid serial stalls
- Mirrors touch tuning constants from `src/main.cpp` — useful for tuning without reflashing the synth
- Added `tools/test-slider` and `tools/mpr121_calibrate` to `Makefile` comments

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
