# Changelog

All notable changes to GaletSynth are documented here.

## [Unreleased]

### Changed — `refactor: split main.cpp into modules (phase 1 — config/dsp/voice headers)`
- Extracted three header-only modules from `main.cpp`: `config.h` (tuning
  constants), `dsp.h` (oscillators, filters, math, `Waveform`), `voice.h`
  (`Voice` struct, presets, bank, cycling). `main.cpp`: 1855 → 1327 lines
- Still a single translation unit (headers `#include`d by `main.cpp`), so the
  built binary is **byte-identical** — pure code reorganization, no behavior
  change. Later phases will move touch detection, the MPR121 driver, and the
  audio engine into their own `.cpp` units (+ host unit tests)

### Added — `feat: persist the selected voice across power cycles`
- The FSR-selected voice is saved to QSPI flash (`PersistentStorage<PersistSettings>`)
  and restored on boot — turn the synth off and on and it comes back on the last
  voice you chose. Saved on each gesture switch (only writes if changed);
  out-of-range stored values fall back to the default. Works because the app runs
  from internal flash, leaving QSPI free for data

### Added — `feat: 4-voice polyphonic Drums MultiVoice + multitouch`
- **Multitouch**: `detect_raw` now finds up to `MAX_FINGERS` (4) separated
  fingers (`tracked[4]`); the mono melodic path is unchanged (uses [0]/[1])
- **Polyphony in Drums mode**: a pool of independent drum voices (`DrumHit
  g_hits[POLY]`) — each its own oscillators, noise, 1-pole lowpass and amp/pitch
  envelopes — summed in the audio callback. Up to 4 drums sound at once (kick +
  snare + hat…). Melodic voices stay monophonic
- The per-voice Moog ladder is **bypassed in Drums mode** for a cheap per-drum
  **1-pole lowpass** (`lp_*`) — keeps Kick/Tom dark, Snare/Hat bright, at a
  fraction of the CPU for 4 simultaneous voices
- **`FIX_DRUM`** (default true): every drum tap is treated as full 100% pressure
  for consistent hits; set false for `vel_sens` velocity dynamics
- **Retrigger model**: genuine taps (real lift + re-tap, off ≥ `TAP_GAP_MS`)
  retrigger with **no limit**; sub-`TAP_GAP_MS` tracker dropouts are debounced as
  flickers (no spurious hits while held); a held pressure bounce (`RISE_THRESH`)
  re-attacks, rate-capped per voice by `retrig_ms`. Mono voices re-articulate via
  `g_retrig` when `retrig_ms > 0`
- New `Voice` fields `vel_sens` and `retrig_ms`; `moog_st` filter variant takes
  caller-supplied state

### Changed — `feat: faster control loop + per-voice velocity sensitivity`
- **Decoupled the control loop from the serial display.** The display now prints
  throttled (`PRINT_INTERVAL_MS`, ~8 Hz) instead of every frame with a 60 ms
  delay, so the control loop runs as fast as the sensor allows (~150–200 Hz,
  capped by `CONTROL_DELAY_MS`). ~10× finer tap-onset timing and pitch/cutoff
  updates — tight drum rolls, snappier response
- **Per-voice velocity (`vel_sens`, 0..1):** the onset hit strength (peak
  pressure, latched at the tap — `VEL_WINDOW_MS`, currently 0 = instant) scales
  loudness; floor = `1 - vel_sens`. Drums set 0.85 for accents / ghost notes;
  default 0 keeps the sustained instruments at fixed loudness
- Note: the faster loop makes the position LEDs' per-frame smoothing snappier
  (it was tuned for the old ~15 Hz rate) — cosmetic, can be made rate-independent

### Added — `feat: per-voice no_cycle flag — exclude voices from the FSR cycle`
- New `no_cycle` Voice field: voices with it set are skipped by the FSR-hold
  gesture (still reachable as the boot voice or via the Drums MultiVoice)
- The four raw drums (Tom/Kick/Snare/Hi-Hat) are now `no_cycle` — the gesture
  cycles the 8 instruments + the Drums MultiVoice (9 stops) instead of all 13
- LED flash count and the serial voice number are now the **position in the
  cycle** (`cycle_pos` / `cycle_total`), not the raw bank index

### Fixed — `fix: snap DSP state on voice change so voices don't bleed`
- When the active voice changed (notably between MultiVoice drum taps), the
  continuous slewed state — cutoff (slow-closing), pitch, drive, detune,
  bitcrush, ring-mod — and the Moog ladder filter's stored energy carried over
  from the previous voice, so e.g. a Kick after a Hat sounded bright. On a voice
  change these are now snapped to the new voice's targets and the filter is
  cleared. Switches only occur while silent (gesture) or at a tap onset, so the
  snap is click-free and gives each hit a fresh start

### Added — `feat: MultiVoice "Drums" — 4-zone playable drum kit across the slider`
- New `VOICE_DRUMS` MultiVoice: when selected, the slider splits into 4 equal
  zones (`MULTI_ZONES[]` — Kick, Snare, Tom, Hat) and each tap triggers the
  zone's drum. The fine position within a zone snaps the pitch to one of
  `MULTI_INTERVALS[]` (root / 4th / 5th / octave) above the drum's base
- Split the active voice into two indices: `g_voice_idx` (selected — gesture /
  header) and `g_active_voice` (what the engine plays). They match except in
  MultiVoice mode, where each tap routes `g_active_voice` to the zone's drum and
  latches drum + pitch for the whole hit; pressure still drives per-drum brightness
- `VOICE_DRUMS` is a Kick-like placeholder bank entry (kept last, `MULTI_IDX =
  NUM_VOICES-1`) so it appears in the FSR cycle and header as "Drums"
- Serial header shows `Drums [Kick|Snare|Tom|Hat] -> <active drum>`
- Boot voice set to the Drums MultiVoice

### Added — `feat: drum kit (Kick/Snare/Hi-Hat) + pitch-envelope and noise high-pass params`
- New per-voice **pitch envelope** (`pitch_env_oct` / `pitch_env_ms`): the pitch
  starts N octaves above the note at onset and decays back — the kick "boom",
  tom thump, snare snap. Retriggers each note onset; a no-op when octaves = 0
- New per-voice **`noise_hp`**: blends the noise oscillator from full white
  ("shhh") to high-passed ("tsss") via a one-pole high-pass (global corner
  `NOISE_HP_COEF`) — gives hi-hats/cymbals their crisp top
- New tap-to-play percussion voices completing a drum kit:
  - `VOICE_KICK` — deep punchy kick, low sine + sub, 2-oct/45 ms pitch boom
  - `VOICE_SNARE` — bright noisy rattle + tonal modes, open filter, quick snap
  - `VOICE_HIHAT` — high-passed "tsss" + faint metallic squares, short sizzle
  - `VOICE_TOM` also gains a 1-oct/80 ms pitch drop
- Boot voice set to Hi-Hat (`g_voice_idx = 11`); FSR gesture cycles all 12

### Added — `feat: VOICE_TOM — noise-driven percussion`
- New `VOICE_TOM`: a tap-to-play drum built around the noise oscillator over two
  low sine modes (body + an inharmonic 1.6× ring) and a sub thump. Instant
  attack + short ~150 ms release make each tap a hit; a dark, fairly closed
  filter keeps it round (tom, not a bright snare). Finger-1 position tunes the
  drum across 40–400 Hz (kick/tom up). Boot voice set to Tom (`g_voice_idx = 8`)
- Note: this synth's envelope sustains while held, so it's a *tap* instrument —
  holding gives a noise wash, not a decaying one-shot

### Added — `feat: VOICE_PAD — warm ensemble pad`
- New `VOICE_PAD`: two detuned saws (1.006×) for ensemble/chorus width, a soft
  triangle sub and an airy sine octave, through a dark low-resonance filter. The
  pad character is in the envelope — a very slow ~2.2 s attack swell and a ~2 s
  release tail — plus a whisper of noise for air
- Boot voice set to Pad (`g_voice_idx = 7`); the FSR gesture still cycles all 8

### Changed — `feat: show active voice and FSR pressure on the serial screen`
- Serial display gains a `VOICE n/total  Name  (lo-hiHz)` header (updates live as
  the FSR gesture cycles voices) and an `FSR pressure NNN%` row with a bar
  (inverse of master volume: 0% unpressed, 100% pressed to the mute floor)

### Added — `feat: live voice switching via FSR-hold gesture`
- Voices are now **runtime-switchable** (were compile-time `constexpr`). All
  presets live in a `VOICES[]` bank selected by a `volatile g_voice_idx`
- **FSR-hold gesture**: press and hold the FSR to the mute floor — the voice
  advances after 5 s (`VOICE_SWITCH_FIRST_MS`), then every 2 s while held
  (`VOICE_SWITCH_REPEAT_MS`); releasing re-arms. Each switch **flashes the LEDs
  N times** (N = voice number) and prints `[voice n/total] name`
- Audio callback snapshots the voice once per block (no field tearing on a
  mid-block switch); envelope/glide coefficients recompute on voice change
- Gesture freezes the idle-chase / rebaseline timers while held
- Boot voice = the `g_voice_idx` initializer; cycle order = `VOICES[]` order
- Trade-off: the per-sample waveform/noise/keytrack/osc2 branches no longer fold
  at compile time (negligible on the H750); flash usage ~71.6% → ~74.8%

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
