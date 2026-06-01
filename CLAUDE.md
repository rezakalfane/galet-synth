# GaletSynth — Claude Code Context

## Project

Glass touch bass synth for Daisy Seed + MPR121 capacitive sensor.
- Main firmware: `src/main.cpp`
- Diagnostic / calibration tools: `tools/*.cpp`

## Build

libDaisy lives at `/Users/username/Workspaces/Daisy/DaisyExamples/libDaisy`.
Always pass all three flags — libDaisy's own Makefile requires them:

```bash
LIBDAISY=/Users/username/Workspaces/Daisy/DaisyExamples/libDaisy

make TARGET=src/main \
  libdaisy_dir=$LIBDAISY \
  SYSTEM_FILES_DIR=$LIBDAISY/core \
  LIBDAISY_DIR=$LIBDAISY
```

For a tool:
```bash
make TARGET=tools/test-slider ...same flags...
```

After `make clean`, always recreate build directories before building:
```bash
mkdir -p build/src build/tools
```

## Flash

Put Daisy in DFU mode (hold BOOT, tap RESET), then:

```bash
dfu-util -a 0 -s 0x08000000:leave -D build/src/main.bin
# or for a tool:
dfu-util -a 0 -s 0x08000000:leave -D build/tools/<name>.bin
```

The `dfu-util: Error during download get_status` at the end is normal — the device booted successfully.

## Serial monitor

```bash
screen /dev/tty.usbmodem* 115200
```

The main synth prints a live status display (voice, electrode bars, finger
pos/pressure, FSR, audio params) — but **throttled to ~8 Hz** so it can't slow
the control loop. The control loop itself runs as fast as the sensor allows
(~150–200 Hz, capped by `CONTROL_DELAY_MS`) for tight timing; the display rate
is `PRINT_INTERVAL_MS`. `tools/test-slider` is still handy for sensor-only view.

## Known gotcha — Makefile LDFLAGS

The `Makefile` must **not** contain:
```makefile
LDFLAGS = -specs=nano.specs -specs=nosys.specs
```
libDaisy already adds these flags. Duplicating them causes a fatal linker error:
`attempt to rename spec 'link' to already defined spec 'nano_link'`

If this error appears, remove the `LDFLAGS` line from `Makefile`.

## Key constants in src/main.cpp

| Constant | Line ~| Purpose |
|---|---|---|
| `VOICES[]` / `g_voice_idx` | ~243 | **Voice bank** + active index. Cycled live by the FSR-hold gesture; `g_voice_idx` initializer = boot voice. See [Voices](#voices) |
| `VOICE_SWITCH_FIRST_MS` / `_REPEAT_MS` | ~263 | FSR-hold gesture timing (3 s to first switch, then every 2 s) |
| `MAX_FINGERS` | ~36 | Fingers tracked (4) — polyphony count for the Drums MultiVoice |
| `FIX_DRUM` | ~430 | `true` = every drum tap is full 100% pressure (consistent); `false` = `vel_sens` dynamics |
| `MULTI_ZONES[]` / `MULTI_INTERVALS[]` | ~425 | Drums: zone→drum map and the per-zone pitch intervals |
| `TAP_GAP_MS` / `RISE_THRESH` (in `main`) | — | Lift debounce, and held re-attack sensitivity (drum retrigger tuning) |
| `TOUCH_THRESHOLD` | 39 | Min delta to register touch |
| `PRESSURE_MAX_REF[12]` | 40 | Per-electrode 100% pressure delta |
| `PRESSURE_MIN_REF[12]` | 40 | Per-electrode 0% pressure delta (was `PRESSURE_DEAD_ZONE`) |
| `MIN_FINGER_SEP` | 44 | Min electrode gap for two-finger detection |
| `QUANTIZE_ENABLED` | ~247 | `false` = fully continuous pitch, no snapping |
| `SLEW_CUT` | ~525 | Cutoff opening slew (~35 ms — smooths stepped pressure targets) |
| `NOISE_HP_COEF` | ~660 | High-pass corner for `noise_hp` voices (higher = brighter "tsss") |
| `REG_CONFIG1 / CONFIG2` | ~270 | MPR121 AFE registers (FFI, CDC, CDT, ESI) |

> Pitch glide and the amp attack/release used to be the global `SLEW_FREQ` /
> `SLEW_AMP_A` / `SLEW_AMP_R` constants. They are now **per-voice** (`glide_ms`,
> `attack_ms`, `release_ms`) — see Voices below.

## Voices

A **Voice** bundles everything that defines a sound. Every parameter that shapes
the tone lives in one `Voice` struct (`src/main.cpp:71`). All presets sit in the
`VOICES[]` bank (`src/main.cpp ~243`); the active one is a **runtime** index:

```cpp
static constexpr Voice VOICES[] = { VOICE_LEAD, VOICE_BASS, ... };
static volatile int    g_voice_idx   = 2;   // SELECTED voice (gesture/header); boot value here (OpenBass)
static volatile int    g_active_voice = 2;  // ACTIVE voice the engine plays
```

Two indices: **`g_voice_idx`** is what the FSR gesture selects and the header
shows; **`g_active_voice`** is what the audio engine actually plays. They're
equal except in MultiVoice mode (below), where the touch loop routes
`g_active_voice` per tap. The audio callback **snapshots `g_active_voice` once
per block** (`const Voice& VOICE = VOICES[vi]`) so a mid-block switch can't tear
fields; the touch loop rebinds its own `VOICE` each frame after MultiVoice
resolution. Envelope/glide coefficients (`ms_to_coeff`) recompute only when the
index changes. Because selection is runtime, the per-sample waveform dispatch /
noise / keytrack / osc2 branches no longer fold away — negligible on the H750.

### MultiVoice — "Drums": 4-voice polyphonic kit (`VOICE_DRUMS`, `MULTI_*`)

`VOICE_DRUMS` (kept **last** in the bank, `MULTI_IDX = NUM_VOICES-1`) is a meta-
voice: when selected, the slider splits into 4 equal zones (`MULTI_ZONES[]` —
Kick, Snare, Tom, Hat by bank index). It is **polyphonic** — up to `MAX_FINGERS`
(4) fingers each play their own drum at once.

- **Detection**: `detect_raw` finds up to 4 separated fingers; `tracked[4]`. The
  mono melodic path still only reads `tracked[0]`/`tracked[1]`.
- **Engine**: a pool of `DrumHit g_hits[POLY]` — each a self-contained voice
  (oscillators + noise + **its own** 1-pole lowpass `lp_*`, not the 4-pole Moog,
  for CPU; + amp/pitch envelopes). `drum_render` sums them; the mono engine is
  held silent (`g_amp_target = 0`) and the callback uses the sum when
  `g_voice_idx == MULTI_IDX`. `drum_trigger` latches drum + pitch + velocity-
  derived cutoff/drive per hit. Reuses `moog_st` is **not** used here (1-pole).
- **Pitch**: zone → drum; fine position within the zone snaps to
  `MULTI_INTERVALS[]` (root / 4th / 5th / octave) above the drum's `freq_low`.
- **Velocity / `FIX_DRUM`**: with `FIX_DRUM = true` (default) every tap is full
  100% pressure (consistent hits); set false for `vel_sens` dynamics.
- **Retrigger** (per finger): a genuine tap (real lift + re-tap, off ≥
  `TAP_GAP_MS`) retriggers with **no limit**; a sub-`TAP_GAP_MS` tracker dropout
  is debounced as a flicker (no spurious hit); while held, a sharp pressure rise
  (`RISE_THRESH`) re-attacks, rate-capped by the voice's `retrig_ms`.

The `VOICE_DRUMS` struct itself is a Kick-like placeholder so it shows in the
bank/gesture/header ("Drums"); its own audio fields aren't used (the zone drums
are). Other voices stay monophonic; in mono a held re-attack re-articulates via
`g_retrig` (only voices with `retrig_ms > 0`).

### Switching voices live — FSR-hold gesture

Hold the FSR pressed to the mute floor (`fsr_raw <= FSR_MIN`): the first voice
advance fires after `VOICE_SWITCH_FIRST_MS` (3 s), then it keeps advancing every
`VOICE_SWITCH_REPEAT_MS` (2 s) while still held; releasing re-arms the 3 s wait.
The cycle **skips voices flagged `no_cycle`** (`cycle_next`) — so the raw drums
are out, and the gesture steps through the instruments + the Drums MultiVoice.
Each switch flashes all three LEDs **N times = position in the cycle**
(`cycle_pos`, via `flash_voice_leds`) and prints `[voice n/total] name`. The
gesture freezes the idle-chase / rebaseline timers while held. To change the
**boot** voice, edit the `g_voice_idx` initializer; to change cycle order or
membership, reorder `VOICES[]` or flip `no_cycle`.

### Voice struct fields

| Field | Meaning |
|---|---|
| `name` | Printed in the boot banner |
| `freq_low`, `freq_high` | Pitch range finger-1 sweeps across (exponential) |
| `log_freq_ratio` | **Precomputed** `ln(freq_high/freq_low)` (`logf` isn't `constexpr`) |
| `osc{1,2,sub,oct}_ratio` | Pitch ratio vs the fundamental (1.0=root, 0.5=octave down, 2.0=up, `RATIO_FIFTH`=perfect 5th) |
| `osc{1,2,sub,oct}_level` | Mix level (0 silences that oscillator) |
| `osc{1,2,sub,oct}_wave` | `WAVE_TRI` / `WAVE_SINE` / `WAVE_SQUARE` / `WAVE_SAW` (per oscillator) |
| `osc2_detune` | `true`: finger-2 position detunes osc2 (lead). `false`: osc2 holds `osc2_ratio` (e.g. a fixed 5th, or a chorus/scream detune like 1.005) |
| `cutoff_mult` | Base cutoff = (tracked freq) × this |
| `cutoff_oct_max` | Octaves the cutoff opens across finger-1 pressure |
| `resonance` | Moog ladder Q. ~0.9 ≈ self-oscillation scream; ~0.3 = smooth |
| `drive_max` | Pressure-driven waveshaper gain on top of 1.0 |
| `noise_level` | White noise mixed into the stack (breath/chiff/hiss; 0 = none) |
| `keytrack` | 0..1 — how much the cutoff base follows pitch (1 = full; <1 holds cutoff lower as pitch rises) |
| `ringmod_ratio` | Ring-mod carrier pitch ratio. 1.0 = unison; non-integer (e.g. 2.5) = inharmonic bell/metallic |
| `ringmod_max` | Max ring-mod wet from finger-2 pressure (gated >85%) |
| `fold_max` | Max wavefold amount from finger-2 pressure |
| `attack_ms` | Amp envelope attack time |
| `release_ms` | Amp envelope + filter-close release time |
| `glide_ms` | Portamento between pitches |
| `pitch_env_oct` | Pitch envelope: octaves the pitch starts ABOVE the note at onset, decaying back (0 = off). Kick "boom", tom thump, zap |
| `pitch_env_ms` | Time the pitch envelope takes to settle to the note |
| `noise_hp` | Noise tone: 0 = white ("shhh"), 1 = high-passed ("tsss", for hi-hats/cymbals). Uses the global `NOISE_HP_COEF` corner |
| `no_cycle` | `true` = skip this voice in the FSR-gesture cycle (still reachable as boot voice or via the Drums MultiVoice). The 4 raw drums set this. Default `false` = in cycle |
| `vel_sens` | 0..1 velocity sensitivity: onset hit strength (peak pressure, latched at the tap) scales loudness; loudness floor = `1 - vel_sens`. Drums = 0.85; default 0 = fixed loudness (the sustained instruments) |
| `retrig_ms` | Held re-attack rate cap (ms): min interval between pressure-bounce repeats while a finger stays down. 0 = no held re-attack (melodic default). Genuine taps ignore this — only the held-bounce path is capped. Drums ~400 |

Envelope/glide are in **milliseconds**, converted to per-sample slew
coefficients (`ms_to_coeff`) in the audio callback, recomputed on voice change.
The pitch envelope retriggers on each note onset (amp target rising from
silence). All three fields default to 0 (off) for voices that omit them.

### Notes

- **Waveforms** (`enum Waveform`, `src/main.cpp:69`): sine uses a parabolic
  approximation; square/saw are naive (they alias at high pitch, fine on bass).
- **Adding a voice**: copy a `VOICE_*` block, retune, then add it to `VOICES[]`
  (it joins the FSR cycle). Fields are positional — keep the trailing
  `// noise, keytrack, ...` extras line in the right order.
- The finger-2 effect *mappings* (ring-mod gated >85% pressure, wavefold curve,
  detune range) are still shared by all voices; only their **ceilings**
  (`ringmod_max`, `fold_max`) and the carrier ratio are per-voice.

### Current presets

| `VOICE_*` | Character |
|---|---|
| `LEAD` | Original glass-Moog lead — triangle, finger-2 detune, expressive |
| `BASS` | Round & dark power chord (osc2 = fixed 5th → root+5th+oct+sub) |
| `BASS_OPEN` | Same chord stack, brighter base + wider pressure sweep |
| `BASS_RICH` | Mixed waveforms (saw/square/sine), dark base, huge 13-oct acid sweep |
| `ORGAN` | Clean all-sine organ/flute, melodic register, 1.005× chorus, breath noise |
| `SCREAM` | Aggressive detuned saws, high register, heavy drive, near-self-osc res, inharmonic (2.5×) metallic ring-mod |
| `BASS_CLOSED` | Deep muffled sub — filter near the fundamental, tiny sweep, reduced keytrack, long tail |
| `PAD` | Warm ensemble pad — detuned saws + tri sub + sine top, dark filter, very slow ~2.2 s swell, long tail |
| `TOM` | Noise-driven percussion — **tap to play**. Dark/round tom: noise + low sine body, pitch env (1 oct/80 ms), short tail; position tunes 40–400 Hz |
| `KICK` | **Tap** — deep punchy thud, low sine + sub, 2-oct/45 ms pitch "boom", very closed filter; tunes 30–80 Hz |
| `SNARE` | **Tap** — bright noisy rattle over two tonal modes, open filter, quick 0.7-oct/30 ms snap; tunes 150–500 Hz |
| `HIHAT` | **Tap** — crisp high "tsss": high-passed noise (`noise_hp=1`) + faint metallic squares, short sizzle tail; tunes 4–11 kHz |
| `DRUMS` | **MultiVoice** — slider splits into 4 zones (Kick/Snare/Tom/Hat); each tap triggers the zone's drum, fine position snaps pitch to root/4th/5th/octave (see the MultiVoice subsection above) |

## Hardware

- **MCU**: Daisy Seed (STM32H750)
- **Sensor**: MPR121 at I2C address `0x5A`, bit-bang on SDA=D12, SCL=D11
- **Glass**: 3mm soda-lime over copper electrodes

## Branching convention

Feature branches: `feature/<name>` → merge to `main` → delete branch.
Update `CHANGELOG.md` after merging.
