# Glass Touch Bass Synth
### Daisy Seed · MPR121 · Bit-Bang I2C · C++

A capacitive touch synthesiser played through a glass surface. One or two fingers control pitch, filter, distortion and detune in real time. Built on the Daisy Seed embedded audio platform using a custom bit-bang I2C driver and a software touch tracker with centroid position interpolation.

---

## Hardware

| Component | Detail |
|---|---|
| MCU / Audio | Daisy Seed (STM32H750, 480 MHz, 24-bit 48 kHz) |
| Touch sensor | MPR121 capacitive touch controller (12 electrodes) |
| Interface | Glass surface over bare copper electrodes |
| SDA pin | D12 (PB8) — 4.7 kΩ pull-up to 3.3 V |
| SCL pin | D11 (PB9) — 4.7 kΩ pull-up to 3.3 V |
| MPR121 address | 0x5A (ADDR pin → GND) |
| Audio out | Headphone jack — both channels carry the same mono signal |

### Wiring

```
Daisy D12 ──┬── MPR121 SDA
            └── 4.7kΩ ── 3V3

Daisy D11 ──┬── MPR121 SCL
            └── 4.7kΩ ── 3V3

Daisy 3V3 ──── MPR121 VCC
Daisy GND  ──── MPR121 GND
                MPR121 ADDR → GND   (sets address 0x5A)
```

> **Note on jack wiring:** Use a TRS (stereo, 3-pole) jack. A TS (mono, 2-pole) jack will short the right channel to ground when a TRS headphone plug is fully inserted.

---

## Building

Requires [libDaisy](https://github.com/electro-smith/libDaisy) and the Daisy toolchain.

```bash
# Set LIBDAISY to wherever libDaisy lives on your machine
LIBDAISY=/path/to/libDaisy

# Build the synth
make TARGET=src/main libdaisy_dir=$LIBDAISY SYSTEM_FILES_DIR=$LIBDAISY/core LIBDAISY_DIR=$LIBDAISY

# Flash via USB DFU (hold BOOT, tap RESET, then run):
dfu-util -a 0 -s 0x08000000:leave -D build/src/main.bin
```

To build a diagnostic tool instead:

```bash
make TARGET=tools/test-slider ...same flags...
dfu-util -a 0 -s 0x08000000:leave -D build/tools/test-slider.bin
```

---

## Files

| File | Purpose |
|---|---|
| `src/main.cpp` | Main synth — touch tracker + audio engine |
| `tools/test-slider.cpp` | Live sensor display — delta bars, position and pressure (no audio) |
| `tools/mpr121_calibrate.cpp` | Full AFE + per-electrode pressure calibration tool |
| `tools/mpr121_reader.cpp` | Debug tool — shows raw electrode data and deltas |
| `tools/i2c_scanner.cpp` | Scans I2C bus and finds MPR121 address |
| `tools/osc_test.cpp` | Minimal sine oscillator — confirms audio output works |
| `tools/mpr121_glass_tuner.cpp` | Sweeps CDC/CDT settings, finds best config for glass |
| `tools/mpr121_glass_tuner2.cpp` | Phase 2 tuner — sweeps FFI and ESI settings |

---

## Playing

On boot you will hear a short beep confirming audio is working. The serial monitor then shows electrode data and finger positions.

**Finger 1** controls the main voice:
- **Position** (left → right) — pitch from 50 Hz to 300 Hz, quantised to the nearest chromatic note on touch-down, then continuous glide while sliding
- **Pressure** — opens the low-pass filter from dark/closed to wide open; also adds vibrato and drive at high pressure

**Finger 2** adds timbre modulation:
- **Position** (left = flat, centre = unison, right = sharp) — detunes the second oscillator up to ±1.5 semitones
- **Pressure** — adds wavefold distortion (subtle warmth at medium, gritty at high); also gates how much detune is applied

**Calibration:** keep fingers off the glass for 2 seconds after power-on — the baseline is captured automatically. It recaptures every 2 seconds of idle time to compensate for drift.

---

## Signal Chain

```
Osc 1  (fundamental, triangle)      × 0.50
Osc 2  (detuned copy, triangle)     × 0.22   ← finger 2 position controls detune
Sub    (octave below, triangle)      × 0.18
Osc Oct (octave above, triangle)     × 0.15
         │
         ▼
    Oscillator mix
         │
    Ring modulator  ← finger 2 extreme pressure
         │
    Waveshaper (fast_tanh drive)  ← finger 1 pressure
         │
    Moog 4-pole ladder filter  ← finger 1 pressure (exponential cutoff sweep)
         │
    Wavefold distortion  ← finger 2 pressure
         │
    Output soft clip (fast_tanh × 1.3)
         │
    Amplitude envelope
         │
    L + R out (identical / mono)
```

---

## Touch Detection Parameters

These constants at the top of the file control touch sensitivity and finger tracking.

```cpp
static constexpr int32_t  TOUCH_THRESHOLD    = 10;
```
Minimum capacitance delta (counts) to register a touch. **Raise** if you get false triggers from vibration or electrical noise. **Lower** if touches are not detected reliably. Range: typically 5–20 for glass surfaces. Current value: **10** (tuned for 3mm soda-lime glass).

```cpp
static constexpr int32_t  PRESSURE_MAX_REF[12] = {
    100,60,60,60,60,60,
    60,60,60,60,60,100
};
```
Per-electrode delta count that maps to 100% pressure. Edge channels (0 and 11) are set higher because they naturally produce larger deltas even with moderate pressure. Tune each entry by pressing firmly on that electrode and noting the peak delta in `tools/test-slider`.

```cpp
static constexpr int32_t  PRESSURE_DEAD_ZONE[12] = {
    30,23,13,13,13,13,
    13,13,13,13,23,35
};
```
Per-electrode delta below which pressure reads 0%. Edge channels have higher values to compensate for their greater sensitivity — without this, a light touch on channel 0 or 11 would immediately register as medium pressure. Should be set to roughly the delta produced by the lightest intentional touch on each electrode.

```cpp
static constexpr bool QUANTIZE_ENABLED = true;
```
When `true`, touch-down snaps to the nearest note in `SCALE[]`. Once sliding, pitch follows the finger continuously. Set to `false` for fully continuous pitch from first contact — no snapping at all.

```cpp
static constexpr int32_t  MIN_FINGER_SEP     = 4;
```
Minimum electrode separation required to recognise a second finger. Prevents a single wide finger blob from being misread as two fingers.

```cpp
static constexpr uint32_t REBASELINE_IDLE_MS = 2000;
```
Milliseconds of no touch before the baseline is automatically recaptured. Lower values adapt faster to temperature/humidity drift but may interrupt playing. 2000–5000 ms is a good range.

```cpp
static constexpr int32_t  MAX_POS_JUMP       = 200;
```
Maximum position change (0–1000 scale) per frame to maintain finger identity. If a finger "teleports" further than this it is treated as a new finger. Lower values make tracking more stable; higher values allow faster sliding.

```cpp
static constexpr int      REBASELINE_SAMPLES = 32;
```
Number of electrode readings averaged for each baseline capture. Higher = more accurate baseline, longer calibration pause.

---

## Musical Mapping

```cpp
static constexpr float FREQ_LOW  = 50.0f;   // Hz at electrode 0 (left)
static constexpr float FREQ_HIGH = 300.0f;  // Hz at electrode 11 (right)
```
Frequency range of the instrument. The mapping is **exponential** (equal musical intervals per unit of travel), so the spacing feels even as you slide.

```cpp
static const int SCALE[] = {0,1,2,3,4,5,6,7,8,9,10,11}; // chromatic
```
Scale used for note quantisation on touch-down. Change this array to restrict to a specific scale:

| Scale | Array |
|---|---|
| Chromatic (default) | `{0,1,2,3,4,5,6,7,8,9,10,11}` |
| Major | `{0,2,4,5,7,9,11}` |
| Natural minor | `{0,2,3,5,7,8,10}` |
| Pentatonic major | `{0,2,4,7,9}` |
| Pentatonic minor | `{0,3,5,7,10}` |
| Blues | `{0,3,5,6,7,10}` |
| Dorian | `{0,2,3,5,7,9,10}` |

After quantising on touch-down, sliding the finger continuously tracks the raw position — giving you glide and expression within a note.

---

## Envelope Parameters

```cpp
static constexpr float SLEW_AMP_A  = 0.9700f;  // attack  (~3ms)
```
Amplitude attack speed. **Lower** = faster (more percussive). **Higher** = slower fade-in (more organ-like). Range 0.80–0.99. Below 0.95 you may hear onset clicks.

```cpp
static constexpr float SLEW_AMP_R  = 0.999971f; // release (~5 seconds)
```
Amplitude release speed. This controls how long notes sustain after lifting the finger. Reference values:

| Value | Approximate release time |
|---|---|
| `0.9990f` | ~100 ms |
| `0.9995f` | ~200 ms |
| `0.9998f` | ~500 ms |
| `0.99993f` | ~2 seconds |
| `0.999971f` | ~5 seconds |
| `0.999986f` | ~10 seconds |

---

## Filter Parameters

```cpp
float cutoff_oct = 12.0f * prs_cut;   // octaves of sweep
```
Total filter sweep range in octaves. At 12 octaves and a base of `0.3 × freq`, full pressure opens the filter from near-silent to 18 kHz at any pitch. **Raise** for more dramatic sweep; the practical ceiling is about 14 octaves before the filter becomes numerically unstable.

```cpp
float cutoff = clampf(freq * 0.3f * oct_mult, 20.0f, 18000.0f);
```
- `0.3f` — base multiplier. Sets where the filter sits at zero pressure relative to the played note. `0.25` is very dark; `0.5` is slightly brighter at rest.
- `18000.0f` — hard ceiling in Hz. Raise to `20000.0f` to allow the filter to scream at its absolute maximum.

```cpp
float filtered = moog(mix, eff_cutoff, 0.75f, sr);
```
Filter resonance (Q). At `0.75` the filter peak is very audible and approaching self-oscillation. **Raise toward 1.0** for more resonance scream (will self-oscillate above ~0.85 depending on cutoff frequency). **Lower toward 0.3** for a smoother, warmer sound.

```cpp
float prs_exp  = prs01 * prs01 * prs01;   // power-3 pressure curve for cutoff
```
Pressure curve shape for the filter. Power-3 (cubic) stays dark through the first half of pressure and opens explosively in the upper range. **Power-2** is more responsive at low pressure. **Power-4** stays even darker until you really press.

---

## Oscillator Mix

All oscillators are triangle waves. Adjust amplitudes to taste:

```cpp
float osc1    = tri(s_phase1)   * 0.50f;   // fundamental
float osc2    = tri(s_phase2)   * 0.22f;   // detuned copy (finger 2 position)
float sub     = tri(s_phase_sub)* 0.18f;   // one octave below
float osc_oct = tri(s_phase_oct)* 0.15f;   // one octave above
```

Total mix amplitude = 0.50 + 0.22 + 0.18 + 0.15 = 1.05. Keep the sum below ~1.2 to avoid driving the waveshaper too hard by default.

Oscillator frequencies:
- **osc1**: fundamental + vibrato
- **osc2**: fundamental × `det_ratio` (detune from finger 2)
- **sub**: fundamental × 0.5 (one octave below)
- **osc_oct**: fundamental × 2.0 (one octave above)
- **rm_carrier**: fundamental (ring mod only, not summed into mix directly)

---

## Finger 2 Effects

```cpp
float detune_pos  = (pos01 - 0.5f) * 3.0f;   // ±1.5 semitones
```
Maximum detune range. Finger 2 at centre = no detune. Left = flat, right = sharp. Range `3.0f` = ±1.5 semitones (subtle chorus). Raise to `12.0f` for a full ±6 semitone spread.

```cpp
float bc  = smootherstep2(ss3) * 0.15f;
```
Wavefold distortion maximum amount. `0.15` = subtle warmth at maximum pressure. Raise to `0.35` for audible grit, `0.60` for heavy distortion. The triple-smootherstep curve means the effect only arrives at high pressure regardless of this cap.

```cpp
float rm = smoothstep2(rm_in) * 0.15f;   // ring mod cap
```
Ring modulation maximum wet level. Only activates above 85% finger 2 pressure. `0.15` is barely perceptible — raise to `0.40` for a more metallic character.

---

## Slew Rates Reference

All slew parameters use a one-pole lowpass form: `s = s × pole + target × (1 − pole)`. Higher pole value = slower response.

| Constant | Value | Response | Controls |
|---|---|---|---|
| `SLEW_FREQ` | 0.9985 | ~14 ms | Pitch glide between notes |
| `SLEW_CUT` | 0.9800 | ~1 ms | Filter cutoff opening speed |
| `SLEW_MISC` | 0.9900 | ~2 ms | Drive and vibrato |
| `SLEW_F2_A` | 0.9800 | ~10 ms | Finger 2 effect attack |
| `SLEW_F2_R` | 0.9990 | ~200 ms | Finger 2 effect release |
| `SLEW_AMP_A` | 0.9700 | ~3 ms | Amplitude attack |
| `SLEW_AMP_R` | 0.999971 | ~5 s | Amplitude release |

---

## MPR121 AFE Configuration

These values were determined by the sensitivity sweep tools and tuned for 3 mm soda-lime glass.

```cpp
mpr_write(REG_CONFIG1, 0x50);  // FFI=10 iterations | CDC=16 µA charge current
mpr_write(REG_CONFIG2, 0x6C);  // CDT=16 µs charge time | SFI=18 iterations | ESI=1 ms
```

| Register | Bits | Value | Meaning |
|---|---|---|---|
| CONFIG1 `[7:6]` | FFI | 01 = 10 iters | First filter iterations — smoothing |
| CONFIG1 `[5:0]` | CDC | 010000 = 16 µA | Charge current — lower is better through glass |
| CONFIG2 `[6:4]` | CDT | 110 = 16 µs | Charge time — longer for glass dielectric |
| CONFIG2 `[3:2]` | SFI | 11 = 18 iters | Second filter iterations |
| CONFIG2 `[1:0]` | ESI | 00 = 1 ms | Electrode sample interval |

Touch threshold: **4** counts. Release threshold: **2** counts. (Default is 12/6 for bare electrode — glass requires lower thresholds.)

### Tuning for different glass

If signal is too weak (thin glass, bad contact): lower `TOUCH_THRESHOLD`, lower `PRESSURE_MAX_REF`, or try CDC=16µA with CDT=32µs (CONFIG2 = `0x7C`).

If signal is too noisy (false triggers): raise `TOUCH_THRESHOLD`, or switch to FFI=34 (CONFIG1 = `0xD0`).

---

## Serial Monitor Output

The main synth (`src/main.cpp`) does not print to serial during play — all serial output has been removed from the audio loop to keep touch polling fast.

Use `tools/test-slider` for live sensor monitoring. Flash it and open at 115200 baud — it refreshes at ~10 Hz:

```
 CH | DELTA | BAR
----+-------+--------------------
  0 |     0 | [                    ]
  5 |    18 | [################    ] 1
  6 |    12 | [###########         ]
 ...
 10 |    22 | [####################] 2
 11 |     8 | [#######             ]

POS  |----------1--------------------2----------|
  1  pos: 421  prs:  63%  [##########      ]
  2  pos: 834  prs:  41%  [######          ]
  freq:  97Hz  cut: 2840Hz  det:+0.8s  fx: 4%  vib: 0%
--------------------------------------------
```

- **CH**: Electrode number (0–11, left to right)
- **DELTA**: Capacitance change from baseline (counts). Resting = 0, touch = 10–35 through 3mm glass
- **BAR**: Visual bar scaled to the current peak delta. `1` / `2` marks the peak channel for each finger
- **POS**: Position track — `1` and `2` show finger positions across the slider range
- **pos**: Finger position 0–1000 (left to right)
- **prs**: Pressure 0–100% (sqrt-mapped from raw delta)
- **freq**: Current synthesised frequency in Hz
- **cut**: Current filter cutoff in Hz
- **det**: Osc 2 detune in semitones
- **fx**: Wavefold amount %
- **vib**: Vibrato depth %

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| MPR121 not found at 0x5A | SDA/SCL swapped | Check D12=SDA, D11=SCL |
| MPR121 not found (scanner works) | Audio ISR interfering with I2C | Ensure MPR121 init is before `StartAudio()` |
| All deltas stuck at 0 | Wrong register address | Electrode data starts at 0x04, not 0x1C |
| Baseline much lower than raw | Baseline not settled | Increase post-init delay or use software baseline |
| Clicks on note start | Attack too fast | Raise `SLEW_AMP_A` toward 0.97 |
| False touches at threshold 5 | Glass vibration noise | Raise `TOUCH_THRESHOLD` to 7–10 (update `PRESSURE_DEAD_ZONE` to match) |
| Clicks on note end | Cutoff jumps on lift | Cutoff should track amplitude during release — check the release coupling code |
| False touches | Threshold too low | Raise `TOUCH_THRESHOLD` |
| Pressure always 0% | `PRESSURE_MAX_REF` too high | Lower to match real peak delta seen in monitor |
| Pressure always 100% | `PRESSURE_MAX_REF` too low | Raise to 2–3 counts above typical maximum |
| Two fingers not detected | Fingers too close | Need at least `MIN_FINGER_SEP` (4) channels of separation |
| Finger identity swaps | Fingers moving faster than `MAX_POS_JUMP` | Raise `MAX_POS_JUMP` |
| No audio | Codec not initialised | Ensure `hw.Init()` → MPR121 init → `hw.StartAudio()` order |
| Audio only on one side | TS jack with TRS plug | Use TRS jack or mono headphone |

---

## Possible Extensions

- **MIDI output** — map finger 1 position to MIDI note + pitch bend, pressure to aftertouch
- **Reverb** — a simple Schroeder reverb or plate reverb tail would suit the long release
- **Different waveforms** — swap `tri()` for a sawtooth or add a square wave to osc1 for more edge
- **Vibrato LFO sync** — tie LFO rate to tempo from an external clock input
- **Second glass slider** — add a second MPR121 for a two-dimensional control surface
- **Custom slumped glass** — kiln-formed glass (borosilicate, Bullseye 96) with controlled thickness gives stronger and more uniform signal than drinking glass walls

---

*Built iteratively with Claude — Daisy Seed firmware, touch sensor tuning, and DSP all developed through conversation.*
