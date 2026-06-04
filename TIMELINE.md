# 🎛️ GaletSynth — Development Timeline

A glass touch bass synth for **Daisy Seed + MPR121** that grew into a full voice
system, a polyphonic drum kit, and a companion macOS tuning app.

Reconstructed from the commit graph, branch merges, and dates. The workflow
throughout: `feature/*` branch → `--no-ff` merge into `main` → delete branch →
update `CHANGELOG.md`.

---

## 📅 Apr 4–7 2026 · Phase 0 — Foundations
The bare instrument and the bench tools.
- `06d2b81` **Initial commit** — glass touch bass synth (Daisy Seed + MPR121)
- `f88324d` calibration tool → `5b72465` per-electrode pressure calibration + smooth pitch sliding
- `21ae1df` `QUANTIZE_ENABLED` flag · `3eabfd1` `test-slider` diagnostic tool
- `702aae3` first **CLAUDE.md** project context

> *~7-week gap — hardware bring-up (FSR + LEDs added next).*

---

## 📅 May 31 · Phase 1 — The Voice system & sound design
The conceptual leap: a sound becomes a **`Voice`**.
- `65edf15` FSR sensor + LEDs
- `f2905a8` **Voice abstraction** — switchable presets, per-oscillator waveforms
- `5a95be0` Organ / Scream / ClosedBass · `f5d4c15` per-voice envelope / glide / keytracking
- `d882fb0` **live voice switching via the FSR-hold gesture**
- `9ed8c9d` Pad · `fcc7135` Tom · `9059676` drum kit (Kick/Snare/Hat) + pitch-env + noise HP

---

## 📅 Jun 1 · Phase 2 — Polyphony, persistence & modularization
From mono toy to a structured, multitouch instrument.
- `36132ac` **MultiVoice "Drums"** — 4-zone playable kit · `08d2a95` **4-voice polyphony + multitouch**
- `ad5c24e` faster control loop + per-voice velocity · `50c4d6e` `no_cycle` flag
- `32a38c9` **persist the selected voice across power cycles** (QSPI)
- `190b305 → 8261208` **split `main.cpp` into modules** (config/dsp/voice → mpr121/touch → engine → host unit tests)
- `f2f104a` MPR121 ESI 16→1 ms (fast taps) · `400c74d` per-LED brightness (less LED→audio noise)

---

## 📅 Jun 2 · Phase 3 — Expressive voices & effects
*(merges: `voice-select-preview`, `guitar-voice`, `sh101-voice`, `sh101-chords`, `reverb-delay`)*
- `d3d910a` **tap-the-glass voice select with live preview** + per-voice quantize/scale
- `3aec3c2` Guitar + amp decay-to-sustain · `885e87e` **SH-101** mono synth
- `47ec8ce` **SH-101 diatonic chords** (minor + major twins) → open voicing → per-voice chord fields
- `cc038c9` **shared reverb + delay** with per-voice sends (Organ first)

## 📅 Jun 2 · Phase 4 — The Voice Tuner / VoiceLab toolchain
Design voices on the laptop, against live hardware.
- `701fecd` **live voice tuner over USB serial** + `voicelab.py` CLI → **PySide6 GUI** + shared `galetsynth` core
- `0ee170b` **editable + persistent voice bank** (rename / save / revert)
- `066fb1b` whole-bank **backup/restore to JSON** + tooltips
- `51b63cf` mon dashboard alignment + clean disconnect · `562b8c6` **copy a voice to another slot**

---

## 📅 Jun 3 · Phase 5 — Packaging
- `349b8e6` GUI polish (toolbar, group icons, offline export)
- `a2158f2` **package the tuner as a standalone macOS `VoiceLab.app`** (PyInstaller) + docs

---

## 📅 Jun 4 · Phase 6 — LFO, real-voice drums & VoiceLab hardening 🚀
Three feature branches, each branch → merge → delete, all pushed to `origin/main`.
- `e10c957` **per-voice LFO** — vibrato / auto-wah / tremolo
  - 🐛 caught on hardware: `Voice` grew without bumping `BANK_MAGIC` → corrupt QSPI
    bank (no sound, "Drums" reading as "Kick", saves reverting). Fixed, plus
    VoiceLab slider-clamp / dump-snapshot / Drums-label robustness.
- `936aae8` **Drums kit plays the real voices** — each hit now runs the 4-pole Moog
  ladder + its drum's resonance (not the 1-pole approximation), with **velocity
  dynamics** (`FIX_DRUM = false`)
- `8b0c5c2` **VoiceLab save confirmation** (+ voice numbers in dialogs) + CR/LF dump
  de-merge + backup/restore progress-dialog re-entrancy fix

---

## 📅 Jun 4 · Phase 7 — USB audio (in progress, on `usb-audio`) 🎙️
Goal: make the synth a composite **USB Audio (capture) + CDC** device so it records
live in a DAW while VoiceLab keeps working. Built on an **integration branch**
(`usb-audio` off `main`), one sub-branch per phase merging in; **`main` stays
untouched until the whole project works.** (Design: `docs/usb-audio-plan.md`.)
- `25bd84d` **plan** — composite UAC + CDC over TinyUSB, run from QSPI/SRAM.
- **Phase 0 — run from the bootloader** (`feature/usbaudio-qspi`): escape the 128 KB
  internal-flash wall (was 93.75% full) so there's room for the USB stack.
- **Phase 0.5 — SRAM boot + persistence** (`feature/usbaudio-persist`): `BOOT_QSPI`
  broke voice persistence (libDaisy refuses QSPI writes while executing from QSPI),
  so switched to **`BOOT_SRAM`** (QSPI stays writable), moved the reverb/delay
  buffers to SDRAM, fixed `SCB->VTOR`, added the `bootvoice` command + VoiceLab
  **"Set as default"** (yellow ★ on the default voice).
- **Phase 1 — TinyUSB CDC** (`feature/usbaudio-tinyusb-cdc`): replaced libDaisy's
  USB logger with a vendored TinyUSB (`lib/tinyusb`); VoiceLab serial now runs over
  TinyUSB CDC (`usb_log`/`usb_cdc_read_avail`). Verified the full protocol on
  hardware. The base for the composite UAC device.
- **Phase 2 — UAC2 stereo capture** (`feature/usbaudio-uac`): composite **CDC +
  UAC2** device — the Daisy is now a class-compliant **2-ch / 48 kHz** USB
  soundcard ("GaletSynth Audio"), recordable live in a DAW while VoiceLab tunes,
  on one cable. Async ring + 47/48/49 clock tracking; the key fix was pumping
  `tud_task()` from the audio callback (~2 kHz) so the iso IN endpoint isn't
  starved by the slow control loop. Verified clean capture on macOS. 🎛️
- **Phase 3 — hardening** (`feature/usbaudio-harden`): a DWT CPU-load meter on the
  audio callback (**17% avg / 30% max** — confirms pumping USB from the audio
  callback is safe), host **mute**, and **lower latency** (cushion 256→128 ≈
  2.7 ms; ring holds steady, zero under/overruns). Confirmed working on macOS **and
  iOS**.
- ⏭️ Next: host testing — Android / TP-7 / TX-6 (UAC1 fallback if needed) — then
  merge `usb-audio` → `main`.

---

### 🌿 Unmerged side branches
- `usb-audio` — the in-progress USB-audio integration branch (Phase 7 above); merges
  to `main` once the soundcard works end to end.
- `update/sensor-jlc` & `update/sensor-jlc-optimized` — sensor / volume / serial-log
  experiments, never merged to `main`.

### 📊 At a glance
| | |
|---|---|
| **Span** | Apr 4 → Jun 4 2026 (~2 months, bulk of the work in 5 days) |
| **Workflow** | `feature/*` → `--no-ff` merge → delete branch → CHANGELOG. Bigger efforts (USB audio) use an **integration branch** (`usb-audio`) with per-phase sub-branches, kept off `main` until done |
| **Arc** | one bass voice → 15-voice bank + poly drum kit → live USB tuner → shipped macOS app → USB-audio soundcard (in progress) |
