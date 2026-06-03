# Changelog

All notable changes to GaletSynth are documented here.

## [Unreleased]

### Changed — `feat: voice-tuner GUI polish`
- **Toolbar**: the top actions are now compact, flat icon+text `QToolButton`s
  grouped by feature (sync · save/copy/revert · backup/restore · export) with
  vertical separators; `mon` sits on the far right. Icons come from the Qt style
  (`QStyle.standardIcon`, `fromTheme` first) — no bundled assets.
- **Group sections**: each parameter group's title gets a monochrome glyph (e.g.
  `▽ Filter`, distinct per oscillator: `∿`/`≈`/`↓`/`↑`) and a clear gap above it.
- **Export works offline**: it now builds the C++ `Voice{…}` straight from the
  on-screen controls (deriving `log_freq_ratio` from the freq range) instead of
  dumping the device — so Export no longer needs the Galet connected.

### Added — `feat: copy a voice to another slot (rename + save to flash)`
- **Firmware** (`serialtune.cpp`): new `copy <n> [name]` command — duplicates the
  current live voice into bank slot `n` and persists it, **without** changing the
  active voice or interrupting audio (no select-away dance). The optional trailing
  name (spaces allowed) names the copy; omitted keeps the live voice's name. The
  source voice's own name is never touched.
- **GUI**: a **Copy to…** button opens one dialog with a target-slot dropdown and a
  Name field (prefilled + pre-selected with the source name) so the copy can be
  renamed on the way out. The voice being edited stays put.
- **CLI**: `copy <n>` passes through (added to tab-completion + help).

### Added — `feat: whole-bank backup / restore to a local JSON file`
- New shared **`tools/galetsynth/bank.py`**: `backup_bank()` reads every editable
  slot off the device (`select` + `dump`) into a portable JSON document;
  `restore_bank()` replays one back (`select` + `set` + `save` per slot, so each
  lands in flash). It drives the existing tuning protocol — no new protocol. The
  **Drums** meta-voice (`schema.MULTI_IDX`, last slot) is skipped: `select`ing it
  remaps the engine to the Kick slot and its fields are an unused placeholder — the
  four raw drums it plays *are* backed up. Global effect params (`reverb_*`/
  `delay_*`) are stored once and re-applied on restore.
- **GUI**: **Backup…** / **Restore…** buttons. Both run on a worker thread (a few
  seconds of serial traffic) so the window stays responsive, with a modal
  `QProgressDialog` that advances per voice. Backup confirms the filename + location
  at the end, defaulting to a timestamped `YYYYMMDDhhmmss-Galet-Backup.json`.
  Restore opens a **modal pick list** — a checkbox per voice (Select all / none) +
  an optional "import global effects" toggle — so you choose exactly which voices
  to import; only the checked ones are written, then the picker/controls re-sync.
- **CLI**: `backup [file.json]` (auto-names if omitted) / `restore <file.json>`,
  with a one-line textual progress bar; tab-completions added.
- `restore_bank()` takes `only=<indices>` + `do_globals` to back the selective
  restore (full restore is still the default; the CLI is unchanged).

### Added — `feat: voice-tuner GUI tooltips (rich-text info boxes)`
- Per-parameter, per-group and per-button tooltips on the PySide6 tuner, rendered
  as small rich-text boxes (bold header + word-wrapped body, 4px padding). Each of
  the 14 group titles gets an `ⓘ` marker. Help text lives in `schema.py`
  (`HELP` / `GROUP_HELP`) so it stays next to the field model and is HTML-escaped
  (several descriptions contain `<`/`>`).

### Fixed — `fix: status dashboard alignment + clean tuner disconnect`
- **Dashboard alignment**: the host link reader (`link.py`) was stripping leading
  spaces off every line, collapsing the firmware's right-aligned status columns.
  It now keeps leading spaces (trims only the line terminator + trailing padding) —
  fixes the alignment with no reflash. Firmware polish too: channels are
  zero-padded (`00`–`11`) and the touched-channel / finger rows are labelled
  `F1`/`F2`.
- **Clean disconnect**: quitting the tuner used to freeze the synth (needing a
  reboot). libDaisy's USB logger switches to *blocking* transmit once a host has
  drained it, so any autonomous `PrintLine` to a port whose host has gone away
  spins forever. New firmware flag `g_serial_quiet` + a `bye` command: the CLI/GUI
  send `bye` on exit, which leaves tune mode (the synth resumes normal play + the
  idle LED chase) and silences all autonomous serial (dashboard, rebaseline/recal,
  voice-gesture events) so nothing blocks on the closed port. `tune 1` clears it on
  reconnect.

### Fixed — `fix: serial stays responsive while the synth is idle`
- The idle LED-chase loop (`idle_chase_and_calibrate`) spun until a glass touch /
  FSR press without servicing USB serial, so a connected tuner's commands sat
  unparsed — the GUI wouldn't sync at boot and **Backup** stalled until you touched
  the glass. The chase now polls `serial_tune_poll()` and bails the instant tune
  mode is entered, and the synth no longer enters the chase at all while a host
  tuner is connected (`g_live_tune`). The CLI/GUI send `tune 0` on a clean exit so
  normal idle behavior resumes.

### Added — `feat: editable + persistent voice bank (rename/save/revert)`
- **Firmware** (`src/persist.{h,cpp}`): the read-only `constexpr VOICES[]` is now
  copied at boot into a mutable RAM bank **`g_bank[]`** — what the engine, control
  loop and tuner all read. `persist_init()` builds the factory bank then overlays
  any saved edits from QSPI flash; `persist_save_bank()` serializes the whole bank
  (+ `g_voice_idx`) back, writing only when it changed (wear). The `Voice`'s
  `name`/`scale` pointers can't persist verbatim, so the stored form keeps a
  portable name string + a scale id and fixes the pointers up on load (each bank
  voice's name lives in a RAM buffer, so it's editable). A `BANK_MAGIC` guards
  against old/foreign blobs (falls back to factory). Replaces the old voice-index-
  only `PersistentStorage` in `main.cpp`.
- **Serial protocol** (`serialtune.cpp`): `save` (commit the live voice incl. name
  into its bank slot, then flash), `factory [all]` (revert a slot or the whole
  bank to source defaults), `names` (list every slot's current name), and
  `set name <text>` (rename; spaces allowed, e.g. "SH-101 min"). `dump` now also
  emits `idx=` so the host syncs by slot rather than by name (rename-proof).
- **GUI** (`voicelab_gui.py`): a Name field + **Save to flash** / **Revert** /
  **Revert all** buttons; the voice picker labels follow the device's *saved*
  names (via `names`), not just the factory defaults. A connection watchdog
  reconnects and re-syncs if the synth is power-cycled mid-session instead of
  dying on a stale port. `_save()` pushes the current name explicitly right before
  `save` — clicking a button doesn't reliably fire the line-edit's
  `editingFinished` first on macOS, which had been committing the stale factory
  name to flash. **CLI** (`voicelab.py`): bare `save` persists the bank to flash;
  `factory`/`name` tab-completions. `link.py` gains a generic `_capture()` helper
  + `names()`, and `send()` now survives a disconnect.

### Added — `feat: live voice tuner (USB serial) + CLI & PySide6 GUI`
- **Firmware** (`src/serialtune.{h,cpp}`): a USB CDC *receive* path + a small
  line protocol (`tune`/`set`/`select`/`dump`/`mon`/`help`) so a laptop can
  reshape the active voice in real time on the hardware. While `tune 1` is on the
  engine + control loop play a mutable `g_live_voice` (engine seam) instead of the
  const bank voice; `set <field> <value>` edits it live (floats/ints/bools/
  waveforms/scale + the global `reverb_*`/`delay_*` effect params). `g_live_rev`
  triggers a coefficient recompute without a state snap, so glide/cutoff stay
  smooth while tuning. The status display + rebaseline/recal prints are suppressed
  in tune mode (`mon 1` re-enables). Kept tiny for the 128 KB internal flash
  (reuse `StartLog` for TX + add an RX callback; no stdio/atof — hand-rolled
  parsing). FLASH ~93%.
- **Laptop tools** (`tools/`): a shared `galetsynth` package (link / schema /
  codegen — one protocol, one field model, one C++ exporter) with two front-ends
  on top of it: `voicelab.py` (CLI REPL: local echo, tab-completion, `export`/
  `save`/`load`) and `voicelab_gui.py` (PySide6 desktop tuner — a slider/checkbox/
  combo per field grouped into labeled sections across columns, voice picker,
  Refresh, Export to a paste-ready C++ `Voice{...}`, and a resizable log pane that
  renders the `mon` status as a smooth in-place dashboard). pyserial + PySide6 live
  in a project `.venv` (Homebrew/PEP-668); `.gitignore` added.

### Added — `feat: shared reverb + delay with per-voice sends (Organ)`
- New **`Reverb`** (compact mono Freeverb — 8 damped comb filters + 4 allpass
  diffusers) and **`Delay`** (mono feedback line with HF damping so repeats
  darken) DSP structs in `dsp.h`. One shared instance of each lives in the engine
  (`s_reverb`, `s_delay`), with ~66 KB / ~144 KB of delay-line buffers in static
  SRAM (total SRAM use ~45%).
- New per-voice **`reverb_send`** / **`delay_send`** (0..1 aux sends) on the
  `Voice` struct. Reverb and delay run as **parallel sends** off the clean
  instrument signal (they don't cascade), each skipped when a voice's send is 0
  (so dry voices cost nothing and the tail decays naturally on voice change).
- The effect **character is global** (one shared room/echo), via new engine
  params meant to be made playable later: `g_reverb_decay` / `g_reverb_level`
  and `g_delay_time_ms` / `g_delay_feedback` / `g_delay_level`. HF damping is a
  fixed constant for each for now.
- **`VOICE_ORGAN`** is the first/test voice to use them (`reverb_send = 0.6`,
  `delay_send = 0.45`); every other voice zero-inits to dry, unchanged. Both
  send fields appended last in `Voice` so positional initializers stay stable.
- Host tests assert the sends are valid 0..1 fractions and at least one voice
  feeds each effect.

### Changed — `feat: chord settings are per-voice (level + voicing)`
- Moved the two hardcoded chord settings onto the `Voice` struct so each chord
  voice owns its sound: **`chord_level`** (was the `CHORD_UPPER_LEVEL` engine
  constant — level of the two added notes vs the root) and **`chord_spread`**
  (was a hardcoded `+12` in the control loop — octaves to lift the 3rd: 0 = close
  root–3rd–5th, 1 = open root–5th–10th). Both SH-101 twins use `chord_level = 0.9`,
  `chord_spread = 1`, so the sound is unchanged — chord voicing/balance is now
  tunable per voice instead of globally. Host test asserts a chord voice sets
  `chord_level > 0` (else its upper notes would be silent).

### Added — `feat: diatonic chord voices + SH-101 plays triads`
- New per-voice **`Voice::chords`** boolean: when true the voice plays a **3-note
  diatonic triad** instead of a single note. The control loop snaps the root to
  the voice's scale as before, then `diatonic_triad` (new, in `voice.h`) stacks
  scale-thirds (degrees i, i+2, i+4) to get the 3rd and 5th; the offsets become
  frequency ratios (`g_chord_ratio2/3`) the audio engine multiplies the root by.
  Chords therefore stay in key and change quality with position (i, ii°, III…).
- **Engine**: the chord's upper two notes are rendered as extra osc1+osc2 voices
  above the root (sub-osc and noise stay on the root for a tight low end), at
  `CHORD_UPPER_LEVEL` (~0.6×) for drive/filter headroom. They ride the root's
  glide + vibrato via the ratios, and the whole chord passes through one ladder
  filter. The `if(VOICE.chords)` branch folds away for every single-note voice.
- **`VOICE_SH101`** ("SH-101 min") now sets `chords = true` and quantizes to
  **`SCALE_MINOR`** (was chromatic) — a chromatic scale would make stacked thirds
  a whole-tone cluster, so a musical scale is required.
- **`VOICE_SH101_MAJ`** ("SH-101 maj") added — the same synth quantizing to
  **`SCALE_MAJOR`** for bright triads. Major/minor is switchable live by cycling
  between the two twins (no special control). It slots in right after `VOICE_SH101`
  and before `VOICE_DRUMS`; `NUM_VOICES` 14→15, cyclable 10→11, `MULTI_ZONES`
  (drum indices 8–11) unaffected.
- `CHORD_UPPER_LEVEL` tuned 0.6→0.9 (the upper two notes were too quiet relative
  to the root).
- Appended `chords` last in the `Voice` struct so every other preset zero-inits
  to `false` (unchanged). Host test added for `diatonic_triad` (minor/major
  triad shapes, octave wrap, off-scale snap, no-scale fallback) plus a bank
  invariant: every chord voice must quantize to a non-chromatic scale.

### Docs — `docs: sync README + CLAUDE.md with current voices/gesture/constants`
- **README voice table**: added `VOICE_GUITAR` (it had replaced `VOICE_BASS_CLOSED`,
  which was still listed) and `VOICE_SH101`; renumbered Drums to 14 and noted the
  `no_cycle` raw-drums exclusion and where SH101 sits in the bank vs. the cycle.
- **README live-switch gesture**: replaced the old "hold FSR, auto-advances every
  2 s" text with the current three-phase **FSR-hold + tap-the-glass** gesture
  (2 s no-touch enter → tap to advance with live preview → release to save).
- **README Possible Extensions**: dropped "Save the selected voice" and "Pitch
  envelope" (both shipped).
- **README touch params**: updated stale `PRESSURE_MAX_REF` values and removed the
  `PRESSURE_DEAD_ZONE[12]` block (that constant no longer exists — the pressure
  floor is now the scalar `TOUCH_THRESHOLD`); fixed the matching troubleshooting row.
- **CLAUDE.md key-constants table**: corrected `TOUCH_THRESHOLD` (was "39"; it's 10
  and lives in `config.h`), removed the nonexistent `PRESSURE_MIN_REF[12]` row, and
  pointed the touch constants at `config.h` instead of stale `main.cpp` line numbers.

### Added — `feat: SH-101-style mono synth voice`
- New **`VOICE_SH101`** ("SH-101"): a Roland SH-101 homage — saw + unison square
  (pulse) over a square sub-oscillator an octave down (the fat single-VCO stack),
  through the resonant 4-pole ladder. Cutoff sits closed-ish and finger pressure
  sweeps it wide open with high resonance for the squelchy acid "wow"; portamento
  glide + chromatic quantize keep acid lines in tune with bendable slides. Placed
  after the raw drums in the bank so the drum indices (and `MULTI_ZONES`) stay
  put; it's cyclable and shows up after Pad in the FSR cycle. Bumped the bank-size
  asserts in the host test (NUM_VOICES 13→14, cyclable 9→10).

### Added — `feat: electric guitar voice + per-voice amp decay-to-sustain`
- New **`VOICE_GUITAR`** ("Guitar") replacing `VOICE_BASS_CLOSED` in the bank slot
  before Pad: a driven sawtooth **power chord** (root + fixed 5th + octave-down
  body + octave-up jangle), **quantized to a major scale** (tapped chords land in
  key), overdrive grit + resonant mid "quack", a fast pitch-env **pick-attack
  twang** and a little pick-grit noise on the onset.
- New **per-voice amp decay-to-sustain** (`Voice::decay_ms` / `Voice::sustain`):
  after the attack, the gated level falls to `sustain` over `decay_ms`, so a note
  can pluck and ring out instead of holding flat. `decay_ms = 0` (the default for
  every other voice) means **no decay** — identical to before, so nothing else is
  affected. The Guitar uses `decay_ms = 1200`, `sustain = 0` (rings to silence).
  Implemented as an `s_decay` multiplier on the gated amp target in the engine.

### Changed — `feat: FSR-hold + tap-the-glass voice select (replaces timed auto-advance)`
- Reworked the live voice-switch gesture from a slow timed auto-advance into a
  fast, user-paced **hold-FSR / tap-the-glass** cycle:
  - **Enter** by pressing the FSR fully to the mute floor *with no glass touch*
    for 2 s (`VOICE_SELECT_ENTER_MS`). The FSR reads inversely (hard press → vol 0),
    and requiring no touch keeps normal muting from tripping it. LEDs blink the
    current position. (Master volume is forced to a monitor level in select mode
    so previews stay audible despite the press muting.)
  - **Advance** by tapping the **glass** while the FSR is held (rising-edge,
    debounced by `VOICE_SELECT_TAP_GAP_MS`) — steps to the next cyclable voice and
    **wraps** after the last; a single LED blink per advance (the preview IDs the voice).
  - **Keep** by **releasing the FSR** (it climbs back up) → persists and exits
    immediately, blinking the **saved voice's number** to confirm.
  - LEDs blink the **current voice number** on enter and the **saved number** on
    save; single blink per tap-advance in between.
- **Live audio preview**: while cycling, the selected voice plays **live** on the
  glass so you recognise it by ear. A **mono** voice plays through the **real note
  path** (the select branch falls through instead of skipping) driven by your real
  glass pressure/position, so the preview is identical to the actual voice — every
  filter and effect applies (including the **Moog cutoff sweep**; the earlier
  hand-rolled preview omitted it and vibrato/quantize). Press for the filter, slide
  for pitch, lift to release; the Drums kit fires a kick + snare one-shot "one-two".
  Gated by `VOICE_PREVIEW_ENABLED`
  (config.h, default on; false = silent LED-only cycling). Preview drums fire as
  **one-shots** (jump to full amp + immediately ungate) so they decay instead of
  sustaining — a gated preview drum had no finger-lift to release it, which left
  the Drums kit's noise running at 100 % once selected.
- The **saved-voice confirm blink is now non-blocking** (was blocking
  `flash_voice_leds`): it overlays the LED position display from the normal loop,
  so you can play immediately after releasing the FSR instead of waiting it out.
- During the mono preview the **LEDs follow the finger** (the blink overlays only
  while actually animating), and **finger-2 effects** (detune / wavefold / ring-mod)
  apply — both fall out of the preview using the real note path.
- The **idle chase** (5 s no-touch LED pulse) now also exits on an FSR press to the
  floor, so voice-select can be entered straight from idle without first touching
  the glass.
- LED feedback is now driven by a **non-blocking** blinker (`blink_start` /
  `blink_tick`) so fast taps register while it flashes; the old blocking
  `flash_voice_leds` is startup-only.
- Persistence now writes QSPI **once on commit** instead of on every step
  (less flash wear). Removed `VOICE_SWITCH_FIRST_MS` / `_REPEAT_MS`; added
  `VOICE_SELECT_ENTER_MS` / `_TAP_GAP_MS`.

### Changed — `feat: per-voice pitch quantize (Organ only)`
- Pitch quantization is now a per-voice property: added a `quantize` field to the
  `Voice` struct (default `false`). Only `VOICE_ORGAN` enables it, so the Organ
  snaps finger-1 to the nearest chromatic note on touch-down (then bends freely on
  slide) while every other voice plays fully continuous pitch.
- `QUANTIZE_ENABLED` in `config.h` is now a master switch — the touch-down snap
  gates on `QUANTIZE_ENABLED && VOICE.quantize`, so it must also be true for any
  snapping to happen.

### Added — `feat: per-LED brightness levels (reduce LED→audio noise)`
- Per-LED brightness knobs in `config.h` (`LED1_LEVEL`/`LED2_LEVEL`/`LED3_LEVEL`)
  applied to every LED write (position, idle sweep, voice flash). Lowering an
  LED's level cuts its current, which reduces the LED switching noise that
  couples into the audio (worst on battery). LED3 (the 200 Hz software-PWM LED)
  is the main culprit, so it's set to 50%; the analog DAC LEDs stay at 100%

### Fixed — `fix: MPR121 ESI 16ms → 1ms for fast-tap response`
- `REG_CONFIG2` was `0x4C`, whose ESI (electrode sample interval) bits decode to
  16 ms — the sensor only refreshed electrode values ~62×/sec, blurring/dropping
  quick taps (and its comment wrongly claimed "ESI=1ms"). Set to `0x48` (ESI =
  1 ms, CDT/SFI unchanged) so the sensor refreshes ~16× faster; fast tapping and
  rolls now register reliably. Verified stable on the glass (no added jitter)

### Added — `feat: startup voice-blink + FSR volume deadzone`
- On boot (after the grace period + audio start) the LEDs flash N times = the
  boot/restored voice's cycle position — same gesture as a live voice switch, so
  you can see which voice you're on (handy with the persisted voice)
- FSR master volume gains a 10% deadzone (`FSR_DEADZONE`): light pressure holds
  full volume; only past 10% of travel does it ramp down to 0 at the min. The
  deadzone tracks `fsr_max`, so it recalibrates with the idle re-cal

### Added — `test: host unit-test harness for the pure modules (phase 4)`
- New `test/` harness (`test/test_main.cpp` + `test/Makefile`, run with
  `make -C test`) — compiles `dsp.h`, `voice.h`, and `touch` on the host (no
  Daisy, no gtest; plain assertion macros) and runs ~200 checks: oscillator/
  filter/`ms_to_coeff` behaviour, `detect_raw`/`update_tracked`/`centroid`/
  `pressure_pct`, and the voice `cycle_*` / MultiVoice logic
- Surfaced a known quirk: `pressure_pct`'s integer Newton-sqrt converges poorly
  and saturates to 100 before the electrode max; left as-is (mappings were tuned
  around it), the test asserts the envelope (bounds + monotonic), not a curve

### Changed — `refactor: split main.cpp into modules (phase 3 — audio engine unit)`
- Extracted the **audio engine** into `engine.{h,cpp}`: the `g_*` control params,
  the per-sample DSP state, `moog()`, the polyphonic drum voices (`DrumHit`,
  `g_hits[]`, `drum_trigger`, `drum_render`), and the `AudioCallback`
- The `g_*` params are now the explicit control-loop ↔ audio-callback seam
  (`extern` in `engine.h`); `hw`/`led3`/`g_voice_idx`/`g_active_voice` stay in
  `main.cpp` and are `extern`'d to the engine
- `main.cpp` is now just the app: init, persistence, the touch→params control
  loop, and the serial display — **1855 → 768 lines** across the three phases.
  Behavior unchanged, verified on hardware (flash 106756 → 108460 B; the audio
  params crossing the TU boundary cost a little whole-program optimization)

### Changed — `refactor: split main.cpp into modules (phase 2 — mpr121 + touch units)`
- Extracted the **MPR121 bit-bang driver** (`mpr121.{h,cpp}` — I2C, `mpr_init`,
  `read_electrodes`, `capture_baseline`, owns the SDA/SCL `GPIO`) and the **touch
  detection/tracking** (`touch.{h,cpp}` — `detect_raw`, `update_tracked`,
  `pressure_pct`, `centroid_window`, `tracked[]`) into real compiled translation
  units. `touch` is pure logic over electrode deltas → unit-testable off-target
- Makefile compiles the extra units only for `TARGET=src/main` (tools unaffected)
- `main.cpp`: 1327 → ~1120 lines. Behavior unchanged (flash 106720 → 106756 B,
  a few bytes from the multi-unit layout)

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
