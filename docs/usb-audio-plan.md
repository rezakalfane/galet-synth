# USB Audio — Implementation Plan

**Goal:** make the Daisy enumerate as a **class-compliant USB soundcard (audio IN)**
so its output can be recorded live in a DAW / laptop / phone, **while keeping the
VoiceLab serial features working** over the same cable.

**Approach:** a **composite USB device** = *USB Audio Class* (capture/IN) **+** *CDC
ACM* (the existing serialtune link), built on **TinyUSB** (replacing libDaisy's ST
USB-device stack), running from **QSPI** via the Daisy bootloader to escape the
128 KB internal-flash limit.

> Status: design only. No code yet. Each phase below is independently flashable
> and testable, matching the iterate-on-hardware workflow.

---

## 1. Target spec

| Item | Decision | Notes |
|---|---|---|
| Bus | USB **Full-Speed** (12 Mbit/s) | Daisy Seed connector = OTG-FS on PA11/PA12, VBUS PA9 |
| Direction | **IN** (device → host), i.e. a recording **input** | We're a source, not a playback sink |
| Channels / rate | **stereo, 48 kHz** | Matches the engine's native rate → no resampling |
| Sample format | **16-bit** first; 24-bit as a follow-up | 48 smp/frame × 4 B = 192 B/frame (16-bit); ≪ 1023 B FS iso cap |
| Audio class | **UAC2** first (TinyUSB-native), **UAC1** as fallback | see §6 — this is the main open decision |
| Second interface | **CDC ACM** (VoiceLab) | composite, so tuning + recording coexist |
| Execution | **QSPI** (`APP_TYPE=BOOT_QSPI`) | required for flash headroom (§3) |

**Bandwidth sanity check:** stereo/48 k/16-bit = 192 KB/s; 24-bit = 288 KB/s. The
FS isochronous ceiling is ~1023 B/frame × 1000 frame/s ≈ 1 MB/s, so even 24-bit
stereo fits comfortably on one iso IN endpoint.

---

## 2. Why this is non-trivial (the three real problems)

1. **Flash.** The current firmware is at **93.75 % of 128 KB**. A USB stack +
   audio/CDC classes will not fit. → run from **QSPI** (§3).
2. **The USB stack.** libDaisy ships **only ST USBD-CDC** — no audio class. We bring
   in **TinyUSB**, which has proven composite UAC+CDC support, and stop libDaisy
   from initialising its own USB. → §4, §5.
3. **Clock.** The Daisy's audio clock is the codec/SAI, not USB. For a **capture
   (IN)** endpoint the *device* owns the rate, so we use an **asynchronous IN**
   endpoint and vary samples-per-frame (47/48/49) to track the true clock — **no
   feedback endpoint needed** (that's only for playback/OUT). → §7.

---

## 3. Phase 0 — Move to QSPI execution (foundation, no audio yet)

The riskiest infrastructure step; prove it in isolation first.

- Add a `BOOT_QSPI` build path to our `Makefile` (libDaisy already provides
  `STM32H750IB_qspi.lds` and the `APP_TYPE=BOOT_QSPI` plumbing; app lands at
  `0x90040000`).
- Install the Daisy bootloader once: `make program-boot` (then the app is loaded
  with `make program-dfu`, no more `:leave` to `0x08000000`).
- **CPU under QSPI XIP:** the H7 caches make execute-in-place usually fine, but to
  be safe, pin the hot paths — the **audio callback and DSP inner loops** — into
  **ITCM/SRAM** (libDaisy section attributes). Measure CPU before/after.
- Keep `BOOT_NONE` (internal-flash) buildable as a fallback during the transition.

**Acceptance:** the *unchanged* synth + VoiceLab run identically from QSPI; audio
CPU headroom unchanged within ~1–2 %. Update the flash instructions in
`CLAUDE.md` / `README`.

---

## 4. Phase 1 — TinyUSB device, CDC only (swap the stack, no audio yet)

De-risk the libDaisy↔TinyUSB coexistence before adding audio.

- Vendor **TinyUSB** into the tree (e.g. `lib/tinyusb/`), add its `dwc2` device
  sources + our `tusb_config.h` to the Makefile.
- Configure: `CFG_TUSB_MCU = OPT_MCU_STM32H7`, rhport0 =
  `OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED`; route **`OTG_FS_IRQHandler` →
  `tud_int_handler(0)`**.
- **Stop libDaisy owning USB:** do **not** call `hw.StartLog()` / the USBD CDC.
  Re-point **serialtune** TX/RX from `hw.PrintLine` + `SetReceiveCallback` to
  `tud_cdc_write` / `tud_cdc_read`. The line protocol is unchanged; only the
  transport moves. (`print_kv_f` and friends keep their hand-rolled float
  formatting — just write to the CDC FIFO instead of the logger.)
- Call **`tud_task()`** from the main control loop (USB device servicing). The
  control-loop cadence stays the sensor-bound ~150–200 Hz; `tud_task` is cheap.

**Acceptance:** VoiceLab connects over the TinyUSB CDC port and `tune/set/dump`
behave exactly as today; no regression in the synth.

---

## 5. Phase 2 — Add the UAC capture interface (composite)

- Enable `CFG_TUD_AUDIO` alongside `CFG_TUD_CDC`; build the **composite
  descriptor**: CDC (with its IAD) + the audio function (AudioControl +
  AudioStreaming interfaces, alt-0 zero-bandwidth / alt-1 active, one **iso IN**
  endpoint, Type-I PCM, 2 ch, 16-bit, 48 kHz).
- **Tap the audio path:** in `engine.cpp`'s `AudioCallback`, after the stereo
  `out[]` is written, push the same frames into a **lock-free SPSC ring buffer**
  (producer = SAI DMA IRQ; consumer = USB). Buffer in DTCM/SRAM, sized for a few
  ms (e.g. 8–16 frames). Use `volatile` head/tail + a memory barrier.
- **USB consumer:** on each `tud_audio` IN request, drain `N` samples where `N` is
  47/48/49 chosen to keep the ring near half-full (§7).
- The tap must be **zero-cost when no host is streaming** (alt-0 / not enumerated):
  guard the ring writes behind a `g_usb_audio_active` flag.

**Acceptance:** macOS sees a "GaletSynth" stereo input; recording in a DAW
captures the synth cleanly; VoiceLab still works simultaneously over CDC.

---

## 6. Open decision — UAC2 vs UAC1

TinyUSB's audio class is **UAC2-oriented** (its examples are UAC2). Tradeoffs:

- **UAC2 (recommended first target):** native to TinyUSB → least descriptor work;
  class-compliant on **macOS, iOS, Linux, Windows 10+**. Works fine over FS for 2
  channels. Risk: some quirky hosts prefer UAC1.
- **UAC1 (fallback):** broadest/oldest compatibility, but a more **custom
  descriptor** with TinyUSB and slightly more hand-work.

**Plan:** build **UAC2** first (fastest path to a working input on mac/iOS). If the
**TP-7 / TX-6** (or Android) refuse it in host mode, add a **UAC1 descriptor
variant** behind a build flag. The audio data path (ring buffer, sample sizing) is
identical either way — only the descriptors differ.

---

## 7. Clock / sample-rate strategy (async IN)

- Nominal 48 samples per 1 ms FS frame. The codec clock drifts vs USB SOF, so:
  - track the **ring-buffer fill level**;
  - send **48** normally, **49** when the ring is filling (codec slightly fast),
    **47** when draining (codec slightly slow).
- This is standard **adaptive/asynchronous IN** behaviour; the host's UAC driver
  absorbs the ±1-sample jitter. No explicit feedback endpoint (those are for OUT).
- Match the DAW project to **48 kHz** to keep everything aligned.

**Latency:** 1 ms frame + a few ring frames ≈ **3–5 ms** device-side; DAW adds its
own buffer. Fine for recording; live-monitoring round-trip is the usual DAW story.

---

## 8. Host compatibility matrix (to verify on hardware)

| Host | Expectation |
|---|---|
| **macOS (laptop)** | ✅ class-compliant UAC2/UAC1, no driver |
| **iOS (phone)** | ✅ records class-compliant USB audio (Camera-Kit/USB-C) |
| **Android** | ⚠️ usually UAC1, device-dependent |
| **TE TP-7 / TX-6** | ❓ depends on whether they act as a USB-audio **host**; **must be tested**. UAC1 may be the better bet here |

The TE devices are the real unknown — they're USB-audio *devices* themselves, and
host-mode ingest of another UAC source isn't guaranteed by spec. Test early; the
UAC1 fallback (§6) exists for exactly this.

---

## 9. Codebase touch-points

| Area | Change |
|---|---|
| `Makefile` | `BOOT_QSPI` target + flags; add TinyUSB sources + `tusb_config.h`; ITCM placement |
| `lib/tinyusb/` | vendored TinyUSB (dwc2 FS device) |
| `usb_descriptors.c/.h` (new) | composite CDC + UAC descriptors + strings |
| `usb_audio.{h,cpp}` (new) | ring buffer, `tud_audio` callbacks, sample-rate tracker, `tud_task` hook |
| `engine.cpp` `AudioCallback` | tap stereo `out[]` → ring buffer (guarded by `g_usb_audio_active`) |
| `serialtune.cpp` | TX/RX moved from libDaisy logger/`SetReceiveCallback` → `tud_cdc_*`; protocol unchanged |
| `main.cpp` | don't `StartLog()`; call `tud_task()` in the control loop; `OTG_FS_IRQHandler` → `tud_int_handler(0)` |
| `CLAUDE.md` / `README` | new QSPI build + bootloader flash instructions; the serial monitor is now the TinyUSB CDC port |

---

## 10. Risks & mitigations

- **USB IRQ / clock ownership conflict** (libDaisy vs TinyUSB) → ensure libDaisy
  never inits OTG-FS as a device; we own `OTG_FS_IRQHandler`. Verify the USB clock
  (48 MHz) is configured (libDaisy's `System` init already enables it).
- **QSPI XIP CPU cost** → pin audio/DSP hot paths in ITCM; profile.
- **Ring underrun/overrun → clicks** → size the buffer for a few ms; tune the
  47/48/49 thresholds; assert on under/overflow during bring-up.
- **TE host-mode incompatibility** → UAC1 fallback variant.
- **Flash workflow change** breaks the muscle-memory `dfu` command → document
  clearly; keep a `BOOT_NONE` fallback build for non-audio work.
- **VoiceLab port busy** semantics may differ slightly with TinyUSB CDC → verify
  reconnect/"resource busy" handling in `galetsynth/link.py`.

---

## 11. Phased delivery (each independently testable)

1. **Phase 0 — QSPI boot.** Unchanged firmware runs from QSPI. *(infra)*
2. **Phase 1 — TinyUSB CDC.** VoiceLab works over TinyUSB; libDaisy USB retired.
3. **Phase 2 — UAC2 IN.** macOS records the synth; CDC coexists.
4. **Phase 3 — Hardening.** Buffer/latency tuning; 24-bit option; clock tracker.
5. **Phase 4 — Compatibility.** Test iOS / Android / TP-7 / TX-6; add UAC1 variant
   if needed.

Branch per phase (`feature/usbaudio-qspi`, `-tinyusb-cdc`, `-uac`, …) →
merge → CHANGELOG, per the project convention.

---

## 12. Decisions captured

- **Stack:** TinyUSB (composite UAC + CDC).
- **Keep VoiceLab:** yes — composite device, CDC interface retained; you can tune
  and record over one cable.
- **Class:** UAC2 first, UAC1 fallback for quirky hosts (esp. the TE devices).
- **Open before coding:** confirm we're OK installing the Daisy bootloader (changes
  the flash procedure) and vendoring TinyUSB into the repo.
