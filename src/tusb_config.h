// TinyUSB configuration for GaletSynth — STM32H750, USB Full-Speed device.
// Phase 1: CDC only (the VoiceLab serial link). Audio (UAC) is added in Phase 2.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ── MCU / OS / port ───────────────────────────────────────────────────────────
#define CFG_TUSB_MCU            OPT_MCU_STM32H7
#define CFG_TUSB_OS             OPT_OS_NONE
// The Daisy connector (PA11/PA12) is the FS controller (USB2_OTG_FS) → dwc2 port 0.
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define BOARD_TUD_RHPORT        0
#define CFG_TUSB_DEBUG          0

// DMA-able, 4-byte-aligned USB buffers (default section is fine on the H7).
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN      __attribute__((aligned(4)))

// ── Device stack ────────────────────────────────────────────────────────────
#define CFG_TUD_ENABLED         1
#define CFG_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE  64

// ── Classes ───────────────────────────────────────────────────────────────────
#define CFG_TUD_CDC             1
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_AUDIO           0   // → 1 in Phase 2
#define CFG_TUD_VENDOR          0

// CDC FIFO sizes. The line protocol is small; 256 B each is plenty and matches
// the old libDaisy CDC behaviour.
#define CFG_TUD_CDC_RX_BUFSIZE  256
#define CFG_TUD_CDC_TX_BUFSIZE  256
#define CFG_TUD_CDC_EP_BUFSIZE  64

#ifdef __cplusplus
}
#endif
