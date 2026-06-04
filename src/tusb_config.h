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
#define CFG_TUD_AUDIO           1   // Phase 2: UAC2 stereo capture (composite with CDC)
#define CFG_TUD_VENDOR          0

// CDC FIFO sizes. The line protocol is small; 256 B each is plenty and matches
// the old libDaisy CDC behaviour.
#define CFG_TUD_CDC_RX_BUFSIZE  256
#define CFG_TUD_CDC_TX_BUFSIZE  256
#define CFG_TUD_CDC_EP_BUFSIZE  64

// ── Audio (UAC2) — stereo, 16-bit, 48 kHz capture (device → host) ─────────────
// A stereo mic. No canned 2-ch mic descriptor exists, so TWO_CH_DESC_LEN mirrors
// the 1-ch one with the two-channel feature unit (defined here so audio_device.c
// can size buffers; the matching descriptor body lives in usb_descriptors.c).
#define TUD_AUDIO_MIC_TWO_CH_DESC_LEN (TUD_AUDIO_DESC_IAD_LEN \
    + TUD_AUDIO_DESC_STD_AC_LEN + TUD_AUDIO_DESC_CS_AC_LEN \
    + TUD_AUDIO_DESC_CLK_SRC_LEN + TUD_AUDIO_DESC_INPUT_TERM_LEN \
    + TUD_AUDIO_DESC_OUTPUT_TERM_LEN + TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN \
    + TUD_AUDIO_DESC_STD_AS_INT_LEN + TUD_AUDIO_DESC_STD_AS_INT_LEN \
    + TUD_AUDIO_DESC_CS_AS_INT_LEN + TUD_AUDIO_DESC_TYPE_I_FORMAT_LEN \
    + TUD_AUDIO_DESC_STD_AS_ISO_EP_LEN + TUD_AUDIO_DESC_CS_AS_ISO_EP_LEN)

#define CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE             48000
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN                TUD_AUDIO_MIC_TWO_CH_DESC_LEN
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT                1
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ             64
#define CFG_TUD_AUDIO_ENABLE_EP_IN                   1
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX   2
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX           2
#define CFG_TUD_AUDIO_EP_SZ_IN                       TUD_AUDIO_EP_SIZE(CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX            CFG_TUD_AUDIO_EP_SZ_IN
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ         (8 * CFG_TUD_AUDIO_EP_SZ_IN)

#ifdef __cplusplus
}
#endif
