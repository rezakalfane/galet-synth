// USB descriptors for GaletSynth — Phase 2: a composite device with a CDC ACM
// interface (the VoiceLab serial link) + a UAC2 stereo capture interface (the
// soundcard input). Interfaces: CDC = 0,1 ; Audio Control = 2 ; Audio Streaming = 3.
#include "tusb.h"
#include <string.h>

#define USB_VID 0x0483          // STMicroelectronics
#define USB_PID 0x5741          // bumped from the CDC-only 0x5740 (new descriptor set)
#define USB_BCD 0x0200

// ── Device descriptor (IAD/misc so the composite enumerates cleanly) ──────────
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};
uint8_t const *tud_descriptor_device_cb(void) { return (uint8_t const *)&desc_device; }

// ── Stereo (two-channel) UAC2 mic descriptor — adapted from the 1-ch macro with
//    a two-channel feature unit + 2 physical/logical channels. Entity IDs: input
//    terminal 1, feature unit 2, output terminal 3, clock source 4. ────────────
#define TUD_AUDIO_MIC_TWO_CH_DESCRIPTOR(_itfnum, _stridx, _nBytesPerSample, _nBitsUsedPerSample, _epin, _epsize) \
    TUD_AUDIO_DESC_IAD(_itfnum, 0x02, 0x00), \
    TUD_AUDIO_DESC_STD_AC(_itfnum, 0x00, _stridx), \
    TUD_AUDIO_DESC_CS_AC(0x0200, AUDIO_FUNC_MICROPHONE, TUD_AUDIO_DESC_CLK_SRC_LEN+TUD_AUDIO_DESC_INPUT_TERM_LEN+TUD_AUDIO_DESC_OUTPUT_TERM_LEN+TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN, AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS), \
    TUD_AUDIO_DESC_CLK_SRC(0x04, AUDIO_CLOCK_SOURCE_ATT_INT_FIX_CLK, (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), 0x01, 0x00), \
    TUD_AUDIO_DESC_INPUT_TERM(0x01, AUDIO_TERM_TYPE_IN_GENERIC_MIC, 0x03, 0x04, /*_nchannelslogical*/ 0x02, /*_channelcfg*/ AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, 0x00, AUDIO_CTRL_R << AUDIO_IN_TERM_CTRL_CONNECTOR_POS, 0x00), \
    TUD_AUDIO_DESC_OUTPUT_TERM(0x03, AUDIO_TERM_TYPE_USB_STREAMING, 0x01, 0x02, 0x04, 0x0000, 0x00), \
    TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL(0x02, 0x01, \
        /*master*/ (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), \
        /*ch1*/    (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), \
        /*ch2*/    (AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_MUTE_POS | AUDIO_CTRL_RW << AUDIO_FEATURE_UNIT_CTRL_VOLUME_POS), 0x00), \
    TUD_AUDIO_DESC_STD_AS_INT((uint8_t)((_itfnum)+1), 0x00, 0x00, 0x00), \
    TUD_AUDIO_DESC_STD_AS_INT((uint8_t)((_itfnum)+1), 0x01, 0x01, 0x00), \
    TUD_AUDIO_DESC_CS_AS_INT(0x03, AUDIO_CTRL_NONE, AUDIO_FORMAT_TYPE_I, AUDIO_DATA_FORMAT_TYPE_I_PCM, /*_nchannelsphysical*/ 0x02, AUDIO_CHANNEL_CONFIG_NON_PREDEFINED, 0x00), \
    TUD_AUDIO_DESC_TYPE_I_FORMAT(_nBytesPerSample, _nBitsUsedPerSample), \
    TUD_AUDIO_DESC_STD_AS_ISO_EP(_epin, (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_DATA), _epsize, 0x01), \
    TUD_AUDIO_DESC_CS_AS_ISO_EP(AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, AUDIO_CTRL_NONE, AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 0x0000)

// ── Configuration descriptor (composite) ──────────────────────────────────────
enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_AUDIO_CONTROL, ITF_NUM_AUDIO_STREAMING, ITF_NUM_TOTAL };

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82
#define EPNUM_AUDIO_IN  0x83

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + CFG_TUD_AUDIO_FUNC_1_DESC_LEN)

static uint8_t const desc_fs_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    // CDC: comm interface, string idx 4, notify EP, notify size, data OUT/IN, EP size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    // UAC2 stereo capture: control interface, string idx 5, 16-bit/16-bit, IN EP.
    TUD_AUDIO_MIC_TWO_CH_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, 5,
        CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX * 8,
        EPNUM_AUDIO_IN, CFG_TUD_AUDIO_EP_SZ_IN),
};
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_fs_configuration;
}

// ── String descriptors ────────────────────────────────────────────────────────
static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},   // 0: language = English (0x0409)
    "GaletSynth",                 // 1: Manufacturer
    "GaletSynth Voice Tuner",     // 2: Product
    "GS-0001",                    // 3: Serial number
    "GaletSynth CDC",             // 4: CDC interface
    "GaletSynth Audio",           // 5: Audio interface
};
static uint16_t _desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    uint8_t chr_count;
    if(index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if(index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        const char *str = string_desc_arr[index];
        chr_count = (uint8_t)strlen(str);
        if(chr_count > 31) chr_count = 31;
        for(uint8_t i = 0; i < chr_count; i++) _desc_str[1 + i] = str[i];
    }
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
