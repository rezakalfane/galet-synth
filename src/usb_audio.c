// UAC2 stereo capture for GaletSynth — see usb_audio.h. Entity IDs (match the
// descriptor in usb_descriptors.c): input terminal 1, feature unit 2, output
// terminal 3, clock source 4. Async IN endpoint: the device owns the clock and
// varies samples/frame (47/48/49) to track it.
#include "tusb.h"
#include "usb_audio.h"

#define N_CH CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX   // 2

// ── Control state ─────────────────────────────────────────────────────────────
static uint32_t s_samp_freq = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
static uint8_t  s_clk_valid = 1;
static int8_t   s_mute[N_CH + 1];                 // [0]=master, [1..N]=channels
static int16_t  s_volume[N_CH + 1];
static audio_control_range_4_n_t(1) s_freq_rng;

static volatile uint8_t s_streaming = 0;
static volatile uint8_t s_primed    = 0;   // ring cushion built before draining

// ── Capture ring — stereo int16 interleaved, free-running indices (int16 units).
//    Producer: audio ISR (usb_audio_push). Consumer: USB (tx pre-load). ─────────
#define RING_SZ 4096                              // int16 elems = 2048 frames ≈ 42 ms
static int16_t           s_ring[RING_SZ];
static volatile uint32_t s_wr = 0, s_rd = 0;

// Diagnostic counters (read via the `astat` serial command).
volatile uint32_t g_aud_over = 0, g_aud_under = 0, g_aud_calls = 0, g_aud_fill = 0;

void usb_audio_push(int16_t l, int16_t r)
{
    if(!s_streaming) return;
    if((uint32_t)(s_wr - s_rd) >= (RING_SZ - 2)) { g_aud_over++; return; }  // full → drop
    s_ring[s_wr & (RING_SZ - 1)]       = l;
    s_ring[(s_wr + 1) & (RING_SZ - 1)] = r;
    s_wr += 2;
}
int usb_audio_is_streaming(void) { return s_streaming; }

// ── Streaming start/stop (alt setting of the AS interface) ────────────────────
bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t alt = TU_U16_LOW(p_request->wValue);
    if(alt != 0) { s_wr = s_rd = 0; s_primed = 0; s_streaming = 1; }   // start fresh, re-prime
    else         { s_streaming = 0; }
    return true;
}
bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport; (void)p_request;
    s_streaming = 0;
    return true;
}

// ── TX feed: called per IN request (~1 ms). Send ~48 stereo frames, nudged to
//    47/49 to keep the ring near half-full (async clock tracking). ─────────────
// Async IN clock tracking: hold the ring near TARGET frames of latency by sending
// 48±1 frames/packet. The packet-size variation conveys the device's true rate to
// the host, which locks its resampler to it — no feedback endpoint needed.
#define AUD_TARGET_FRAMES 256       // ~5.3 ms latency cushion
#define AUD_HYST          24

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    (void)rhport; (void)itf; (void)ep_in; (void)cur_alt_setting;
    static int16_t tmp[49 * N_CH];
    uint32_t frames = (uint32_t)(s_wr - s_rd) / 2;         // available stereo frames
    g_aud_calls++; g_aud_fill = frames;

    // Warm-up: don't drain until the cushion has built, so we never start starved.
    if(!s_primed) {
        if(frames >= AUD_TARGET_FRAMES) s_primed = 1;
        for(uint32_t i = 0; i < 48 * N_CH; i++) tmp[i] = 0; // send silence meanwhile
        tud_audio_write((uint8_t *)tmp, (uint16_t)(48 * N_CH * 2));
        return true;
    }

    uint32_t send = 48;
    if(frames > AUD_TARGET_FRAMES + AUD_HYST)      send = 49;   // too full → drain faster
    else if(frames < AUD_TARGET_FRAMES - AUD_HYST) send = 47;   // too empty → ease off

    if(frames < send) g_aud_under++;                           // count starved packets
    uint32_t real = (frames < send) ? frames : send;           // underrun guard
    uint32_t i = 0;
    for(; i < real * N_CH; i++) tmp[i] = s_ring[(s_rd + i) & (RING_SZ - 1)];
    s_rd += real * N_CH;
    for(; i < send * N_CH; i++) tmp[i] = 0;                     // pad with silence if short
    tud_audio_write((uint8_t *)tmp, (uint16_t)(send * N_CH * 2));
    return true;
}

// ── Entity control: SET (feature unit mute/volume) ────────────────────────────
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void)rhport;
    uint8_t ch       = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel  = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);
    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);
    if(entityID == 2 && ch <= N_CH) {
        if(ctrlSel == AUDIO_FU_CTRL_MUTE)   { s_mute[ch]   = ((audio_control_cur_1_t *)pBuff)->bCur; return true; }
        if(ctrlSel == AUDIO_FU_CTRL_VOLUME) { s_volume[ch] = (int16_t)((audio_control_cur_2_t *)pBuff)->bCur; return true; }
    }
    return false;
}

// ── Entity control: GET (input-terminal connector, FU mute/volume, clock) ─────
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uint8_t ch       = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel  = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    if(entityID == 1) {                                    // input terminal
        if(ctrlSel == AUDIO_TE_CTRL_CONNECTOR) {
            audio_desc_channel_cluster_t ret = { 0 };
            ret.bNrChannels    = N_CH;
            ret.bmChannelConfig = (audio_channel_config_t)0;
            ret.iChannelNames  = 0;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &ret, sizeof(ret));
        }
        return false;
    }
    if(entityID == 2) {                                    // feature unit
        if(ctrlSel == AUDIO_FU_CTRL_MUTE)
            return tud_control_xfer(rhport, p_request, &s_mute[ch], 1);
        if(ctrlSel == AUDIO_FU_CTRL_VOLUME) {
            if(p_request->bRequest == AUDIO_CS_REQ_CUR)
                return tud_control_xfer(rhport, p_request, &s_volume[ch], sizeof(s_volume[ch]));
            if(p_request->bRequest == AUDIO_CS_REQ_RANGE) {
                audio_control_range_2_n_t(1) ret;
                ret.wNumSubRanges = 1;
                ret.subrange[0].bMin = -90 * 256; ret.subrange[0].bMax = 0; ret.subrange[0].bRes = 256;
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &ret, sizeof(ret));
            }
        }
        return false;
    }
    if(entityID == 4) {                                    // clock source
        if(ctrlSel == AUDIO_CS_CTRL_SAM_FREQ) {
            if(p_request->bRequest == AUDIO_CS_REQ_CUR)
                return tud_control_xfer(rhport, p_request, &s_samp_freq, sizeof(s_samp_freq));
            if(p_request->bRequest == AUDIO_CS_REQ_RANGE)
                return tud_control_xfer(rhport, p_request, &s_freq_rng, sizeof(s_freq_rng));
        }
        if(ctrlSel == AUDIO_CS_CTRL_CLK_VALID)
            return tud_control_xfer(rhport, p_request, &s_clk_valid, sizeof(s_clk_valid));
        return false;
    }
    return false;
}

// One-time init of the control state. Called from usb_fs_init().
void usb_audio_init(void)
{
    s_samp_freq = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    s_clk_valid = 1;
    s_freq_rng.wNumSubRanges      = 1;
    s_freq_rng.subrange[0].bMin   = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    s_freq_rng.subrange[0].bMax   = CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE;
    s_freq_rng.subrange[0].bRes   = 0;
}
