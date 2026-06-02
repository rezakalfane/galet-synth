/**
 * mpr121_synth.cpp — Glass Touch Moog Synth
 * Daisy Seed — Bit-Bang I2C  SDA=D12  SCL=D11
 *
 * FINGER 1:
 *   Position  → Pitch (quantized on touch-down, continuous slide after)
 *   Pressure  → Filter cutoff (light=dark/closed, hard=bright/open)
 *               + Vibrato depth (press harder for more wobble)
 *
 * FINGER 2:
 *   Position  → Oscillator detune (spread between osc1 and osc2)
 *   Pressure  → Wavefold distortion (gentle saturation to complex fold)
 *               + Ring mod amount (extreme pressure adds metallic edge)
 *
 * EXPRESSION:
 *   - Note quantize on initial touch, then free pitch bend on slide
 *   - Pressure-controlled vibrato (LFO depth from finger 1 pressure)
 *   - Portamento / glide between quantized notes
 *   - Sub oscillator one octave below (adds weight)
 *   - Soft-clip waveshaper before output
 *   - Amplitude envelope: fast attack, pressure-controlled release
 */

#include "daisy_seed.h"
#include "util/PersistentStorage.h"
#include <math.h>
#include <string.h>

using namespace daisy;
using namespace daisy::seed;

#include "config.h"   // tuning constants
#include "dsp.h"      // oscillators, filters, math (Waveform)
#include "voice.h"    // Voice struct, presets, bank, cycling
#include "mpr121.h"   // capacitive sensor driver (bit-bang I2C)
#include "touch.h"    // finger detection / tracking
#include "engine.h"   // audio engine: synth state, drum voices, AudioCallback
#include "serialtune.h" // live voice tuning over USB serial

// Persisted settings (saved to QSPI flash, survives power-off).
struct PersistSettings {
    int32_t voice_idx;
    bool operator!=(const PersistSettings& o) const { return o.voice_idx != voice_idx; }
};


// Selected voice index — set by the FSR-hold gesture, shown in the header.
// Change the initial value to pick the boot voice.
volatile int g_voice_idx = 2;                  // 2 = OpenBass (extern in engine.h)

// Active voice index — what the audio engine actually plays. Normally tracks
// g_voice_idx; in MultiVoice mode the touch loop routes it per tap to a drum.
volatile int g_active_voice = 2;               // matches the boot voice (extern in engine.h)




// ── Globals ───────────────────────────────────────────────────────────────────
DaisySeed hw;
GPIO      led3;  // software-PWM LED on A0 (ADC0 / D15 / pin 22)




// ── Note quantizer ────────────────────────────────────────────────────────────
// Returns the nearest MIDI note in the given scale for a raw float MIDI pitch.
// The scale is the voice's (Voice::scale / scale_len); a null/empty scale leaves
// the pitch continuous.
float quantize_midi(float midi, const int8_t* scale, int scale_len)
{
    if(!scale || scale_len <= 0) return midi;
    int root = (int)midi;
    int pc   = root % 12;
    int oct  = root / 12;
    // find nearest scale degree
    int best_deg=0, best_dist=127;
    for(int i=0;i<scale_len;i++){
        int d=scale[i]-pc; if(d<0)d=-d;
        int d2=12-d; if(d2<d)d=d2;
        if(d<best_dist){best_dist=d;best_deg=scale[i];}
    }
    float q=(float)(oct*12+best_deg);
    // fractional semitone from the quantized note (for slide expression)
    float frac=midi-(float)root;
    return q+frac;
}


// ── Bar helpers ───────────────────────────────────────────────────────────────
void make_bar(char* out,int32_t val,int32_t max,int w)
{
    int n=(max>0&&val>0)?(int)((val*w+max/2)/max):0;
    if(n>w)n=w;
    out[0]='[';
    for(int i=0;i<w;i++)out[1+i]=(i<n)?'#':' ';
    out[1+w]=']';out[2+w]='\0';
}

void make_pos_bar(char* out,int w)
{
    out[0]='|';
    for(int i=0;i<w;i++)out[1+i]='-';
    out[1+w]='|';out[2+w]='\0';
    for(int slot=0;slot<MAX_FINGERS;slot++){
        if(!tracked[slot].alive)continue;
        int p=(int)(((int64_t)tracked[slot].pos*(w-1)+500)/1000);
        if(p<0)p=0;if(p>=w)p=w-1;
        out[1+p]=(char)('1'+slot);
    }
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main()
{
    hw.Init();
    hw.SetAudioBlockSize(4);

    // ── Persisted voice (QSPI flash) ──────────────────────────────────────────
    // Restore the last selected voice across power cycles. First boot writes the
    // current g_voice_idx as the factory default.
    PersistentStorage<PersistSettings> storage(hw.qspi);
    storage.Init(PersistSettings{ g_voice_idx });
    {
        int32_t v = storage.GetSettings().voice_idx;
        if(v >= 0 && v < NUM_VOICES) g_voice_idx = v;
    }
    g_active_voice = (g_voice_idx == MULTI_IDX) ? MULTI_ZONES[0] : g_voice_idx;

    // Power-on grace period so the lid can be closed before baseline capture.
    System::Delay(5000);

    // ── I2C pins + MPR121 init BEFORE audio starts ────────────────────────────
    // The audio ISR firing during bit-bang I2C corrupts I2C timing.
    // Do all sensor work first, then start the audio engine.
    mpr_idle_pins();

    uint16_t baseline[N_CH];
    if(mpr_init())
    {
        System::Delay(300);
        capture_baseline(baseline);
    }
    // If MPR121 fails we still start audio so the problem is audible (silence)

    // ── DAC LEDs (pin 30 = DAC1, pin 31 = DAC2) + PWM LED (A0 = pin 22) ──────
    DacHandle::Config dac_cfg;
    dac_cfg.bitdepth   = DacHandle::BitDepth::BITS_12;
    dac_cfg.buff_state = DacHandle::BufferState::ENABLED;
    dac_cfg.mode       = DacHandle::Mode::POLLING;
    dac_cfg.chn        = DacHandle::Channel::BOTH;
    hw.dac.Init(dac_cfg);
    hw.dac.WriteValue(DacHandle::Channel::BOTH, 0);

    led3.Init(seed::A0, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
    led3.Write(false);

    // ── FSR (A1 / ADC1) ───────────────────────────────────────────────────────
    // Wired so unpressed reads ~max (3.3V). Press pulls ADC down. Fixed
    // thresholds: raw ≤ FSR_MIN → 0%, raw ≥ FSR_MAX → 100%, linear between.
    AdcChannelConfig fsr_cfg;
    fsr_cfg.InitSingle(seed::A1);
    hw.adc.Init(&fsr_cfg, 1);
    hw.adc.Start();

    constexpr uint16_t FSR_MIN       = 8000;
    constexpr float    FSR_DEADZONE  = 0.10f;  // top 10% of travel = full volume (no change)
    uint16_t           fsr_max       = 55000;
    // Volume stays 100% until pressure passes the deadzone (raw drops below
    // fsr_dead), then ramps linearly to 0% at FSR_MIN.
    uint16_t           fsr_dead       = (uint16_t)(fsr_max - FSR_DEADZONE * (fsr_max - FSR_MIN));
    float              fsr_active_inv = 1.0f / (float)(fsr_dead - FSR_MIN);

    auto commit_fsr_avg = [&](uint32_t acc, int n) {
        if(n == 0) return;
        uint16_t avg = (uint16_t)(acc / n);
        // Sanity floor — without enough headroom over FSR_MIN the volume curve
        // becomes hyper-sensitive to noise.
        if(avg < FSR_MIN + 500) avg = FSR_MIN + 500;
        fsr_max        = avg;
        fsr_dead       = (uint16_t)(fsr_max - FSR_DEADZONE * (fsr_max - FSR_MIN));
        fsr_active_inv = 1.0f / (float)(fsr_dead - FSR_MIN);
    };

    // Startup calibration: 2 s silent FSR average → fsr_max.
    auto calibrate_fsr_silent = [&]() {
        uint32_t start = System::GetNow();
        uint32_t acc = 0; int n = 0;
        while(System::GetNow() - start < 2000) {
            acc += hw.adc.Get(0);
            n++;
            System::Delay(10);
        }
        commit_fsr_avg(acc, n);
    };

    // Idle behaviour: sweep a virtual position 0→1000→0 through the same
    // LED-tent math as the pitch mapping (LED2=pos 0, LED1=middle, LED3=pos 1),
    // recalibrate fsr_max during the first 2 s, then keep sweeping until any
    // electrode crosses TOUCH_THRESHOLD.
    auto idle_chase_and_calibrate = [&]() {
        uint32_t start = System::GetNow();
        uint32_t acc = 0; int n = 0;
        bool     calibrated = false;
        constexpr uint32_t SWEEP_PERIOD_MS = 12000;  // one 0→1→0 cycle
        constexpr float    TAU             = 6.2831853f;
        uint16_t scan[N_CH];

        while(true) {
            uint32_t elapsed = System::GetNow() - start;

            // Sinusoidal position 0→1→0 over SWEEP_PERIOD_MS — eases at the
            // endpoints so the reversal is gentle instead of a triangle kink.
            float phase = (float)(elapsed % SWEEP_PERIOD_MS) / (float)SWEEP_PERIOD_MS;
            float p01   = 0.5f * (1.0f - cosf(phase * TAU));

            // Same tent + gamma-0.25 math as the pitch LED mapping.
            float l1_raw = 1.0f - 2.0f * p01;                 if(l1_raw < 0) l1_raw = 0;
            float l3_raw = 2.0f * p01 - 1.0f;                 if(l3_raw < 0) l3_raw = 0;
            float l2_raw = 1.0f - 2.0f * fabsf(p01 - 0.5f);   if(l2_raw < 0) l2_raw = 0;
            float led1   = sqrtf(sqrtf(l2_raw));              // middle (DAC1)
            float led2   = sqrtf(sqrtf(l1_raw));              // pos 0  (DAC2)
            float led3   = sqrtf(sqrtf(l3_raw)) * LED3_INTENSITY; // pos 1 (PWM A0)
            hw.dac.WriteValue(DacHandle::Channel::ONE, (uint16_t)(led1 * LED1_MAX));
            hw.dac.WriteValue(DacHandle::Channel::TWO, (uint16_t)(led2 * LED2_MAX));
            g_led3_duty = led3;

            // FSR: drive calibration, and also break out if it's pressed to the
            // floor — so the voice-select gesture (hold FSR) works straight from
            // idle, without having to touch the glass first to exit the chase.
            uint16_t fsr_now = hw.adc.Get(0);
            if(fsr_now <= FSR_MIN) break;
            if(!calibrated) {
                acc += fsr_now;
                n++;
                if(elapsed >= 2000) {
                    commit_fsr_avg(acc, n);
                    calibrated = true;
                }
            }

            // Exit on any touch.
            read_electrodes(scan);
            int32_t max_d = 0;
            for(int i = 0; i < N_CH; i++) {
                int32_t d = (int32_t)baseline[i] - (int32_t)scan[i];
                if(d > max_d) max_d = d;
            }
            if(max_d >= TOUCH_THRESHOLD) break;

            System::Delay(10);
        }

        // Clean handoff back to the main loop's LED writer.
        hw.dac.WriteValue(DacHandle::Channel::ONE, 0);
        hw.dac.WriteValue(DacHandle::Channel::TWO, 0);
        g_led3_duty = 0.0f;
    };

    calibrate_fsr_silent();

    // Boot-voice snapshot for the startup banner / initial targets. The main
    // loop rebinds its own `VOICE` each frame so it always reflects the current
    // voice index (which the FSR gesture can change live).
    const Voice& VOICE = VOICES[g_voice_idx];

    // ── Start audio ───────────────────────────────────────────────────────────
    // Set initial targets before callback fires (voice-relative starting pitch)
    g_target_freq   = VOICE.freq_low * 2.0f;
    g_target_cutoff = 100.0f;
    g_amp_target    = 0.0f;
    hw.StartAudio(AudioCallback);

    // Boot beep — 200ms tone so you can confirm headphone audio is working
    g_amp_target = 0.4f;
    System::Delay(200);
    g_amp_target = 0.0f;
    System::Delay(50);

    // ── USB serial (blocks until host connects) ───────────────────────────────
    hw.StartLog(false); // false = don't wait for serial monitor
    serial_tune_init(); // add the USB receive path for the live tuning protocol

    hw.PrintLine("Glass Moog Synth");
    hw.PrintLine("================");
    hw.PrintLine("Voice %d/%d: %s  (%d-%dHz)",
                 cycle_pos(g_voice_idx), cycle_total(),
                 VOICE.name, (int)VOICE.freq_low, (int)VOICE.freq_high);
    hw.PrintLine("F1 pos=pitch  F1 prs=cutoff+vibrato");
    if(VOICE.osc2_detune)
        hw.PrintLine("F2 pos=detune F2 prs=wavefold+ringmod");
    else
        hw.PrintLine("F2 prs=wavefold+ringmod (osc2=fixed)");
    hw.PrintLine("Press FSR fully (no touch) 2s -> select; tap to cycle; release to keep");
    hw.PrintLine("MPR121 OK — baseline captured");
    hw.PrintLine("Ready.\n");

    uint16_t data[N_CH];
    int32_t  delta[N_CH];
    char     bar[32], pos_bar[52];
    Finger   raw[MAX_FINGERS];

    uint32_t idle_since      = System::GetNow();
    uint32_t last_touch_ms   = System::GetNow();
    bool     f1_was_on       = false;
    uint32_t f1_last_seen_ms = 0;
    static constexpr uint32_t F1_REQUANTIZE_MS = 200; // re-quantize only after this long off
    float    f1_midi_base    = 60.0f; // last quantized note
    bool     f1_sliding      = false; // true after first touch settles

    // Control loop runs as fast as the sensor allows (~150-200 Hz) for tight
    // timing; the serial display is throttled to a readable rate independently.
    static constexpr uint32_t PRINT_INTERVAL_MS = 120;  // ~8 display refreshes/sec
    static constexpr uint32_t CONTROL_DELAY_MS  = 3;    // small cap so we don't hammer I2C
    uint32_t last_print_ms = System::GetNow();

    // Velocity: capture peak pressure over a short window at each tap onset, then
    // hold it for the note. Scales loudness by the voice's vel_sens.
    static constexpr uint32_t VEL_WINDOW_MS = 0;   // latch velocity on the onset frame — snappiest
    float    vel         = 1.0f;   // latched hit velocity 0..1
    float    vel_peak    = 0.0f;   // running peak pressure during the window
    uint32_t vel_start   = 0;      // window start
    bool     vel_active  = false;  // capturing

    // Smoothed LED brightness state (0..1), slewed toward per-frame targets.
    // LED1 = DAC1 (pos 0 end), LED2 = DAC2 (middle), LED3 = PWM A0 (pos 1000 end).
    float s_led1 = 0.0f, s_led2 = 0.0f, s_led3 = 0.0f;

    // ── FSR voice-select gesture state (tap-to-cycle) ─────────────────────────
    int      sel_mode      = 0;      // 0 = normal play, 1 = voice-select mode
    bool     full_state    = false;  // schmitt-latched "FSR pressed to the floor"
    uint32_t full_since    = 0;      // when full_state went true (enter timer; 0 = not full)
    bool     prev_held     = false;  // bridged glass-touch state last loop (tap edge detect)
    uint32_t touch_last_ms = 0;      // last loop the glass read as touched (lift bridge)
    uint32_t last_tap_ms   = 0;      // last glass-tap advance (debounce)
    uint32_t drum2_at      = 0;      // scheduled 2nd drum hit (snare) for the kit preview
    // Non-blocking LED blinker: blink_start(n) queues n flashes, blink_tick()
    // advances them between loops so glass taps still register (the old blocking
    // flash stalled the loop up to ~2 s). Voice-select blinks the position on
    // enter and on save (which voice), and a single blink per tap-advance (the
    // audio preview IDs it). Startup uses the blocking flasher for the boot voice.
    int      blink_left    = 0;
    bool     blink_on      = false;
    uint32_t blink_t       = 0;
    auto leds_write = [&](bool on){
        hw.dac.WriteValue(DacHandle::Channel::ONE, on ? (uint16_t)LED1_MAX : 0);
        hw.dac.WriteValue(DacHandle::Channel::TWO, on ? (uint16_t)LED2_MAX : 0);
        g_led3_duty = on ? LED3_INTENSITY : 0.0f;
    };
    auto blink_start = [&](int n, uint32_t now){
        blink_left = n;
        if(n > 0){ leds_write(true); blink_on = true; blink_t = now + 110; }
        else     { leds_write(false); blink_on = false; }
    };
    auto blink_tick = [&](uint32_t now){
        if(blink_left == 0 && !blink_on) return;
        if((int32_t)(now - blink_t) < 0)  return;
        if(blink_on){ leds_write(false); blink_on = false; blink_left--; blink_t = now + 140; }
        else if(blink_left > 0){ leds_write(true); blink_on = true; blink_t = now + 110; }
    };

    // Blocking flash used only at startup to announce the boot/restored voice.
    auto flash_voice_leds = [&](int count){
        for(int f = 0; f < count; f++){
            leds_write(true);  System::Delay(110);
            leds_write(false); System::Delay(140);
        }
    };

    // Startup: flash the LEDs to announce the boot/restored voice (its cycle
    // position) — same gesture as a live voice switch.
    flash_voice_leds(cycle_pos(g_voice_idx));

    // ── Polyphonic Drums state ────────────────────────────────────────────────
    bool     hit_was_alive[MAX_FINGERS] = {};   // debounced-alive, previous frame
    uint32_t hit_last_alive[MAX_FINGERS]= {};   // last RAW-alive time (for lift debounce)
    float    prs_sm[MAX_FINGERS]        = {};   // lightly-smoothed pressure
    uint32_t last_retrig[MAX_FINGERS]   = {};   // last held re-attack time (refractory)
    // Genuine taps (a real lift + re-tap) retrigger with NO limit. A tracker
    // dropout shorter than TAP_GAP_MS is treated as a flicker — the note keeps
    // holding, no spurious retrigger. While HELD, a sharp pressure rise
    // re-attacks but is rate-limited by the voice's retrig_ms.
    static constexpr uint32_t TAP_GAP_MS  = 40;    // min off-time to count as a real lift
    static constexpr float    PRS_SMOOTH  = 0.5f;  // light smoothing (preserve the attack edge)
    static constexpr float    RISE_THRESH = 18.0f; // % pressure rise/frame = a held re-attack

    // Trigger drum slot `s` from a tap at (pos, pressure): zone → drum, fine
    // position → interval pitch, pressure → velocity (loudness/brightness/drive).
    auto trigger_drum = [&](int s, int32_t pos, int32_t pressure){
        float p    = clampf((float)pos / 1000.0f, 0.0f, 1.0f);
        int   zone = (int)(p * 4.0f); if(zone > 3) zone = 3;
        int   vidx = MULTI_ZONES[zone];
        const Voice& V = VOICES[vidx];
        float subp = p * 4.0f - (float)zone;
        int   step = (int)(subp * (float)MULTI_NINTERVALS);
        if(step >= MULTI_NINTERVALS) step = MULTI_NINTERVALS - 1;
        float freq = V.freq_low * MULTI_INTERVALS[step];
        float v    = FIX_DRUM ? 1.0f : clampf((float)pressure / 100.0f, 0.0f, 1.0f);
        float amp  = 0.72f * ((1.0f - V.vel_sens) + V.vel_sens * v);
        // Cutoff from velocity, same curve as the mono path.
        float cutoff_oct = V.cutoff_oct_max * smootherstep_f(v);
        float oa = cutoff_oct * 0.6931f;
        float om = 1.0f + oa*(1.0f + oa*(0.5f + oa*(0.1667f + oa*0.0417f)));
        float tf = (V.keytrack >= 0.999f) ? freq
                   : V.freq_low * powf(freq / V.freq_low, V.keytrack);
        float cutoff = clampf(tf * V.cutoff_mult * om, 20.0f, 18000.0f);
        float drive  = 1.0f + smoothstep_f(v) * V.drive_max;
        drum_trigger(g_hits[s], vidx, freq, amp, cutoff, drive, hw.AudioSampleRate());
        g_active_voice = vidx;   // header shows the last-hit drum
    };

    // Fire one drum (bank index `bankidx`) on hit-pool slot `slot` at full
    // velocity, at its natural low pitch. Used by the kit preview below.
    auto preview_drum = [&](int slot, int bankidx){
        const Voice& V = VOICES[bankidx];
        float freq   = V.freq_low;
        // Bright enough that the noisy drums (snare/hat) crack through; the
        // tonal drums stay dark via their own low cutoff_mult.
        float cutoff = clampf(freq * V.cutoff_mult * 3.0f, 60.0f, 16000.0f);
        float drive  = 1.0f + 0.6f * V.drive_max;
        drum_trigger(g_hits[slot], bankidx, freq, 0.85f, cutoff, drive, hw.AudioSampleRate());
        // ONE-SHOT: jump to full amp and immediately ungate so the hit decays on
        // its own. A preview has no finger-lift to clear the gate, so leaving it
        // gated would sustain the drum's noise forever (the "100% noise when Drums
        // selected" bug). attack_ms (~1 ms) makes skipping the ramp inaudible.
        g_hits[slot].amp  = g_hits[slot].amp_tgt;
        g_hits[slot].gate = false;
    };

    while(true)
    {
        // Drain any pending USB serial commands (live voice tuning).
        serial_tune_poll();

        // ── FSR → master volume ──────────────────────────────────────────────
        // Full volume through the deadzone (light pressure), then ramps to 0 as
        // pressure increases down to FSR_MIN.
        uint16_t fsr_raw = hw.adc.Get(0);
        float    vol;
        if(fsr_raw >= fsr_dead)     vol = 1.0f;
        else if(fsr_raw <= FSR_MIN) vol = 0.0f;
        else                        vol = (float)(fsr_raw - FSR_MIN) * fsr_active_inv;
        g_master_vol = vol;

        // Schmitt-latch "FSR pressed to the floor" (hysteresis so a steady hard
        // press doesn't chatter the tap edge). Note the FSR reads INVERSELY: a
        // hard press drives fsr_raw down to/below FSR_MIN (vol -> 0), while easing
        // off raises it (vol -> 1). The voice-select gesture is handled below,
        // after touch detection, so it can require no glass touch.
        if(fsr_raw <= FSR_MIN) full_state = true;    // pressed to the floor = "full"
        else if(vol >= 0.4f)   full_state = false;   // clearly eased off
        // else: hold previous state in the dead band

        read_electrodes(data);

        int32_t max_delta=1;
        for(int i=0;i<N_CH;i++){
            delta[i]=(int32_t)baseline[i]-(int32_t)data[i];
            if(delta[i]<0)delta[i]=0;
            if(delta[i]>max_delta)max_delta=delta[i];
        }

        int n_raw=detect_raw(delta,raw,MAX_FINGERS);
        bool touching=(n_raw>0);

        // ── FSR voice-select gesture (hold FSR + tap/play the glass) ─────────
        // ENTER: press the FSR fully (to the mute floor) with NO glass touch for
        //   ENTER_MS. LEDs blink the current voice's position.
        // PLAY/ADVANCE: with the FSR held, each GLASS TAP advances to the next
        //   voice (wrapping, single blink); the voice then plays through the REAL
        //   note path driven by your actual glass pressure/position, so every
        //   filter/effect matches — press for the Moog cutoff sweep, slide for
        //   pitch, lift to release. The Drums kit plays a canned kick + snare
        //   "one-two" instead.
        // KEEP: release the FSR → persist + exit; the saved position blinks
        //   NON-blocking, so you can start playing immediately.
        {
            uint32_t now = System::GetNow();
            // Bridge brief touch dropouts so a flicker mid-slide isn't read as a
            // lift + new tap (advancing unintentionally).
            if(touching) touch_last_ms = now;
            bool held = touching || (int32_t)(now - touch_last_ms) < 60;

            if(sel_mode == 0){
                if(full_state && !touching){
                    if(full_since == 0) full_since = now;
                    idle_since = now; last_touch_ms = now;   // freeze idle/rebaseline
                    if(now - full_since >= VOICE_SELECT_ENTER_MS){
                        sel_mode     = 1;
                        g_amp_target = 0.0f;                 // silence any held note
                        hw.PrintLine("[voice select] %s (%d/%d) - tap/slide glass; release FSR to keep",
                                     VOICES[g_voice_idx].name, cycle_pos(g_voice_idx), cycle_total());
                        blink_start(cycle_pos(g_voice_idx), now);  // blink the current voice number
                    }
                } else {
                    full_since = 0;                          // touched or not full → disarm
                }
            } else {
                idle_since = now; last_touch_ms = now;       // freeze idle while selecting
                g_master_vol = 0.85f;                        // monitor level (the FSR mutes)
                if(!full_state){
                    // FSR released → keep the shown voice (persist) and exit. The
                    // saved-position blink is NON-blocking (driven from the LED
                    // section) so playing isn't stalled.
                    storage.GetSettings().voice_idx = g_voice_idx;
                    storage.Save();                          // writes QSPI only if changed
                    hw.PrintLine("[voice kept %d/%d] %s",
                                 cycle_pos(g_voice_idx), cycle_total(), VOICES[g_voice_idx].name);
                    drum2_at = 0;
                    sel_mode = 0; full_since = 0;
                    blink_start(cycle_pos(g_voice_idx), now); // confirm saved voice (non-blocking)
                } else {
                    bool tap = held && !prev_held &&
                               (int32_t)(now - last_tap_ms) >= (int32_t)VOICE_SELECT_TAP_GAP_MS;
                    if(tap){
                        // glass tap (debounced rising edge) → advance one voice (wraps)
                        last_tap_ms = now;
                        g_voice_idx = cycle_next(g_voice_idx);
                        hw.PrintLine("[voice %d/%d] %s",
                                     cycle_pos(g_voice_idx), cycle_total(), VOICES[g_voice_idx].name);
                        blink_start(1, now);                 // single blink per advance
                    }
                    if(!VOICE_PREVIEW_ENABLED){
                        g_finger_on = false; g_amp_target = 0.0f;   // silent cycling
                        prev_held = held;
                        blink_tick(now); System::Delay(CONTROL_DELAY_MS); continue;
                    }
                    if(g_voice_idx == MULTI_IDX){
                        // Drums kit: a tap fires a kick + snare "one-two" (one-shots).
                        if(tap){
                            g_finger_on = false; g_amp_target = 0.0f;   // hush mono
                            preview_drum(0, MULTI_ZONES[0]);
                            drum2_at = now + 150;
                        }
                        if(drum2_at && (int32_t)(now - drum2_at) >= 0){
                            preview_drum(1, MULTI_ZONES[1]);            // delayed snare
                            drum2_at = 0;
                        }
                        prev_held = held;
                        blink_tick(now); System::Delay(CONTROL_DELAY_MS); continue;
                    }
                    // Mono voice: fall through to the REAL note path below (pressure
                    // pinned to 100%) — the preview is the actual voice, all effects.
                }
            }
            prev_held = held;   // for next loop's tap edge detection
        }

        // ── Auto-rebaseline ───────────────────────────────────────────────────
        if(touching){
            idle_since=System::GetNow();
        } else {
            if(System::GetNow()-idle_since>=REBASELINE_IDLE_MS){
                tracked[0].alive=tracked[1].alive=false;
                capture_baseline(baseline);
                if(!serial_display_muted()) hw.PrintLine("[rebaseline]");
                idle_since=System::GetNow();
            }
        }

        // ── 5 s idle → chase + FSR recalibration, loops until touch ──────────
        if(touching){
            last_touch_ms = System::GetNow();
        } else if(System::GetNow() - last_touch_ms >= 5000){
            idle_chase_and_calibrate();
            last_touch_ms = System::GetNow();
            idle_since    = System::GetNow();
            if(!serial_display_muted()) hw.PrintLine("[fsr recal] fsr_max=%d", (int)fsr_max);
        }

        update_tracked(raw,n_raw);

        // ── Finger 1 → Pitch + Filter cutoff + Vibrato ───────────────────────
        bool f1_on = tracked[0].alive;
        if(f1_on) f1_last_seen_ms = System::GetNow();
        bool f1_fresh = !f1_was_on && (System::GetNow() - f1_last_seen_ms >= F1_REQUANTIZE_MS);

        // ── MultiVoice routing ───────────────────────────────────────────────
        // Drums mode is polyphonic: each of up to 4 fingers drives its own drum
        // slot — a fresh touch triggers the zone's drum (pitch from the fine
        // position), lifting it releases that slot. The mono engine is held
        // silent. Other voices stay monophonic and use the finger-1 path below.
        bool multi = (g_voice_idx == MULTI_IDX);
        if(!multi) g_active_voice = g_voice_idx;

        // ── Re-attack detector (all fingers, with lift debounce) ─────────────
        // deb_alive[] = alive after debouncing brief dropouts. A debounced
        // rising edge is a genuine tap (fires with NO limit); a sharp pressure
        // rise while held re-attacks (rate-limited by retrig_ms).
        bool     reatk[MAX_FINGERS] = {};
        bool     deb_alive[MAX_FINGERS] = {};
        uint32_t now_ms = System::GetNow();
        for(int i = 0; i < MAX_FINGERS; i++){
            bool raw = tracked[i].alive;
            if(raw) hit_last_alive[i] = now_ms;
            // Bridge sub-TAP_GAP_MS dropouts so a flicker isn't seen as a lift.
            bool deb = raw || (int32_t)(now_ms - hit_last_alive[i]) < (int32_t)TAP_GAP_MS;
            deb_alive[i] = deb;
            if(!deb) continue;                              // truly lifted

            int vidx;
            if(multi){
                int z = (int)(clampf((float)tracked[i].pos/1000.0f,0.0f,1.0f) * 4.0f);
                if(z > 3) z = 3; vidx = MULTI_ZONES[z];
            } else vidx = g_active_voice;
            float rms = VOICES[vidx].retrig_ms;
            float prs = (float)tracked[i].pressure;

            if(!hit_was_alive[i]){
                // Genuine new tap (after a real lift) — fire immediately, no cap.
                prs_sm[i] = prs; reatk[i] = true; last_retrig[i] = now_ms;
            } else if(raw){
                // Held: re-attack on a sharp pressure rise, capped by retrig_ms.
                float prev = prs_sm[i];
                prs_sm[i] = prev*PRS_SMOOTH + prs*(1.0f - PRS_SMOOTH);
                if(rms > 0.0f && (prs_sm[i] - prev) > RISE_THRESH
                   && (int32_t)(now_ms - last_retrig[i]) >= (int32_t)rms){
                    reatk[i] = true; last_retrig[i] = now_ms;
                }
            }
        }

        if(multi){
            // Poly drums: each finger drives its own slot.
            for(int i = 0; i < POLY; i++){
                if(reatk[i])                                  trigger_drum(i, tracked[i].pos, tracked[i].pressure);
                else if(!deb_alive[i] && hit_was_alive[i])    g_hits[i].gate = false;  // lifted → release
            }
            g_amp_target = 0.0f;   // mono engine silent; the poly drums play in the callback
        } else {
            // Mono: a re-attack mid-note re-articulates it (initial touch is
            // handled by the amp gate below).
            if(reatk[0] && hit_was_alive[0]) g_retrig = true;
        }

        for(int i = 0; i < MAX_FINGERS; i++) hit_was_alive[i] = deb_alive[i];

        // Bind the active voice (last-hit drum in MultiVoice mode) for finger-2 / header.
        const Voice& VOICE = g_live_tune ? g_live_voice : VOICES[g_active_voice];

        if(f1_on && !multi)
        {
            float pos01 = clampf((float)tracked[0].pos / 1000.0f, 0.0f, 1.0f);
            float prs01 = clampf((float)tracked[0].pressure / 100.0f, 0.0f, 1.0f); // hard clamp

            // ── Direct exponential frequency mapping across the voice range ──
            // freq = freq_low * (freq_high/freq_low)^pos01
            //      = freq_low * e^(pos01 * log_freq_ratio)
            float earg = pos01 * VOICE.log_freq_ratio;
            float ex   = 1.0f + earg*(1.0f + earg*(0.5f + earg*0.1667f));
            float freq_continuous = clampf(VOICE.freq_low * ex, VOICE.freq_low, VOICE.freq_high);

            // Convert to MIDI for quantizer: midi = 69 + 12*log2(freq/440)
            // log2(x) = ln(x)/ln(2); ln approx via y=(x-1)/(x+1) series
            float freq_ratio = freq_continuous / 440.0f;
            float y = (freq_ratio - 1.0f) / (freq_ratio + 1.0f);
            float y2 = y*y;
            float ln_ratio = 2.0f*y*(1.0f + y2*(0.3333f + y2*0.2f));
            float midi_raw = 69.0f + 17.3123f * ln_ratio; // 17.3123 = 12/ln(2)

            float freq; // final frequency to use

            if(QUANTIZE_ENABLED && VOICE.quantize && f1_fresh)
            {
                // Touch-down: snap to nearest scale note
                f1_midi_base = quantize_midi(midi_raw, VOICE.scale, VOICE.scale_len);
                // Convert quantized note number back to Hz
                float qe = (f1_midi_base - 69.0f) * 0.05776f;
                freq = clampf(440.0f*(1.0f+qe*(1.0f+qe*(0.5f+qe*0.1667f))), VOICE.freq_low, VOICE.freq_high);
                f1_sliding = false;
                // Chord voices: build the diatonic triad from the snapped root and
                // hand the engine the 3rd/5th frequency ratios (it adds those notes
                // above the root). Recomputed only on a fresh note; while sliding the
                // ratios hold so the whole chord bends together.
                if(VOICE.chords){
                    int third, fifth;
                    diatonic_triad((int)f1_midi_base, VOICE.scale, VOICE.scale_len, third, fifth);
                    // Voicing width: lift the 3rd by chord_spread octaves (0 = close
                    // root–3rd–5th cluster; 1 = open root–5th–10th, clearer over the
                    // sub-osc). Per-voice (Voice::chord_spread).
                    third += 12 * VOICE.chord_spread;
                    g_chord_ratio2 = exp2f((float)third / 12.0f);
                    g_chord_ratio3 = exp2f((float)fifth / 12.0f);
                }
            }
            else
            {
                // Sliding or quantize disabled: follow continuous frequency directly
                f1_midi_base = f1_midi_base*0.88f + midi_raw*0.12f;
                freq = freq_continuous;
                f1_sliding = true;
            }
            freq = clampf(freq, VOICE.freq_low, VOICE.freq_high);

            // Cutoff: base tracks 2× pitch, pressure opens it up
            // Light press = 1× pitch (dark, Moog-like tracking)
            // Hard press = up to 12000 Hz (wide open)
            // ── Moog-style exponential cutoff ─────────────────────────────
            // At zero pressure: cutoff = 0.5x pitch (below fundamental, very dark)
            // At full pressure: cutoff = 0.5x * 2^(6*prs) — exponential opening
            // 6 octaves of sweep = factor of 64x, so fully open = 32x pitch
            // This gives the classic Moog "closed → wah → bright" feel.
            // Power-3 on pressure so the bottom half stays dark and controlled.
            // ── Smootherstep pressure curves (6x⁵ - 15x⁴ + 10x³) ──────────
            // Starts slow, steep in the middle, eases gently at the top.
            // Much more playable than power curves — feels like a real knob.
            auto smoothstep = [](float x) -> float {
                x = x<0?0:x>1?1:x;
                return x*x*(3.0f - 2.0f*x);
            };
            auto smootherstep = [](float x) -> float {
                x = x<0?0:x>1?1:x;
                return x*x*x*(x*(x*6.0f - 15.0f) + 10.0f);
            };

            // Cutoff: smootherstep — very dark at low, dramatic in mid, eases at top.
            // Base cutoff and sweep depth are voice-defined.
            float prs_cut  = smootherstep(prs01);
            float cutoff_oct = VOICE.cutoff_oct_max * prs_cut;
            float oct_arg  = cutoff_oct * 0.6931f;
            float oct_mult = 1.0f + oct_arg*(1.0f + oct_arg*(0.5f + oct_arg*(0.1667f + oct_arg*0.0417f)));
            // Keytracking: how much the cutoff base follows pitch. keytrack=1 is
            // full tracking (cutoff ∝ freq); <1 holds the cutoff lower as pitch
            // rises, for a more consistent tone across the range. (constexpr →
            // the common keytrack=1 case folds to no powf.)
            float track_freq = (VOICE.keytrack >= 0.999f)
                ? freq
                : VOICE.freq_low * powf(freq / VOICE.freq_low, VOICE.keytrack);
            float cutoff   = clampf(track_freq * VOICE.cutoff_mult * oct_mult, 20.0f, 18000.0f);

            // Vibrato: smoothstep, only activates above 60% pressure
            float vib_in = clampf((prs01 - 0.6f) / 0.4f, 0.0f, 1.0f);
            float vib    = smoothstep(vib_in) * 0.8f;

            // Drive: smoothstep — clean at low, progressively warmer (voice-defined max)
            float drive = 1.0f + smoothstep(prs01) * VOICE.drive_max;

            // ── Velocity → loudness ──────────────────────────────────────────
            // Capture peak pressure for a short window at onset, then hold it.
            if(!f1_was_on){ vel_active = true; vel_peak = prs01; vel_start = System::GetNow(); }
            if(vel_active){
                if(prs01 > vel_peak) vel_peak = prs01;
                if(System::GetNow() - vel_start >= VEL_WINDOW_MS) vel_active = false;
                vel = vel_peak;
            }
            float vel_scale = (1.0f - VOICE.vel_sens) + VOICE.vel_sens * vel;

            g_target_freq    = freq;
            g_target_cutoff  = cutoff;
            g_vibrato_depth  = vib;
            g_target_drive   = drive;
            g_finger_on      = true;
            g_amp_target     = 0.72f * vel_scale;

        }
        else if(!multi)
        {
            // Finger lifted: gate off, reset slide state
            f1_sliding = false;
            g_finger_on  = false;
            g_amp_target = 0.0f;
            // Do NOT reset cutoff here — let it close with the amplitude
            // in the audio callback to avoid discontinuity clicks
        }

        // ── LED targets + slew ────────────────────────────────────────────────
        // Three-LED position indicator. Linear "tents" hand off cleanly:
        //   LED1 (DAC1, middle)       → 1 - 2*|p - 0.5|
        //   LED2 (DAC2, pos 0 end)    → max(0, 1 - 2*p)
        //   LED3 (PWM A0, pos 1 end)  → max(0, 2*p - 1)
        // Fourth-root curve (gamma=0.25) keeps each LED above its forward-voltage
        // floor at the handoff points (50% raw → ~84% drive). Pressure adds a
        // subtle ~75%→100% intensity scale.
        // The voice-select blink overlays the position display only while a blink
        // is actually animating (entry count, per-advance tick, saved-voice
        // confirm). Otherwise the normal position display runs — so the LEDs follow
        // the finger during the live preview too, and resume after the save blink.
        if(blink_left > 0 || blink_on){
            blink_tick(System::GetNow());
        } else {
            float led1_target = 0.0f, led2_target = 0.0f, led3_target = 0.0f;
            if(f1_on) {
                float p01      = clampf((float)tracked[0].pos / 1000.0f, 0.0f, 1.0f);
                float pr01     = clampf((float)tracked[0].pressure / 100.0f, 0.0f, 1.0f);
                float prs_mult = 0.75f + 0.25f * pr01;
                float l1_raw   = 1.0f - 2.0f * p01;                 if(l1_raw < 0) l1_raw = 0;
                float l3_raw   = 2.0f * p01 - 1.0f;                 if(l3_raw < 0) l3_raw = 0;
                float l2_raw   = 1.0f - 2.0f * fabsf(p01 - 0.5f);   if(l2_raw < 0) l2_raw = 0;
                led1_target    = sqrtf(sqrtf(l2_raw)) * prs_mult;  // middle tent
                led2_target    = sqrtf(sqrtf(l1_raw)) * prs_mult;  // pos 0 ramp
                led3_target    = sqrtf(sqrtf(l3_raw)) * prs_mult * LED3_INTENSITY;
            }
            // Snap on touch onset so the LEDs respond instantly to the initial touch;
            // smooth during sustained touch and on lift-off.
            if(f1_on && !f1_was_on) {
                s_led1 = led1_target;
                s_led2 = led2_target;
                s_led3 = led3_target;
            } else {
                float smooth3 = f1_on ? LED_SMOOTH : LED3_RELEASE_SMOOTH;
                s_led1 = s_led1 * LED_SMOOTH + led1_target * (1.0f - LED_SMOOTH);
                s_led2 = s_led2 * LED_SMOOTH + led2_target * (1.0f - LED_SMOOTH);
                s_led3 = s_led3 * smooth3   + led3_target * (1.0f - smooth3);
            }
            hw.dac.WriteValue(DacHandle::Channel::ONE, (uint16_t)(s_led1 * LED1_MAX));
            hw.dac.WriteValue(DacHandle::Channel::TWO, (uint16_t)(s_led2 * LED2_MAX));
            g_led3_duty = s_led3;  // audio callback sigma-delta's this onto A0
        }

        f1_was_on = f1_on;

        // ── Finger 2 → Detune + Bitcrush + Ring mod (mono voices only) ───────
        if(tracked[1].alive && !multi)
        {
            float pos01 = clampf((float)tracked[1].pos / 1000.0f, 0.0f, 1.0f);
            float prs01 = clampf((float)tracked[1].pressure / 100.0f, 0.0f, 1.0f); // hard clamp

            // Detune: finger 2 position maps to ±7 semitones (±1 fifth)
            // Centre (pos=0.5) = no detune, left = flat, right = sharp
            float detune_pos  = (pos01 - 0.5f) * 3.0f;  // ±1.5 semitones raw

            // Wavefold: smootherstep — barely there until deliberate pressure
            auto smootherstep2 = [](float x) -> float {
                x = x<0?0:x>1?1:x;
                return x*x*x*(x*(x*6.0f - 15.0f) + 10.0f);
            };
            // Apply smootherstep twice and cap at 0.20 — effect only arrives
            // with real deliberate pressure, stays subtle even at max
            float ss2 = smootherstep2(prs01);
            float ss3 = smootherstep2(ss2);   // triple smootherstep — very lazy
            float bc  = smootherstep2(ss3) * VOICE.fold_max; // ceiling is voice-defined

            // Detune: smoothstep on position offset from centre
            auto smoothstep2 = [](float x) -> float {
                x = x<0?0:x>1?1:x;
                return x*x*(3.0f - 2.0f*x);
            };
            // Detune amount scales with pressure via smoothstep — no detune at low pressure
            float det_prs = smoothstep2(prs01);

            // Ring mod: smoothstep, only above 85% pressure (ceiling voice-defined)
            float rm_in = clampf((prs01 - 0.85f) / 0.15f, 0.0f, 1.0f);
            float rm    = smoothstep2(rm_in) * VOICE.ringmod_max;

            g_target_detune = detune_pos * det_prs; // pressure gates detune amount
            g_bitcrush      = bc;
            g_ringmod       = rm;
        }
        else
        {
            // No second finger: return detune to zero, clear effects
            g_target_detune = 0.0f;
            g_bitcrush      = 0.0f;
            g_ringmod       = 0.0f;
        }

        // ── Serial display (throttled; the control loop above runs much faster) ─
        // Suppressed while live-tuning so the protocol channel stays clean
        // (re-enable with `mon 1`).
        if(serial_display_muted() ||
           System::GetNow() - last_print_ms < PRINT_INTERVAL_MS){
            System::Delay(CONTROL_DELAY_MS);
            continue;
        }
        last_print_ms = System::GetNow();

        if(g_voice_idx == MULTI_IDX)
            hw.PrintLine("VOICE %d/%d  Drums [Kick|Snare|Tom|Hat]  -> %s",
                cycle_pos(g_voice_idx), cycle_total(), VOICE.name);
        else
            hw.PrintLine("VOICE %d/%d  %s  (%d-%dHz)",
                cycle_pos(g_voice_idx), cycle_total(), VOICE.name,
                (int)VOICE.freq_low, (int)VOICE.freq_high);
        hw.PrintLine(" CH | DELTA | BAR");
        hw.PrintLine("----+-------+--------------------");
        for(int i=0;i<N_CH;i++){
            make_bar(bar,delta[i],max_delta,20);
            char mk=' ';
            for(int s=0;s<2;s++)
                if(tracked[s].alive&&i==tracked[s].peak_ch)mk=(char)('1'+s);
            hw.PrintLine(" %2d | %5d | %s %c",i,(int)delta[i],bar,mk);
        }

        make_pos_bar(pos_bar,40);
        hw.PrintLine("\nPOS  %s",pos_bar);

        // Always print both finger rows — inactive shows dashes so line count is stable
        for(int slot=0;slot<2;slot++){
            if(tracked[slot].alive){
                make_bar(bar,tracked[slot].pressure,100,16);
                hw.PrintLine("  %d  pos:%4d  prs:%3d%%  %s",
                    slot+1,(int)tracked[slot].pos,(int)tracked[slot].pressure,bar);
            } else {
                hw.PrintLine("  %d  pos: ---  prs: --%%  [                ]",slot+1);
            }
        }

        // FSR (master-volume) pressure: unpressed = 0%, pressed to mute = 100%.
        int fsr_pct = (int)((1.0f - vol) * 100.0f + 0.5f);
        if(fsr_pct < 0) fsr_pct = 0; else if(fsr_pct > 100) fsr_pct = 100;
        make_bar(bar, fsr_pct, 100, 16);
        hw.PrintLine(" FSR pressure  %3d%%  %s", fsr_pct, bar);

        // Audio param display — fixed width, always same line count
        int freq_i  = (int)g_target_freq;
        int cut_i   = (int)g_target_cutoff;
        int det_i10 = (int)(g_target_detune*10);
        int bc_i    = (int)(g_bitcrush*100);
        int vib_i   = (int)(g_vibrato_depth*100);
        hw.PrintLine("  freq:%4dHz  cut:%5dHz  det:%c%d.%ds  fx:%2d%%  vib:%2d%%",
            freq_i, cut_i,
            det_i10<0?'-':'+',
            det_i10<0?(-det_i10)/10:det_i10/10,
            det_i10<0?(-det_i10)%10:det_i10%10,
            bc_i, vib_i);
        hw.PrintLine("--------------------------------------------");

        System::Delay(CONTROL_DELAY_MS);
    }
}
