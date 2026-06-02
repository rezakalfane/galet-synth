#include "engine.h"
#include "persist.h"   // g_bank — the live, editable voice bank the engine plays

using namespace daisy;
using namespace daisy::seed;

volatile float g_target_freq    = 65.41f;
volatile float g_target_cutoff  = 100.0f;  // starts dark, pressure opens it
volatile float g_target_detune  = 0.0f;     // semitones osc2 offset
volatile float g_target_drive   = 1.0f;     // waveshaper input gain
volatile float g_vibrato_depth  = 0.0f;     // 0–1, LFO depth
volatile float g_bitcrush       = 0.0f;     // 0–1, amount of crush
volatile float g_ringmod        = 0.0f;     // 0–1, ring mod wet
volatile bool  g_finger_on      = false;
volatile float g_amp_target     = 0.0f;
volatile bool  g_retrig         = false;    // mono: re-articulate the note (re-attack)
volatile float g_led3_duty      = 0.0f;     // 0–1, sigma-delta'd to LED3 in audio cb
volatile float g_master_vol     = 1.0f;     // 0–1, master volume from FSR on A1
volatile float g_chord_ratio2   = 1.0f;     // chord 3rd  ratio vs root (1 = unison)
volatile float g_chord_ratio3   = 1.0f;     // chord 5th  ratio vs root (1 = unison)
volatile float g_reverb_decay   = 0.85f;    // reverb tail length (comb feedback)
volatile float g_reverb_level   = 1.5f;     // reverb wet return gain (overall level)
volatile float g_delay_time_ms  = 380.0f;   // delay/echo time
volatile float g_delay_feedback = 0.40f;    // delay repeats (0..~0.95)
volatile float g_delay_level    = 0.55f;    // delay wet return gain

// ── Audio DSP state (audio callback only) ─────────────────────────────────────
static float s_freq      = 65.41f;
static float s_cutoff    = 100.0f;
static float s_detune    = 0.0f;
static float s_drive     = 1.0f;
static float s_vibdepth  = 0.0f;
static float s_bitcrush  = 0.0f;
static float s_ringmod   = 0.0f;
static float s_amp       = 0.0f;
static float s_master_vol= 1.0f;

static float s_phase1    = 0.0f;
static float s_phase2    = 0.0f;
static float s_phase_sub = 0.0f;
static float s_phase_oct = 0.0f; // octave-up oscillator (2x freq)
static float s_phase_rm  = 0.0f; // ring mod carrier phase
static float s_lfo_phase = 0.0f;
// Chord voices (Voice::chords): osc1+osc2 phases for the upper two triad notes.
// Their level (vs the root) and voicing spread are per-voice — VOICE.chord_level
// / chord_spread (the latter applied in the control loop's ratio computation).
static float s_phase1b   = 0.0f, s_phase2b = 0.0f; // 3rd
static float s_phase1c   = 0.0f, s_phase2c = 0.0f; // 5th

// Moog ladder filter state
static float s_flt[4]    = {0,0,0,0};

// Shared reverb — one instance, fed by whichever voice has a reverb_send. Its
// ~66 KB of delay lines live in static storage (zero-initialised = silent). HF
// damping is fixed for now; tail length / level are the global g_reverb_* params.
static Reverb s_reverb;
static constexpr float REVERB_DAMP = 0.4f;

// Shared delay/echo — one instance, fed by whichever voice has a delay_send. Its
// 750 ms buffer (~144 KB) is static (zero = silent). Feedback HF damping fixed;
// time / feedback / level are the global g_delay_* params.
static Delay s_delay;
static constexpr float DELAY_DAMP = 0.30f;

// White-noise generator state (xorshift32) for the per-voice noise oscillator.
static uint32_t s_noise_rng = 0x1234567u;
// One-pole low-pass state used to derive high-passed ("tsss") noise from white.
static float    s_noise_lp  = 0.0f;
// High-pass corner coefficient: higher = brighter/higher "tsss" for hi-hats.
static constexpr float NOISE_HP_COEF = 0.8f;

// Slew rates (per sample at 48 kHz)
// Lower value = faster response.
// NOTE: pitch glide and the amp attack/release are now per-voice (see Voice
// glide_ms / attack_ms / release_ms), converted to coefficients in the audio
// callback. The constants below are the ones still shared by all voices.
// Cutoff opening slew. The target is recomputed only at the touch-loop rate
// (~12-16 Hz) and pressure is quantized to integer %, so a too-fast slew lets
// each coarse step through as zipper/stair-stepping. ~35 ms interpolates across
// those steps for a smooth filter sweep while still feeling responsive.
static constexpr float SLEW_CUT    = 0.9994f; // filter cutoff — ~35 ms smooth
static constexpr float SLEW_MISC   = 0.9900f; // drive/vib
// Finger 2 — asymmetric attack/release
static constexpr float SLEW_F2_A   = 0.9800f; // attack  (~10ms)
static constexpr float SLEW_F2_R   = 0.9990f; // release (~200ms smooth fade)


// Mono ladder using the shared global state (the monophonic melodic engine).
static inline float moog(float in, float cutoff, float res, float sr)
{
    return moog_st(in, cutoff, res, sr, s_flt);
}

DrumHit g_hits[POLY];

void drum_trigger(DrumHit& h, int vidx, float freq, float amp_tgt,
                         float cutoff, float drive, float sr)
{
    const Voice& V = g_bank[vidx];
    h.active=true; h.gate=true; h.vidx=vidx;
    h.freq=freq; h.amp_tgt=amp_tgt; h.cutoff=cutoff; h.drive=drive;
    h.ph1=h.ph2=h.phs=h.pho=0.0f;
    h.amp=0.0f; h.noise_lp=0.0f;
    // 1-pole lowpass coefficient from the (latched) cutoff: a = 1 - e^(-2π·fc/sr).
    h.lp_state = 0.0f;
    h.lp_coeff = clampf(1.0f - expf(-6.2831853f * cutoff / sr), 0.0f, 1.0f);
    h.atk_c      = ms_to_coeff(V.attack_ms,    sr);
    h.rel_c      = ms_to_coeff(V.release_ms,   sr);
    h.penv_c     = ms_to_coeff(V.pitch_env_ms, sr);
    h.penv_start = exp2f(V.pitch_env_oct);
    h.pmult      = h.penv_start;
}

static float drum_render(DrumHit& h, float sr)
{
    if(!h.active) return 0.0f;
    const Voice& V = g_bank[h.vidx];
    // Amp envelope: attack while gated, release on lift; free the slot when done.
    float tgt = h.gate ? h.amp_tgt : 0.0f;
    float c   = (tgt > h.amp) ? h.atk_c : h.rel_c;
    h.amp = h.amp*c + tgt*(1.0f-c);
    if(!h.gate && h.amp < 0.0004f){ h.active=false; return 0.0f; }
    // Pitch envelope decays to 1.0.
    h.pmult = 1.0f + (h.pmult - 1.0f) * h.penv_c;
    float f = h.freq * h.pmult;
    // Oscillators + noise
    float mix = osc(h.ph1,V.osc1_wave)*V.osc1_level
              + osc(h.ph2,V.osc2_wave)*V.osc2_level
              + osc(h.phs,V.sub_wave )*V.sub_level
              + osc(h.pho,V.oct_wave )*V.oct_level;
    if(V.noise_level > 0.0f){
        s_noise_rng ^= s_noise_rng<<13; s_noise_rng ^= s_noise_rng>>17; s_noise_rng ^= s_noise_rng<<5;
        float white  = (float)(int32_t)s_noise_rng * (1.0f/2147483648.0f);
        h.noise_lp  += NOISE_HP_COEF * (white - h.noise_lp);
        float bright = white - h.noise_lp;
        mix += (white + (bright*2.0f - white)*V.noise_hp) * V.noise_level;
    }
    mix = fast_tanh(mix * h.drive);
    // Per-drum 1-pole lowpass (cheap — one multiply-add — instead of the 4-pole
    // Moog) at the drum's own cutoff: keeps Kick/Tom dark, Snare/Hat bright.
    h.lp_state += h.lp_coeff * (mix - h.lp_state);
    h.ph1 += f*V.osc1_ratio/sr; if(h.ph1>=1.0f)h.ph1-=1.0f;
    h.ph2 += f*V.osc2_ratio/sr; if(h.ph2>=1.0f)h.ph2-=1.0f;
    h.phs += f*V.sub_ratio /sr; if(h.phs>=1.0f)h.phs-=1.0f;
    h.pho += f*V.oct_ratio /sr; if(h.pho>=1.0f)h.pho-=1.0f;
    return fast_tanh(h.lp_state*1.3f) * h.amp;
}

void AudioCallback(AudioHandle::InputBuffer,
                   AudioHandle::OutputBuffer out, size_t size)
{
    // LED3 fixed-frequency PWM (200 Hz, 240 levels). Per-sample duty smoother
    // (~20 ms TC) interpolates between the main loop's 16 Hz updates so the
    // brightness transitions look continuous even at low intensity.
    static float    led3_duty_smooth = 0.0f;
    static uint32_t pwm_counter      = 0;
    static bool     led3_state       = false;
    constexpr uint32_t PWM_LEVELS = 240;        // 48 kHz / 240 = 200 Hz PWM
    constexpr float    DUTY_TC    = 0.001f;     // ~20 ms time constant per sample
    float           duty_target = g_led3_duty;
    if(duty_target < 0.0f) duty_target = 0.0f; else if(duty_target > 1.0f) duty_target = 1.0f;

    // Snapshot targets
    float tfreq  = g_target_freq;
    float tcut   = g_target_cutoff;
    float tdet   = g_target_detune;
    float tdrv   = g_target_drive;
    float tvib   = g_vibrato_depth;
    float tbc    = g_bitcrush;
    float trm    = g_ringmod;
    float tamp   = g_amp_target;
    float tmvol  = g_master_vol;
    float tcr2   = g_chord_ratio2;   // chord 3rd ratio (used only when VOICE.chords)
    float tcr3   = g_chord_ratio3;   // chord 5th ratio
    float trvdec = g_reverb_decay;   // reverb tail length
    float trvlvl = g_reverb_level;   // reverb wet return gain
    float sr     = hw.AudioSampleRate();
    int   tdlys  = (int)(g_delay_time_ms * 0.001f * sr);  // delay time in samples
    float tdlyfb = g_delay_feedback; // delay repeats
    float tdlylv = g_delay_level;    // delay wet return gain

    // Snapshot the active voice once per block, so a mid-block switch from the
    // touch loop can't tear fields across the block. All VOICE.* reads below
    // resolve against this reference. g_active_voice tracks g_voice_idx except in
    // MultiVoice mode, where the touch loop routes it per tap to a drum.
    int vi = g_active_voice;
    if(vi < 0 || vi >= NUM_VOICES) vi = 0;
    // Live tuning: play the mutable working copy instead of the const bank voice
    // (see serialtune.cpp). A tune-mode transition forces a full coeff recompute
    // (+ state snap) below, since vi itself doesn't change across it.
    static bool s_was_tuning = false;
    bool tune_changed = (s_was_tuning != g_live_tune);
    s_was_tuning = g_live_tune;
    const Voice& VOICE = g_live_tune ? g_live_voice : g_bank[vi];

    // In the Drums MultiVoice the polyphonic drum engine plays instead of the
    // mono path (which the touch loop holds silent). Decided once per block.
    bool poly_drums = (g_voice_idx == MULTI_IDX);

    // Per-voice envelope/glide coefficients (expf isn't constexpr). Recompute
    // only when the active voice changes.
    static int   s_coeff_vi = -1;
    static float s_glide_c, s_atk_c, s_rel_c;
    static float s_penv_c;      // pitch-env decay coefficient
    static float s_penv_start;  // pitch multiplier at onset = 2^pitch_env_oct
    static float s_decay_c;     // amp decay-to-sustain coefficient (0 = no decay)
    static float s_sustain;     // level the gated amp decays to (1 = hold full)
    if(s_coeff_vi != vi || tune_changed){
        s_glide_c    = ms_to_coeff(VOICE.glide_ms,     sr);
        s_atk_c      = ms_to_coeff(VOICE.attack_ms,    sr);
        s_rel_c      = ms_to_coeff(VOICE.release_ms,   sr);
        s_penv_c     = ms_to_coeff(VOICE.pitch_env_ms, sr);
        s_penv_start = exp2f(VOICE.pitch_env_oct);   // 1.0 when pitch_env_oct == 0
        // decay_ms == 0 → no decay: floor 1.0 so s_decay stays pinned at 1 (the
        // gated note holds full, unchanged). decay_ms > 0 → decay toward sustain.
        s_decay_c    = (VOICE.decay_ms > 0.0f) ? ms_to_coeff(VOICE.decay_ms, sr) : 0.0f;
        s_sustain    = (VOICE.decay_ms > 0.0f) ? VOICE.sustain : 1.0f;
        s_coeff_vi   = vi;
        // New voice: snap the continuous DSP state to this voice's targets so the
        // previous voice's settings don't bleed in (esp. the slow-closing cutoff
        // and the ladder filter's stored energy). Voice changes only happen while
        // silent (FSR gesture) or at a drum-tap onset, so snapping is clean and
        // gives each MultiVoice hit a fresh start.
        s_freq     = tfreq;
        s_cutoff   = tcut;
        s_drive    = tdrv;
        s_detune   = tdet;
        s_bitcrush = tbc;
        s_ringmod  = trm;
        s_flt[0] = s_flt[1] = s_flt[2] = s_flt[3] = 0.0f;
    }

    // Live tuning: when a `set` edits the working voice (g_live_rev bumps) but vi
    // is unchanged, recompute just the ms→coeff conversions — no state snap, so
    // glide / cutoff / filter keep flowing smoothly as you turn the "knobs".
    static uint32_t s_live_seen = 0;
    if(g_live_tune && s_live_seen != g_live_rev){
        s_glide_c    = ms_to_coeff(VOICE.glide_ms,     sr);
        s_atk_c      = ms_to_coeff(VOICE.attack_ms,    sr);
        s_rel_c      = ms_to_coeff(VOICE.release_ms,   sr);
        s_penv_c     = ms_to_coeff(VOICE.pitch_env_ms, sr);
        s_penv_start = exp2f(VOICE.pitch_env_oct);
        s_decay_c    = (VOICE.decay_ms > 0.0f) ? ms_to_coeff(VOICE.decay_ms, sr) : 0.0f;
        s_sustain    = (VOICE.decay_ms > 0.0f) ? VOICE.sustain : 1.0f;
        s_live_seen  = g_live_rev;
    }

    // Pitch-envelope trigger: on a note onset (amp target rising from silence),
    // jump the pitch multiplier up; it decays back to 1.0 over pitch_env_ms.
    static float s_pmult     = 1.0f;
    static float s_prev_tamp = 0.0f;
    static float s_decay     = 1.0f;   // amp decay-to-sustain level (1 = peak)
    bool onset = (tamp > 0.001f && s_prev_tamp <= 0.001f);
    // Mono re-articulation: a re-attack while the note is held restarts the amp
    // envelope from zero and retriggers the pitch envelope (poly drums handle
    // their own retrigger via drum_trigger, so ignore it here).
    if(g_retrig){ g_retrig = false; if(!poly_drums){ onset = true; s_amp = 0.0f; } }
    if(onset){ s_pmult = s_penv_start; s_decay = 1.0f; }  // fresh note → reset decay to peak
    s_prev_tamp = tamp;

    for(size_t i=0;i<size;i++)
    {
        // Slew all parameters
        s_freq    = s_freq   *s_glide_c + tfreq*(1.0f-s_glide_c);
        // Cutoff: fast when opening (smoothed), slow when closing (with release)
        { float sc = (tcut > s_cutoff) ? SLEW_CUT : s_rel_c;
          s_cutoff = s_cutoff * sc + tcut * (1.0f - sc); }
        s_drive   = s_drive   * SLEW_MISC + tdrv * (1.0f - SLEW_MISC);
        s_vibdepth= s_vibdepth* SLEW_MISC + tvib * (1.0f - SLEW_MISC);
        // Finger 2 params: fast attack, slow release
        { float sa = tdet     > s_detune   ? SLEW_F2_A : SLEW_F2_R;
          s_detune   = s_detune   * sa + tdet * (1.0f - sa); }
        { float sa = tbc      > s_bitcrush ? SLEW_F2_A : SLEW_F2_R;
          s_bitcrush = s_bitcrush * sa + tbc  * (1.0f - sa); }
        { float sa = trm      > s_ringmod  ? SLEW_F2_A : SLEW_F2_R;
          s_ringmod  = s_ringmod  * sa + trm  * (1.0f - sa); }
        // Amp decay-to-sustain: s_decay rides 1 → sustain over decay_ms while the
        // note is gated (stays pinned at 1 when decay_ms == 0, i.e. every other
        // voice). It scales the gated target so the note plucks down to `sustain`.
        s_decay = s_decay * s_decay_c + s_sustain * (1.0f - s_decay_c);
        float eff_tamp = tamp * s_decay;
        // Asymmetric envelope: attack while rising OR decaying-while-held (track the
        // decay shape quickly — its rate comes from s_decay); release only once the
        // note is actually let go (tamp → 0).
        float slew_amp = (eff_tamp > s_amp || tamp > 0.001f) ? s_atk_c : s_rel_c;
        s_amp = s_amp * slew_amp + eff_tamp * (1.0f - slew_amp);
        // Master volume slew (smooths FSR jitter / 16 Hz update steps).
        s_master_vol = s_master_vol * SLEW_MISC + tmvol * (1.0f - SLEW_MISC);
        // Pitch envelope decays toward 1.0 (no-op when pitch_env_oct == 0).
        s_pmult = 1.0f + (s_pmult - 1.0f) * s_penv_c;

        // Bleed filter state toward zero when silent
        // Prevents stale filter energy from clicking on next note onset
        if(s_amp < 0.001f) {
            s_flt[0] *= 0.999f;
            s_flt[1] *= 0.999f;
            s_flt[2] *= 0.999f;
            s_flt[3] *= 0.999f;
        }

        // During release, close cutoff in sync with amplitude
        // This prevents clicks from abrupt filter state changes
        if(tamp < 0.001f && s_amp > 0.0001f) {
            // Pull cutoff toward silence-floor proportional to amp decay
            float close_target = 30.0f + s_amp * tcut;
            tcut = close_target;
        }

        // LFO for vibrato (6 Hz sine approximation via triangle)
        s_lfo_phase += 6.0f/sr;
        if(s_lfo_phase>=1.0f)s_lfo_phase-=1.0f;
        float lfo = tri(s_lfo_phase); // -1..1

        // Vibrato: modulate frequency by ±1 semitone * depth
        // powf(2, semitones/12) — approx for small values: exp(x*0.0578)
        float vib_amt = lfo * s_vibdepth * 0.05f; // max ±~0.05 semitones*depth
        float freq_vib = s_freq * (1.0f + vib_amt) * s_pmult;  // s_pmult: pitch-env sweep

        // Osc 2 frequency. Lead: finger-2 detune around the base ratio.
        // Bass (and any fixed-interval voice): hold osc2_ratio (e.g. a 5th).
        // VOICE is constexpr, so only the taken branch is compiled in.
        float freq2;
        if(VOICE.osc2_detune) {
            float det_ratio = 1.0f + s_detune * 0.05776f; // linear approx, good for ±1 oct
            freq2 = freq_vib * VOICE.osc2_ratio * det_ratio;
        } else {
            freq2 = freq_vib * VOICE.osc2_ratio;
        }

        // Sub and octave oscillators (ratios are voice-defined).
        float freq_sub = freq_vib * VOICE.sub_ratio;
        float freq_oct = freq_vib * VOICE.oct_ratio;

        // Ring mod carrier. ringmod_ratio=1.0 is unison; non-integer ratios give
        // inharmonic bell/metallic tones.
        float freq_rm = freq_vib * VOICE.ringmod_ratio;

        // Oscillators (mix levels are voice-defined).
        float osc1    = osc(s_phase1,   VOICE.osc1_wave) * VOICE.osc1_level;
        float osc2    = osc(s_phase2,   VOICE.osc2_wave) * VOICE.osc2_level;
        float sub     = osc(s_phase_sub,VOICE.sub_wave)  * VOICE.sub_level;
        float osc_oct = osc(s_phase_oct,VOICE.oct_wave)  * VOICE.oct_level;

        // Noise (xorshift32 → -1..1), voice-defined level. Adds breath / chiff /
        // hiss. noise_hp blends from full white ("shhh") to high-passed ("tsss",
        // for hi-hats). The `if` folds away when a voice sets noise_level = 0.
        float noise = 0.0f;
        if(VOICE.noise_level > 0.0f){
            s_noise_rng ^= s_noise_rng << 13;
            s_noise_rng ^= s_noise_rng >> 17;
            s_noise_rng ^= s_noise_rng << 5;
            float white = (float)(int32_t)s_noise_rng * (1.0f/2147483648.0f);
            // One-pole high-pass = white minus its low-frequency content.
            s_noise_lp += NOISE_HP_COEF * (white - s_noise_lp);
            float bright = white - s_noise_lp;
            // Blend white → high-passed by noise_hp (×2 to make up lost level).
            float n = white + (bright * 2.0f - white) * VOICE.noise_hp;
            noise = n * VOICE.noise_level;
        }

        // Mix oscillator stack + noise
        float mix = osc1 + osc2 + sub + osc_oct + noise;

        // Diatonic chord (Voice::chords): add the snapped triad's 3rd and 5th as
        // extra saw+pulse voices above the root. They ride freq_vib (so they
        // glide/vibrato with the root) scaled by the per-note ratios. Sub and
        // noise stay on the root only — keeps the low end tight and the chord
        // clear — and the upper notes are scaled by VOICE.chord_level for headroom.
        // VOICE is a runtime ref (selection is live), so this is a cheap
        // well-predicted branch — skipped each sample for single-note voices.
        if(VOICE.chords){
            float f3 = freq_vib * tcr2;   // 3rd
            float f5 = freq_vib * tcr3;   // 5th
            float cl = VOICE.chord_level;
            mix += (osc(s_phase1b, VOICE.osc1_wave) * VOICE.osc1_level
                  + osc(s_phase2b, VOICE.osc2_wave) * VOICE.osc2_level) * cl;
            mix += (osc(s_phase1c, VOICE.osc1_wave) * VOICE.osc1_level
                  + osc(s_phase2c, VOICE.osc2_wave) * VOICE.osc2_level) * cl;
            s_phase1b += f3*VOICE.osc1_ratio/sr; if(s_phase1b>=1.0f)s_phase1b-=1.0f;
            s_phase2b += f3*VOICE.osc2_ratio/sr; if(s_phase2b>=1.0f)s_phase2b-=1.0f;
            s_phase1c += f5*VOICE.osc1_ratio/sr; if(s_phase1c>=1.0f)s_phase1c-=1.0f;
            s_phase2c += f5*VOICE.osc2_ratio/sr; if(s_phase2c>=1.0f)s_phase2c-=1.0f;
        }

        // Ring modulation (finger 2 pressure → metallic/bell edge)
        float rm_carrier = tri(s_phase_rm);
        mix = mix*(1.0f-s_ringmod) + (mix*rm_carrier)*s_ringmod;

        // Waveshaper drive before filter (adds harmonics, warms tone)
        mix = fast_tanh(mix * s_drive);

        // Moog ladder filter — no attack boost (was causing onset clicks)
        float eff_cutoff = clampf(s_cutoff, 20.0f, 20000.0f);
        float filtered = moog(mix, eff_cutoff, VOICE.resonance, sr);

        // Wavefold distortion (finger 2 pressure)
        float crushed = wavefold(filtered, s_bitcrush);

        // Soft clip output → the dry instrument signal (pre master volume).
        float dry = fast_tanh(crushed * 1.3f) * s_amp;

        // Reverb + delay as parallel aux sends. Both read the clean instrument
        // signal (`src`) and add their wet return on top, so they don't cascade
        // into each other. Each is skipped when the voice doesn't send (saves CPU
        // and lets the tail decay out naturally). Time/feedback/level/decay are
        // the global g_reverb_* / g_delay_* params.
        float src = dry;
        if(VOICE.reverb_send > 0.0f)
            dry += s_reverb.process(src * VOICE.reverb_send, trvdec, REVERB_DAMP) * trvlvl;
        if(VOICE.delay_send > 0.0f)
            dry += s_delay.process(src * VOICE.delay_send, tdlys, tdlyfb, DELAY_DAMP) * tdlylv;
        float sample = dry * s_master_vol;

        // Drums MultiVoice: replace the (silent) mono output with the summed
        // polyphonic drum voices, soft-clipped, at master volume.
        if(poly_drums){
            float m = 0.0f;
            for(int k = 0; k < POLY; k++) m += drum_render(g_hits[k], sr);
            sample = fast_tanh(m) * s_master_vol;
        }

        out[0][i] = sample;
        out[1][i] = sample;

        // Advance phases
        s_phase1   += freq_vib*VOICE.osc1_ratio/sr;  if(s_phase1  >=1.0f)s_phase1  -=1.0f;
        s_phase2   += freq2   /sr;  if(s_phase2  >=1.0f)s_phase2  -=1.0f;
        s_phase_sub+= freq_sub/sr;  if(s_phase_sub>=1.0f)s_phase_sub-=1.0f;
        s_phase_oct+= freq_oct/sr;  if(s_phase_oct>=1.0f)s_phase_oct-=1.0f;
        s_phase_rm += freq_rm /sr;  if(s_phase_rm >=1.0f)s_phase_rm -=1.0f;

        // LED3 PWM tick. Smooth duty, recompute threshold, compare counter.
        led3_duty_smooth += (duty_target - led3_duty_smooth) * DUTY_TC;
        uint32_t threshold = (uint32_t)(led3_duty_smooth * (float)PWM_LEVELS + 0.5f);
        if(threshold > PWM_LEVELS) threshold = PWM_LEVELS;
        if(++pwm_counter >= PWM_LEVELS) pwm_counter = 0;
        bool desired = (pwm_counter < threshold);
        if(desired != led3_state){ led3.Write(desired); led3_state = desired; }
    }
}
