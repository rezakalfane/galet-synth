#pragma once
// Audio engine: the synthesis state, the polyphonic drum voices, and the audio
// callback. The g_* params below are the control surface — the touch loop (in
// main.cpp) writes them, the audio callback reads them.
#include "daisy_seed.h"
#include "voice.h"   // Voice, VOICES, MULTI_*
#include "touch.h"   // MAX_FINGERS
#include "dsp.h"     // oscillators / filters used by the engine

// Hardware + voice-selection globals (defined in main.cpp; the engine reads them).
extern daisy::DaisySeed hw;
extern daisy::GPIO      led3;
extern volatile int     g_voice_idx;
extern volatile int     g_active_voice;

// Audio control params — written by the control loop, read by the audio callback.
extern volatile float g_target_freq, g_target_cutoff, g_target_detune, g_target_drive;
extern volatile float g_vibrato_depth, g_bitcrush, g_ringmod;
extern volatile bool  g_finger_on;
extern volatile float g_amp_target;
extern volatile bool  g_retrig;       // mono: re-articulate the note (re-attack)
extern volatile float g_led3_duty;    // 0-1, sigma-delta'd to LED3 in the audio cb
extern volatile float g_master_vol;   // 0-1, master volume from the FSR
// Chord voices (Voice::chords): frequency ratios of the upper two triad notes
// vs the root, set by the control loop at each quantized note. 1.0 = unison (no
// extra note). Ignored by single-note voices.
extern volatile float g_chord_ratio2, g_chord_ratio3;

// ── Polyphonic drum engine (Drums MultiVoice only) ────────────────────────────
// A pool of independent drum voices, one per finger. Each is a self-contained
// slice of the synth (oscillators + noise + ladder filter + amp & pitch
// envelopes) with its own state, so up to POLY drums sound at once. Params are
// latched at the tap (drum, pitch, velocity-scaled level, cutoff, drive); the
// note plays its attack while the finger is down and releases when it lifts.
static constexpr int POLY = MAX_FINGERS;
struct DrumHit {
    bool  active=false, gate=false;
    int   vidx=9;
    float freq=80.0f, amp_tgt=0.0f, cutoff=200.0f, drive=1.0f;
    float ph1=0,ph2=0,phs=0,pho=0;
    float amp=0.0f, pmult=1.0f, noise_lp=0.0f;
    float lp_state=0.0f, lp_coeff=1.0f;   // per-drum 1-pole lowpass
    float atk_c=0, rel_c=0, penv_c=0, penv_start=1.0f;
};

extern DrumHit g_hits[POLY];
// Start a drum hit on slot h (called from the control loop's tap router).
void drum_trigger(DrumHit& h, int vidx, float freq, float amp_tgt,
                  float cutoff, float drive, float sr);
// Registered with hw.StartAudio() in main.
void AudioCallback(daisy::AudioHandle::InputBuffer,
                   daisy::AudioHandle::OutputBuffer, size_t);
