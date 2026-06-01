#pragma once
// Pure DSP/math primitives — no hardware, no global state (except moog()
// which lives in the engine). Header-only & inline for the audio hot path.
#include <math.h>

// Oscillator shape — set per oscillator, so one voice can mix (e.g. saw root +
// square fifth + sine sub). SQUARE and SAW are naive (not band-limited) — they
// alias at high pitch, but on a low bass voice that just reads as rich harmonics
// for the filter to chew.
enum Waveform { WAVE_TRI, WAVE_SINE, WAVE_SQUARE, WAVE_SAW };

static inline float clampf(float v,float lo,float hi){
    return v<lo?lo:v>hi?hi:v;
}
static inline float smoothstep_f(float x){
    x = x<0?0:x>1?1:x; return x*x*(3.0f - 2.0f*x);
}
static inline float smootherstep_f(float x){
    x = x<0?0:x>1?1:x; return x*x*x*(x*(x*6.0f - 15.0f) + 10.0f);
}

// Fast tanh approximation (Padé)
static inline float fast_tanh(float x)
{
    if(x> 3.0f)return  1.0f;
    if(x<-3.0f)return -1.0f;
    float x2=x*x;
    return x*(27.0f+x2)/(27.0f+9.0f*x2);
}

// Convert a one-pole time constant in milliseconds to a per-sample slew
// coefficient: y += (1-coeff)*(target-y) reaches ~63% after `ms`.
static inline float ms_to_coeff(float ms, float sr){
    if(ms <= 0.0f) return 0.0f;            // instant
    return expf(-1.0f / (ms * 0.001f * sr));
}

// ── Oscillator waveforms (phase ph in [0,1) → sample in [-1,1]) ───────────────
// Triangle wave
static inline float tri(float ph){
    return ph<0.5f?(4.0f*ph-1.0f):(3.0f-4.0f*ph);
}
// Sine via parabolic approximation (no libm call) — matches sin(2π·ph).
static inline float osc_sine(float ph){
    constexpr float PI = 3.14159265f;
    float x = ph*2.0f*PI - PI;                 // -π..π (angle shifted by π)
    float y = 1.27323954f*x - 0.405284735f*x*(x<0?-x:x);
    y = 0.225f*(y*(y<0?-y:y) - y) + y;         // one refinement pass
    return -y;                                 // undo the π shift: sin(x+π) = -sin(x)
}
// Naive square / saw — alias at high pitch but sound rich on a bass voice.
static inline float osc_square(float ph){ return ph<0.5f ? 1.0f : -1.0f; }
static inline float osc_saw(float ph){ return 2.0f*ph - 1.0f; }

// Dispatch on a per-oscillator waveform (runtime, since the active voice can be
// switched live). A small jump table — cheap at audio rate on the H750.
static inline float osc(float ph, Waveform w){
    switch(w){
        case WAVE_SINE:   return osc_sine(ph);
        case WAVE_SQUARE: return osc_square(ph);
        case WAVE_SAW:    return osc_saw(ph);
        case WAVE_TRI:
        default:          return tri(ph);
    }
}

// Moog 4-pole ladder, operating on a caller-supplied state array (so multiple
// voices can each have their own filter).
static inline float moog_st(float in, float cutoff, float res, float sr, float st[4])
{
    float f = 2.0f*cutoff/sr;
    if(f>0.98f)f=0.98f;
    float k  = 3.8f*res;
    float fb = k*st[3];
    float x  = in - fb;
    st[0]+=f*(fast_tanh(x)    -fast_tanh(st[0]));
    st[1]+=f*(fast_tanh(st[0])-fast_tanh(st[1]));
    st[2]+=f*(fast_tanh(st[1])-fast_tanh(st[2]));
    st[3]+=f*(fast_tanh(st[2])-fast_tanh(st[3]));
    return st[3];
}
// Wavefolder distortion
// Folds signal back when it exceeds threshold — creates rich harmonics.
// amount 0 = clean, 1 = heavy folding (4 folds)
static inline float wavefold(float in, float amount)
{
    if(amount < 0.01f) return in;
    // Drive the signal harder as amount increases (1x to 5x)
    float driven = in * (1.0f + amount * 4.0f);
    // Fold: reflect signal at ±1 boundary repeatedly
    // fold(x) = 1 - |((x+1) mod 2) - 1|  (triangle fold)
    float x = driven;
    // 2 passes of folding for richness
    for(int fold = 0; fold < 2; fold++) {
        x = x * 0.5f + 0.5f;           // shift to 0..1
        x = x - floorf(x);              // wrap to 0..1
        x = 1.0f - fabsf(2.0f*x - 1.0f); // triangle fold
        x = x * 2.0f - 1.0f;           // back to -1..1
    }
    // Blend dry/wet so at low amounts it stays subtle
    return in*(1.0f-amount) + x*amount;
}
