// Host unit tests for the pure (hardware-free) modules: dsp.h, voice.h, touch.
// Build & run:  make -C test     (no Daisy, no gtest — plain assertions)
#include <cstdio>
#include <cmath>

#include "config.h"
#include "dsp.h"
#include "voice.h"
#include "touch.h"

static int g_total = 0, g_fail = 0;
#define CHECK(cond) do { ++g_total; if(!(cond)){ ++g_fail; \
    printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while(0)
#define NEAR(a,b,eps) (std::fabs((double)(a)-(double)(b)) <= (eps))

// ── dsp.h ─────────────────────────────────────────────────────────────────────
static void test_dsp()
{
    CHECK(clampf(5.0f, 0.0f, 1.0f) == 1.0f);
    CHECK(clampf(-3.0f, 0.0f, 1.0f) == 0.0f);
    CHECK(clampf(0.5f, 0.0f, 1.0f) == 0.5f);

    CHECK(NEAR(fast_tanh(0.0f), 0.0f, 1e-4));
    CHECK(fast_tanh(10.0f) == 1.0f);
    CHECK(fast_tanh(-10.0f) == -1.0f);
    for(float x = -4; x <= 4; x += 0.25f) CHECK(std::fabs(fast_tanh(x)) <= 1.0f);

    // Triangle / saw / square known points + range.
    CHECK(NEAR(tri(0.0f), -1.0f, 1e-5));
    CHECK(NEAR(tri(0.5f),  1.0f, 1e-5));
    CHECK(NEAR(osc_saw(0.0f), -1.0f, 1e-5));
    CHECK(osc_square(0.25f) ==  1.0f);
    CHECK(osc_square(0.75f) == -1.0f);
    for(float p = 0; p < 1.0f; p += 0.05f){
        CHECK(std::fabs(tri(p))        <= 1.0001f);
        CHECK(std::fabs(osc_sine(p))   <= 1.0001f);
    }
    // osc() dispatches to the right shape.
    CHECK(osc(0.3f, WAVE_TRI)    == tri(0.3f));
    CHECK(osc(0.3f, WAVE_SAW)    == osc_saw(0.3f));
    CHECK(osc(0.3f, WAVE_SQUARE) == osc_square(0.3f));

    // Parabolic sine approximation tracks sin(2*pi*ph) within ~0.02.
    CHECK(NEAR(osc_sine(0.0f),  0.0f, 0.02));
    CHECK(NEAR(osc_sine(0.25f), 1.0f, 0.02));
    CHECK(NEAR(osc_sine(0.5f),  0.0f, 0.02));
    CHECK(NEAR(osc_sine(0.75f),-1.0f, 0.02));

    // ms_to_coeff: 0 = instant; longer time = coefficient closer to 1.
    CHECK(ms_to_coeff(0.0f, 48000.0f) == 0.0f);
    float c10  = ms_to_coeff(10.0f,  48000.0f);
    float c100 = ms_to_coeff(100.0f, 48000.0f);
    CHECK(c10 > 0.0f && c10 < 1.0f);
    CHECK(c100 > c10 && c100 < 1.0f);
}

// ── touch ─────────────────────────────────────────────────────────────────────
static void zero(int32_t* d){ for(int i = 0; i < N_CH; i++) d[i] = 0; }

static void test_touch()
{
    // pressure_pct: 0 below threshold, 100 at/above the per-electrode max, and
    // non-decreasing in between. NOTE: the integer Newton-sqrt converges poorly,
    // so the curve saturates to 100 well before the max — known quirk, kept as-is
    // (every pressure mapping was tuned around it); we just assert the envelope.
    CHECK(pressure_pct(TOUCH_THRESHOLD - 1, 5) == 0);
    CHECK(pressure_pct(TOUCH_THRESHOLD + 1, 5) >  0);
    CHECK(pressure_pct(PRESSURE_MAX_REF[5],  5) == 100);
    CHECK(pressure_pct(PRESSURE_MAX_REF[5] + 50, 5) == 100);   // clamped
    int prev = 0;
    for(int32_t pk = TOUCH_THRESHOLD; pk <= PRESSURE_MAX_REF[5]; pk++){
        int v = pressure_pct(pk, 5);
        CHECK(v >= 0 && v <= 100);
        CHECK(v >= prev);              // non-decreasing
        prev = v;
    }

    // centroid_window: a single peak at index 6 → position near 6/(N_CH-1).
    int32_t d[N_CH]; zero(d); d[6] = 80;
    int32_t pk; int pkc;
    int32_t pos = centroid_window(d, 0, N_CH - 1, &pk, &pkc);
    CHECK(pkc == 6 && pk == 80);
    CHECK(NEAR(pos, 6 * 1000 / (N_CH - 1), 60));

    Finger raw[MAX_FINGERS];
    // No touch.
    zero(d);
    CHECK(detect_raw(d, raw, MAX_FINGERS) == 0);
    // One finger.
    zero(d); d[5] = 60;
    CHECK(detect_raw(d, raw, MAX_FINGERS) == 1);
    CHECK(raw[0].active);
    // Two well-separated fingers.
    zero(d); d[2] = 60; d[9] = 55;
    CHECK(detect_raw(d, raw, MAX_FINGERS) == 2);
    // A wide single blob (peaks within MIN_FINGER_SEP) stays one finger.
    zero(d); d[5] = 60; d[6] = 50;
    CHECK(detect_raw(d, raw, MAX_FINGERS) == 1);
    // Three separated fingers.
    zero(d); d[0] = 50; d[4] = 50; d[8] = 50;
    CHECK(detect_raw(d, raw, MAX_FINGERS) == 3);

    // update_tracked: a fresh raw finger becomes alive and keeps identity.
    for(int i = 0; i < MAX_FINGERS; i++) tracked[i].alive = false;
    zero(d); d[3] = 70;
    int n = detect_raw(d, raw, MAX_FINGERS);
    update_tracked(raw, n);
    int alive = 0, slot = -1;
    for(int i = 0; i < MAX_FINGERS; i++) if(tracked[i].alive){ alive++; slot = i; }
    CHECK(alive == 1);
    int32_t p0 = tracked[slot].pos;
    // Same finger next frame (tiny move) → same slot, no new finger.
    zero(d); d[3] = 72;
    n = detect_raw(d, raw, MAX_FINGERS);
    update_tracked(raw, n);
    CHECK(tracked[slot].alive);
    CHECK(NEAR(tracked[slot].pos, p0, 100));
    // Lift → slot dies.
    zero(d);
    n = detect_raw(d, raw, MAX_FINGERS);
    update_tracked(raw, n);
    CHECK(!tracked[slot].alive);
}

// ── voice ─────────────────────────────────────────────────────────────────────
static void test_voice()
{
    CHECK(NUM_VOICES == 15);
    CHECK(MULTI_IDX == NUM_VOICES - 1);

    // 4 raw drums are no_cycle → 11 cyclable (10 instruments + Drums).
    int cyclable = 0;
    for(int i = 0; i < NUM_VOICES; i++) if(!VOICES[i].no_cycle) cyclable++;
    CHECK(cyclable == 11);
    CHECK(cycle_total() == cyclable);

    // cycle_next always lands on a cyclable voice, and wrapping visits exactly
    // `cycle_total` distinct voices before repeating.
    for(int i = 0; i < NUM_VOICES; i++) CHECK(!VOICES[cycle_next(i)].no_cycle);
    int start = 0; while(VOICES[start].no_cycle) start++;
    int idx = start, steps = 0;
    do { idx = cycle_next(idx); steps++; } while(idx != start && steps <= NUM_VOICES + 1);
    CHECK(steps == cycle_total());

    // cycle_pos is 1-based and within range for cyclable voices.
    for(int i = 0; i < NUM_VOICES; i++) if(!VOICES[i].no_cycle){
        int p = cycle_pos(i);
        CHECK(p >= 1 && p <= cycle_total());
    }

    // MultiVoice drum zones point at real, no_cycle drum voices.
    for(int z = 0; z < 4; z++){
        CHECK(MULTI_ZONES[z] >= 0 && MULTI_ZONES[z] < NUM_VOICES);
        CHECK(VOICES[MULTI_ZONES[z]].no_cycle);
    }
    CHECK(MULTI_INTERVALS[0] == 1.0f);
    CHECK(MULTI_NINTERVALS == 4);

    // Reverb / delay sends are 0..1 aux-send fractions; at least one voice (the
    // Organ) currently feeds each shared effect.
    int reverb_voices = 0, delay_voices = 0;
    for(int i = 0; i < NUM_VOICES; i++){
        CHECK(VOICES[i].reverb_send >= 0.0f && VOICES[i].reverb_send <= 1.0f);
        CHECK(VOICES[i].delay_send  >= 0.0f && VOICES[i].delay_send  <= 1.0f);
        if(VOICES[i].reverb_send > 0.0f) reverb_voices++;
        if(VOICES[i].delay_send  > 0.0f) delay_voices++;
    }
    CHECK(reverb_voices >= 1);
    CHECK(delay_voices  >= 1);

    // Chords: the two SH-101 twins build diatonic triads, and a chord voice must
    // quantize to a musical (non-chromatic) scale or the triad degenerates to a
    // cluster.
    int chord_voices = 0;
    for(int i = 0; i < NUM_VOICES; i++) if(VOICES[i].chords){
        chord_voices++;
        CHECK(VOICES[i].quantize);
        CHECK(VOICES[i].scale != nullptr && VOICES[i].scale_len > 0);
        CHECK(VOICES[i].scale != SCALE_CHROMATIC);
        CHECK(VOICES[i].chord_level > 0.0f);   // else the added notes are silent
        CHECK(VOICES[i].chord_spread >= 0);
    }
    CHECK(chord_voices == 2);
}

// ── diatonic_triad ──────────────────────────────────────────────────────────
static void test_chords()
{
    const int n_min = (int)(sizeof(SCALE_MINOR) / sizeof(SCALE_MINOR[0]));
    const int n_maj = (int)(sizeof(SCALE_MAJOR) / sizeof(SCALE_MAJOR[0]));
    int t, f;

    // Minor scale {0,2,3,5,7,8,10}: i = minor triad, ii = diminished, VII = major.
    diatonic_triad(0,  SCALE_MINOR, n_min, t, f); CHECK(t == 3 && f == 7); // i
    diatonic_triad(2,  SCALE_MINOR, n_min, t, f); CHECK(t == 3 && f == 6); // ii°
    diatonic_triad(10, SCALE_MINOR, n_min, t, f); CHECK(t == 4 && f == 7); // VII (wraps)

    // Major scale {0,2,4,5,7,9,11}: I = major, ii = minor.
    diatonic_triad(0,  SCALE_MAJOR, n_maj, t, f); CHECK(t == 4 && f == 7); // I
    diatonic_triad(2,  SCALE_MAJOR, n_maj, t, f); CHECK(t == 3 && f == 7); // ii

    // A pitch class not in the scale snaps to the nearest degree first.
    diatonic_triad(1,  SCALE_MINOR, n_min, t, f); CHECK(t == 3 && f == 7); // → i

    // No scale → fixed major triad fallback; octave wrap handled for any input.
    diatonic_triad(5,  nullptr, 0, t, f);         CHECK(t == 4 && f == 7);
    diatonic_triad(25, SCALE_MINOR, n_min, t, f); CHECK(t == 3 && f == 7); // pc 1 → i
}

int main()
{
    printf("GaletSynth unit tests\n");
    test_dsp();   printf("  dsp    done\n");
    test_touch(); printf("  touch  done\n");
    test_voice(); printf("  voice  done\n");
    test_chords();printf("  chords done\n");
    printf("%s — %d checks, %d failures\n", g_fail ? "FAILED" : "PASSED", g_total, g_fail);
    return g_fail ? 1 : 0;
}
