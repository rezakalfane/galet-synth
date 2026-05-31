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
#include <math.h>
#include <string.h>

using namespace daisy;
using namespace daisy::seed;

// ── Pin / I2C config ──────────────────────────────────────────────────────────
static constexpr Pin      SDA_PIN  = D12;
static constexpr Pin      SCL_PIN  = D11;
static constexpr uint8_t  MPR_ADDR = 0x5A;
static constexpr uint32_t HP       = 5;
static constexpr int      N_CH     = 12;

// ── Touch tuning ──────────────────────────────────────────────────────────────
static constexpr int32_t  TOUCH_THRESHOLD    = 10;
static constexpr int32_t  PRESSURE_MAX_REF[12] = {
    42,35,34,34,34,34,
    34,34,33,32,29,30
};
static constexpr int32_t  MIN_FINGER_SEP     = 4;
static constexpr uint32_t REBASELINE_IDLE_MS = 2000;
static constexpr int      REBASELINE_SAMPLES = 32;
static constexpr int32_t  MAX_POS_JUMP       = 200;

// LED slew at ~60Hz touch loop. Closer to 1.0 = smoother/slower fade.
// 0.0 = no smoothing (snap), 0.7 = ~80ms fade, 0.9 = ~250ms fade.
static constexpr float    LED_SMOOTH         = 0.45f;
// LED3 (software sigma-delta PWM on A0 / ADC0 / pin 22) max-brightness scale.
static constexpr float    LED3_INTENSITY     = 0.1f;
// Smoothing coefficient used for LED3 only when no finger is touching. Smaller
// than LED_SMOOTH = faster fade-out on lift. 0.0 = instant snap to 0.
static constexpr float    LED3_RELEASE_SMOOTH = 0.10f;

// ── Voices ──────────────────────────────────────────────────────────────────
// A "Voice" bundles everything that defines a sound: the oscillator stack
// (which pitches sound and at what level), the filter/drive character, and the
// pitch range finger-1 sweeps across. Presets live in the VOICES[] bank below;
// the active one is `g_voice_idx`, which the FSR-hold gesture cycles live. The
// audio callback snapshots it once per block.

// Oscillator shape — set per oscillator, so one voice can mix (e.g. saw root +
// square fifth + sine sub). SQUARE and SAW are naive (not band-limited) — they
// alias at high pitch, but on a low bass voice that just reads as rich harmonics
// for the filter to chew.
enum Waveform { WAVE_TRI, WAVE_SINE, WAVE_SQUARE, WAVE_SAW };

struct Voice {
    const char* name;

    // Pitch range. Finger-1 position 0–1000 maps exponentially across this.
    float freq_low;
    float freq_high;
    float log_freq_ratio;   // = ln(freq_high / freq_low), precomputed (logf isn't constexpr)

    // Oscillator stack. Each oscillator's pitch = fundamental * ratio, and each
    // has its own waveform. Set a level to 0 to silence an oscillator.
    float osc1_ratio, osc1_level;  Waveform osc1_wave;   // primary
    float osc2_ratio, osc2_level;  Waveform osc2_wave;   // second osc (see osc2_detune)
    float sub_ratio,  sub_level;   Waveform sub_wave;    // weight below
    float oct_ratio,  oct_level;   Waveform oct_wave;    // shimmer / chord top
    bool  osc2_detune;              // true  → finger-2 detune controls osc2 pitch (lead)
                                    // false → osc2 stays at osc2_ratio (e.g. a fixed 5th)

    // Filter character. Base cutoff = freq * cutoff_mult; pressure opens it
    // by up to cutoff_oct_max octaves on top.
    float cutoff_mult;
    float cutoff_oct_max;
    float resonance;

    // Drive — pressure-controlled waveshaper gain added on top of 1.0.
    float drive_max;

    // ── Extras (appended so existing field order stays stable) ────────────────
    float noise_level;     // white noise mixed into the oscillator stack (0 = none)
    float keytrack;        // 0..1 — how much the cutoff follows pitch (1 = full)
    float ringmod_ratio;   // ring-mod carrier pitch ratio (1.0 = unison; non-int = bell/metallic)
    float ringmod_max;     // max ring-mod wet from finger-2 pressure
    float fold_max;        // max wavefold amount from finger-2 pressure
    float attack_ms;       // amp envelope attack time
    float release_ms;      // amp envelope + filter-close release time
    float glide_ms;        // portamento (pitch glide) time

    // Pitch envelope — a fast downward pitch sweep at note onset (kick "boom",
    // zap, tom thump). Left at 0 (the default for voices that omit them) = off.
    float pitch_env_oct;   // octaves the pitch starts ABOVE the note at onset (0 = none)
    float pitch_env_ms;    // time the pitch sweep takes to settle to the note

    // Noise tone: 0 = full white noise ("shhh"), 1 = high-passed/bright ("tsss",
    // for hi-hats/cymbals). Default 0 leaves the noise white. See NOISE_HP_COEF.
    float noise_hp;
};

// Perfect-fifth frequency ratio = 2^(7/12).
static constexpr float RATIO_FIFTH = 1.49831f;

// ── Voice 0: the original glass-Moog lead ─────────────────────────────────────
//   F1 pos→pitch, F1 prs→cutoff+vibrato; F2 pos→detune, F2 prs→wavefold+ringmod.
static constexpr Voice VOICE_LEAD = {
    "Lead",
    50.0f, 300.0f, 1.79176f,   // 50–300 Hz, ln(6)
    1.0f, 0.50f, WAVE_TRI,   // osc1  — root
    1.0f, 0.22f, WAVE_TRI,   // osc2  — detune-controlled, ratio is just the base
    0.5f, 0.18f, WAVE_TRI,   // sub   — one octave below
    2.0f, 0.15f, WAVE_TRI,   // oct   — one octave above
    true,          // osc2 follows finger-2 detune
    0.3f, 12.0f, 0.75f,
    2.5f,
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.0f, 1.0f, 1.0f, 0.15f, 0.18f, 4.0f, 400.0f, 40.0f
};

// ── Voice 1: round & dark power-chord bass (root + 5th + octave + sub) ─────────
//   Same touch mappings, but osc2 is a fixed perfect fifth instead of a detune,
//   so every note sounds as a power chord. Lower range, gentler filter, less
//   drive → deep and round rather than bright and screaming.
static constexpr Voice VOICE_BASS = {
    "Bass",
    40.0f, 160.0f, 1.38629f,   // 40–160 Hz, ln(4) — sits under the lead
    1.0f,        0.45f, WAVE_TRI,   // osc1  — root
    RATIO_FIFTH, 0.30f, WAVE_TRI,   // osc2  — fixed perfect fifth (the power chord)
    0.5f,        0.34f, WAVE_TRI,   // sub   — extra weight for a round bottom
    2.0f,        0.20f, WAVE_TRI,   // oct   — top of the power chord
    false,                // osc2 is a fixed fifth, not finger-2 detune
    0.25f, 7.0f, 0.50f,   // darker: lower base cutoff, gentler sweep, less resonance
    1.0f,                 // less drive — keep it clean and round
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.0f, 1.0f, 1.0f, 0.12f, 0.12f, 6.0f, 300.0f, 25.0f
};

// ── Voice 2: open power-chord bass ────────────────────────────────────────────
//   Same chord stack as VOICE_BASS, but the filter starts brighter and sweeps
//   much further with pressure — light touch is still warm, hard press opens it
//   wide. A touch more resonance and drive for presence as it opens.
static constexpr Voice VOICE_BASS_OPEN = {
    "OpenBass",
    40.0f, 180.0f, 1.50408f,   // 40–180 Hz, ln(4.5) — a little more reach up top
    1.0f,        0.45f, WAVE_TRI,   // osc1  — root
    RATIO_FIFTH, 0.30f, WAVE_TRI,   // osc2  — fixed perfect fifth (the power chord)
    0.5f,        0.30f, WAVE_TRI,   // sub   — slightly less so the openness reads
    2.0f,        0.22f, WAVE_TRI,   // oct   — top of the power chord
    false,                // osc2 is a fixed fifth, not finger-2 detune
    0.5f, 10.0f, 0.65f,   // brighter base + wider pressure sweep + more resonance
    1.8f,                 // more drive — grit comes in as you press
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.0f, 1.0f, 1.0f, 0.15f, 0.15f, 5.0f, 350.0f, 25.0f
};

// ── Voice 3: rich power-chord bass with mixed waveforms + huge filter sweep ────
//   Mixed shapes for body: saw root (harmonic-rich), square fifth (hollow), a
//   clean sine sub (tight low end), saw octave on top. Starts dark, then
//   pressure opens the cutoff dramatically (13 octaves) with high resonance —
//   a vocal, acid-style "wow" as you press harder.
static constexpr Voice VOICE_BASS_RICH = {
    "RichBass",
    40.0f, 160.0f, 1.38629f,   // 40–160 Hz, ln(4)
    1.0f,        0.40f, WAVE_SAW,    // osc1  — saw root (rich harmonics)
    RATIO_FIFTH, 0.26f, WAVE_SQUARE, // osc2  — square fifth (hollow contrast)
    0.5f,        0.32f, WAVE_SINE,   // sub   — clean sine for a tight low end
    2.0f,        0.18f, WAVE_SAW,    // oct   — saw top of the power chord
    false,                // osc2 is a fixed fifth, not finger-2 detune
    0.2f, 13.0f, 0.82f,   // dark base + enormous pressure sweep + high resonance
    2.2f,                 // generous drive — grit blooms with pressure
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.0f, 1.0f, 1.0f, 0.20f, 0.20f, 3.0f, 250.0f, 18.0f   // snappy + fast acid glide
};

// ── Voice 4: clean sine organ / flute (a melodic, non-bass departure) ─────────
//   Pure sine oscillators stacked like organ drawbars: octave-down body, root,
//   octave-up air, plus a second osc tuned a hair sharp (1.005×) for a slow,
//   warm chorus shimmer instead of a chord. Higher register than the basses, a
//   mostly-open clean filter, and just a touch of drive so pressure adds gentle
//   harmonic warmth. Sines never alias, so it stays smooth all the way up.
static constexpr Voice VOICE_ORGAN = {
    "Organ",
    65.0f, 520.0f, 2.07944f,   // 65–520 Hz, ln(8) — a melodic keyboard register
    1.0f,    0.45f, WAVE_SINE,  // osc1  — root (8')
    1.005f,  0.40f, WAVE_SINE,  // osc2  — slightly sharp unison → slow chorus shimmer
    0.5f,    0.30f, WAVE_SINE,  // sub   — octave-down body (16')
    2.0f,    0.28f, WAVE_SINE,  // oct   — octave-up air (4')
    false,                      // osc2 is a fixed chorus detune, not finger-2 detune
    2.0f, 5.0f, 0.20f,          // mostly-open clean filter, gentle pressure lift, low res
    0.8f,                       // light drive — pressure adds subtle warmth, stays pure
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.03f, 1.0f, 1.0f, 0.05f, 0.0f, 8.0f, 120.0f, 8.0f   // breath chiff, fast on/off, no glide
};

// ── Voice 5: screaming aggressive lead ────────────────────────────────────────
//   Detuned sawtooth stack (root + a hair-sharp twin + octave) for a thick,
//   buzzing supersaw bite, pushed up into a cutting register. Heavy drive and a
//   very high filter resonance make the cutoff whistle and scream as pressure
//   sweeps it wide open. Finger-2 pressure piles on ring-mod + wavefold for
//   extra snarl. Loud and nasty — the output soft-clip keeps it from blowing up.
static constexpr Voice VOICE_SCREAM = {
    "Scream",
    90.0f, 720.0f, 2.07944f,   // 90–720 Hz, ln(8) — high, cutting register
    1.0f,    0.42f, WAVE_SAW,   // osc1  — saw root
    1.008f,  0.38f, WAVE_SAW,   // osc2  — slightly sharp twin → thick beating buzz
    0.5f,    0.20f, WAVE_SAW,   // sub   — a little weight underneath
    2.0f,    0.30f, WAVE_SAW,   // oct   — octave up for bright, cutting scream
    false,                      // osc2 is a fixed detune, not finger-2 controlled
    3.0f, 11.0f, 0.90f,         // bright base + wide sweep + near-self-osc resonance
    4.0f,                       // heavy drive — saturated and aggressive
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.05f, 1.0f, 2.5f, 0.40f, 0.30f, 2.0f, 200.0f, 35.0f // hiss + inharmonic metallic ring-mod
};

// ── Voice 6: very closed, dark sub bass ───────────────────────────────────────
//   The opposite of the scream: a deep, muffled power-chord bass. A clean sine
//   sub carries the weight under a mellow triangle root; the fifth and octave
//   are dialed right back. The filter sits down around the fundamental and only
//   creeps open a couple of octaves at full pressure, so it stays dark and
//   muted — felt more than heard. Low resonance, almost no drive.
static constexpr Voice VOICE_BASS_CLOSED = {
    "ClosedBass",
    35.0f, 140.0f, 1.38629f,   // 35–140 Hz, ln(4) — deep
    1.0f,        0.42f, WAVE_TRI,    // osc1  — mellow triangle root
    RATIO_FIFTH, 0.18f, WAVE_TRI,    // osc2  — fifth, dialed back
    0.5f,        0.44f, WAVE_SINE,   // sub   — clean sine carries the weight
    2.0f,        0.10f, WAVE_TRI,    // oct   — barely there, keep it dark
    false,                      // osc2 is a fixed fifth, not finger-2 detune
    1.0f, 2.0f, 0.30f,          // cutoff sits near the fundamental, tiny sweep, low res
    0.5f,                       // minimal drive — stays clean and round
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.0f, 0.6f, 1.0f, 0.0f, 0.0f, 10.0f, 500.0f, 30.0f   // soft, long tail, less keytrack, clean
};

// ── Voice 7: warm ensemble pad ────────────────────────────────────────────────
//   A lush, evolving pad. Two saws a hair apart (1.006×) give an ensemble/chorus
//   width, a soft triangle sub adds body, an airy sine octave sits on top. A
//   moderate, low-resonance filter keeps it warm rather than bright. The pad
//   character comes from the envelope: a slow ~500 ms swell in and a long ~2 s
//   tail, so notes bloom and linger. A whisper of noise adds air.
static constexpr Voice VOICE_PAD = {
    "Pad",
    70.0f, 560.0f, 2.07944f,   // 70–560 Hz, ln(8) — mid melodic register
    1.0f,    0.32f, WAVE_SAW,   // osc1  — saw root
    1.006f,  0.30f, WAVE_SAW,   // osc2  — detuned saw → lush ensemble width
    0.5f,    0.22f, WAVE_TRI,   // sub   — soft triangle body
    2.0f,    0.18f, WAVE_SINE,  // oct   — airy sine top
    false,                      // osc2 is a fixed chorus detune, not finger-2
    1.2f, 3.0f, 0.20f,          // warm & closed: low base cutoff, gentle sweep, low res
    0.6f,                       // gentle drive — stays smooth
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.015f, 1.0f, 1.0f, 0.0f, 0.08f, 2200.0f, 2000.0f, 80.0f   // long swell, long tail
};

// ── Voice 8: tom / percussion (noise-driven) ─────────────────────────────────
//   A drum, not a sustained note — PLAY IT BY TAPPING the glass. Noise over two
//   low sine modes (body + an inharmonic 1.6× ring) and a little sub thump. An
//   instant attack and a short ~150 ms release make each tap a percussive hit.
//   A closed, dark filter keeps it round and muffled (tom, not bright snare);
//   harder presses open it a touch. Finger-1 position tunes the drum.
static constexpr Voice VOICE_TOM = {
    "Tom",
    40.0f, 400.0f, 2.30259f,    // 40–400 Hz, ln(10) — finger-1 tunes from kick/tom up
    1.0f,    0.22f, WAVE_SINE,   // osc1  — body (lower mode)
    1.6f,    0.15f, WAVE_SINE,   // osc2  — upper mode (inharmonic ring)
    0.5f,    0.10f, WAVE_SINE,   // sub   — a little thump
    2.0f,    0.0f,  WAVE_SINE,   // oct   — off
    false,                       // osc2 is a fixed ratio, not finger-2 detune
    6.0f, 3.0f, 0.30f,           // dark/round tom, opened slightly for a touch more body
    1.5f,                        // drive — snap and saturation on hard hits
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.70f, 0.3f, 1.0f, 0.0f, 0.10f, 1.0f, 150.0f, 5.0f,  // noise-dominant, instant hit, short tail
    // pitch_env_oct, pitch_env_ms — tom "boww": start 1 oct up, drop in 80 ms
    1.0f, 80.0f
};

// ── Voice 9: kick drum (noise-driven) ────────────────────────────────────────
//   A deep, punchy thud — PLAY IT BY TAPPING. A low sine body with a sub-octave
//   for weight and a quiet octave "click" for beater attack; a whisper of noise
//   adds beater texture. Very closed dark filter + a little drive keep it round
//   and punchy. Instant attack, short tail. Finger-1 position tunes the kick.
//   A pitch envelope (start 2 oct up, drop in 45 ms) gives the classic "boom".
static constexpr Voice VOICE_KICK = {
    "Kick",
    30.0f, 80.0f, 0.98083f,     // 30–80 Hz, ln(2.67) — deep, position tunes the kick
    1.0f,    0.55f, WAVE_SINE,   // osc1  — deep sine body
    2.0f,    0.10f, WAVE_SINE,   // osc2  — octave "click" for attack punch
    0.5f,    0.15f, WAVE_SINE,   // sub   — sub-octave rumble
    2.0f,    0.0f,  WAVE_SINE,   // oct   — off
    false,                       // osc2 is a fixed ratio, not finger-2 detune
    3.0f, 2.0f, 0.20f,           // very closed/dark — round; tiny pressure sweep
    1.8f,                        // drive — punch and saturation
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.08f, 0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 160.0f, 5.0f,  // soft beater click, instant, short
    // pitch_env_oct, pitch_env_ms — the kick "boom": start 2 oct up, drop in 45 ms
    2.0f, 45.0f
};

// ── Voice 10: snare (noise-driven) ───────────────────────────────────────────
//   The bright, noisy cousin of the Tom — PLAY IT BY TAPPING. Broadband noise
//   (the "wires" rattle) dominates over two tonal modes (body + a 1.8× ring); a
//   wide-open filter keeps it crisp and cracking. Instant attack, short tail,
//   and a quick subtle pitch snap (0.7 oct in 30 ms) for the hit. Harder presses
//   open it further. Finger-1 position tunes the drum.
static constexpr Voice VOICE_SNARE = {
    "Snare",
    150.0f, 500.0f, 1.20397f,   // 150–500 Hz, ln(3.33) — snare tuning range
    1.0f,    0.18f, WAVE_SINE,   // osc1  — body (lower mode)
    1.8f,    0.14f, WAVE_TRI,    // osc2  — upper mode/ring (triangle = more harmonics)
    0.5f,    0.06f, WAVE_SINE,   // sub   — slight body weight
    2.0f,    0.0f,  WAVE_SINE,   // oct   — off
    false,                       // osc2 is a fixed ratio, not finger-2 detune
    18.0f, 4.0f, 0.30f,          // wide open & bright — the noise rattle cracks through
    1.5f,                        // drive — snap
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.85f, 0.3f, 1.0f, 0.0f, 0.10f, 1.0f, 140.0f, 5.0f,  // noise-dominant rattle, instant, short
    // pitch_env_oct, pitch_env_ms — quick subtle snap
    0.7f, 30.0f
};

// ── Voice 11: hi-hat (noise-driven) ──────────────────────────────────────────
//   A crisp "tss" — PLAY IT BY TAPPING. Almost pure high-frequency noise through
//   a wide-open filter, with three inharmonic square partials for metallic edge
//   and no low end. Instant attack and a very short ~55 ms tail = a tight closed
//   hat. Finger-1 position tunes the metallic pitch. (Raise release_ms for an
//   open-hat feel.)
static constexpr Voice VOICE_HIHAT = {
    "HiHat",
    4000.0f, 11000.0f, 1.01160f, // 4000–11000 Hz, ln(2.75) — metallic edge pitch
    1.0f,    0.10f, WAVE_SQUARE,  // osc1  — quiet metallic edge (the tss noise leads)
    1.68f,   0.08f, WAVE_SQUARE,  // osc2  — inharmonic metallic partial
    0.5f,    0.0f,  WAVE_SINE,    // sub   — off (no low end on a hat)
    2.0f,    0.06f, WAVE_SQUARE,  // oct   — high partial (2.0× keeps it under Nyquist)
    false,                        // osc2 is a fixed ratio, not finger-2 detune
    25.0f, 2.0f, 0.20f,           // wide open
    1.2f,                         // light drive
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.80f, 0.2f, 1.0f, 0.0f, 0.05f, 1.0f, 160.0f, 3.0f, // noise-led, instant, sizzly tail
    // pitch_env_oct, pitch_env_ms, noise_hp — high-passed bright "tsss"
    0.0f, 0.0f, 1.0f
};

// ── Voice 12: MultiVoice "Drums" ──────────────────────────────────────────────
//   A meta-voice: when selected, the slider splits into 4 equal zones, each
//   triggering one drum on tap, with the fine position snapping the pitch to an
//   interval (see MULTI_* below). This struct is just a placeholder so it lives
//   in the bank / FSR cycle and shows "Drums" in the header — the actual audio
//   per tap comes from the zone's real drum voice (g_active_voice). Kick-like so
//   it's harmless if briefly active before the first tap.
static constexpr Voice VOICE_DRUMS = {
    "Drums",
    30.0f, 80.0f, 0.98083f,
    1.0f, 0.55f, WAVE_SINE,  1.0f, 0.0f, WAVE_SINE,
    0.5f, 0.0f,  WAVE_SINE,  2.0f, 0.0f, WAVE_SINE,
    false,
    3.0f, 2.0f, 0.20f,
    1.0f,
    0.0f, 0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 160.0f, 5.0f,
    0.0f, 0.0f, 0.0f
};

// ── Voice bank ────────────────────────────────────────────────────────────────
// All voices, in cycle order. Holding the FSR pressed steps through these live
// (see the voice-switch gesture in the main loop). Edit the order here to taste.
static constexpr Voice VOICES[] = {
    VOICE_LEAD,
    VOICE_BASS,
    VOICE_BASS_OPEN,
    VOICE_BASS_RICH,
    VOICE_ORGAN,
    VOICE_SCREAM,
    VOICE_BASS_CLOSED,
    VOICE_PAD,
    VOICE_TOM,
    VOICE_KICK,
    VOICE_SNARE,
    VOICE_HIHAT,
    VOICE_DRUMS,   // MultiVoice — must stay last (MULTI_IDX = NUM_VOICES-1)
};
static constexpr int NUM_VOICES = (int)(sizeof(VOICES) / sizeof(VOICES[0]));

// ── MultiVoice "Drums" config ─────────────────────────────────────────────────
// MULTI_IDX is the bank slot of VOICE_DRUMS (kept last). When that voice is
// selected, the slider splits into 4 equal zones mapped to these drum voices
// (left→right), and the fine position within a zone snaps the pitch to one of
// MULTI_INTERVALS above the drum's base pitch (freq_low).
static constexpr int   MULTI_IDX        = NUM_VOICES - 1;
static constexpr int   MULTI_ZONES[4]   = { 9, 10, 8, 11 };  // Kick, Snare, Tom, HiHat (bank indices)
static constexpr float MULTI_INTERVALS[] = { 1.0f, 1.33484f, 1.49831f, 2.0f }; // root, 4th, 5th, octave
static constexpr int   MULTI_NINTERVALS = (int)(sizeof(MULTI_INTERVALS) / sizeof(MULTI_INTERVALS[0]));

// Selected voice index — set by the FSR-hold gesture, shown in the header.
// Change the initial value to pick the boot voice.
static volatile int g_voice_idx = 2;           // 2 = OpenBass

// Active voice index — what the audio engine actually plays. Normally tracks
// g_voice_idx; in MultiVoice mode the touch loop routes it per tap to a drum.
static volatile int g_active_voice = 2;        // matches the boot voice

// FSR voice-switch gesture: hold the FSR pressed (volume at/near 0) to cycle.
// First advance after FIRST_MS; while still held, keep advancing every REPEAT_MS.
static constexpr uint32_t VOICE_SWITCH_FIRST_MS  = 5000;
static constexpr uint32_t VOICE_SWITCH_REPEAT_MS = 2000;

// ── Musical mapping ───────────────────────────────────────────────────────────
// Set to false to disable all quantization (fully continuous pitch)
static constexpr bool QUANTIZE_ENABLED = false;
// Quantize to chromatic scale (all 12 semitones)
// Change to {0,2,4,5,7,9,11} for major scale, {0,2,3,5,7,8,10} for minor, etc.
static const int SCALE[] = {0,1,2,3,4,5,6,7,8,9,10,11}; // chromatic
static constexpr int SCALE_LEN = 12;

// ── MPR121 registers ──────────────────────────────────────────────────────────
static constexpr uint8_t REG_ELE0_LSB  = 0x04;
static constexpr uint8_t REG_MHDR=0x2B,REG_NHDR=0x2C,REG_NCLR=0x2D,REG_FDLR=0x2E;
static constexpr uint8_t REG_MHDF=0x2F,REG_NHDF=0x30,REG_NCLF=0x31,REG_FDLF=0x32;
static constexpr uint8_t REG_NHDT=0x33,REG_NCLT=0x34,REG_FDLT=0x35;
static constexpr uint8_t REG_DEBOUNCE=0x5B,REG_CONFIG1=0x5C,REG_CONFIG2=0x5D;
static constexpr uint8_t REG_ECR=0x5E,REG_SOFTRESET=0x80;

// ── Globals ───────────────────────────────────────────────────────────────────
DaisySeed hw;
GPIO      sda, scl;
GPIO      led3;  // software-PWM LED on A0 (ADC0 / D15 / pin 22)

// ── Bit-bang I2C ──────────────────────────────────────────────────────────────
inline void sda_high(){sda.Init(SDA_PIN,GPIO::Mode::INPUT, GPIO::Pull::PULLUP);}
inline void sda_low() {sda.Init(SDA_PIN,GPIO::Mode::OUTPUT,GPIO::Pull::NOPULL);sda.Write(false);}
inline void scl_high(){scl.Init(SCL_PIN,GPIO::Mode::INPUT, GPIO::Pull::PULLUP);}
inline void scl_low() {scl.Init(SCL_PIN,GPIO::Mode::OUTPUT,GPIO::Pull::NOPULL);scl.Write(false);}
inline void d(){System::DelayUs(HP);}

void i2c_start(){sda_high();d();scl_high();d();sda_low();d();scl_low();d();}
void i2c_stop() {sda_low();d();scl_high();d();sda_high();d();}

bool i2c_write_byte(uint8_t byte)
{
    for(int i=7;i>=0;i--){
        if(byte&(1<<i))sda_high();else sda_low();
        d();scl_high();d();scl_low();d();
    }
    sda_high();d();scl_high();d();
    bool ack=!sda.Read();
    scl_low();d();
    return ack;
}

uint8_t i2c_read_byte(bool ack)
{
    uint8_t b=0;sda_high();
    for(int i=7;i>=0;i--){d();scl_high();d();if(sda.Read())b|=(1<<i);scl_low();}
    if(ack)sda_low();else sda_high();
    d();scl_high();d();scl_low();d();sda_high();
    return b;
}

bool mpr_write(uint8_t reg,uint8_t val)
{
    i2c_start();
    if(!i2c_write_byte((MPR_ADDR<<1)|0)){i2c_stop();return false;}
    if(!i2c_write_byte(reg))            {i2c_stop();return false;}
    if(!i2c_write_byte(val))            {i2c_stop();return false;}
    i2c_stop();return true;
}

bool mpr_read(uint8_t reg,uint8_t* buf,uint8_t len)
{
    i2c_start();
    if(!i2c_write_byte((MPR_ADDR<<1)|0)){i2c_stop();return false;}
    if(!i2c_write_byte(reg))            {i2c_stop();return false;}
    i2c_start();
    if(!i2c_write_byte((MPR_ADDR<<1)|1)){i2c_stop();return false;}
    for(uint8_t i=0;i<len;i++) buf[i]=i2c_read_byte(i<len-1);
    i2c_stop();return true;
}

bool mpr_init()
{
    mpr_write(REG_SOFTRESET,0x63);System::Delay(10);
    mpr_write(REG_ECR,0x00);
    mpr_write(REG_MHDR,0x01);mpr_write(REG_NHDR,0x01);
    mpr_write(REG_NCLR,0x10);mpr_write(REG_FDLR,0x20);
    mpr_write(REG_MHDF,0x01);mpr_write(REG_NHDF,0x01);
    mpr_write(REG_NCLF,0x10);mpr_write(REG_FDLF,0x20);
    mpr_write(REG_NHDT,0x00);mpr_write(REG_NCLT,0x00);mpr_write(REG_FDLT,0x00);
    for(uint8_t ch=0;ch<12;ch++){mpr_write(0x41+ch*2,4);mpr_write(0x42+ch*2,2);}
    mpr_write(REG_DEBOUNCE,0x11);
    mpr_write(REG_CONFIG1,0xF0);  // FFI=34 | CDC=48uA
    mpr_write(REG_CONFIG2,0x4C);  // CDT=4us | SFI=18 | ESI=1ms
    mpr_write(REG_ECR,0x8C);
    System::Delay(100);
    uint8_t ecr=0;mpr_read(REG_ECR,&ecr,1);
    return(ecr==0x8C);
}

void read_electrodes(uint16_t* out)
{
    uint8_t buf[24];
    mpr_read(REG_ELE0_LSB,buf,24);
    for(int i=0;i<N_CH;i++)
        out[i]=(uint16_t)(buf[i*2]|((buf[i*2+1]&0x03)<<8));
}

void capture_baseline(uint16_t* bl)
{
    uint32_t acc[N_CH]={0};uint16_t tmp[N_CH];
    for(int s=0;s<REBASELINE_SAMPLES;s++){
        read_electrodes(tmp);
        for(int i=0;i<N_CH;i++)acc[i]+=tmp[i];
        System::Delay(5);
    }
    for(int i=0;i<N_CH;i++)bl[i]=(uint16_t)(acc[i]/REBASELINE_SAMPLES);
}

// ── Touch structs ─────────────────────────────────────────────────────────────
struct Finger { bool active; int32_t pos,pressure,peak_ch,peak_delta; };
struct TrackedFinger { bool alive; int32_t pos,pressure; int peak_ch; int32_t peak_delta; };
static TrackedFinger tracked[2]={};

int32_t centroid_window(int32_t* d,int s,int e,int32_t* pk_out,int* pkch_out)
{
    int32_t sw=0,swx=0,pk=0;int pkc=s;
    for(int i=s;i<=e;i++){
        int32_t w=d[i]>0?d[i]:0;
        sw+=w;swx+=w*i;
        if(w>pk){pk=w;pkc=i;}
    }
    if(pk_out)*pk_out=pk;if(pkch_out)*pkch_out=pkc;
    if(sw==0)return -1;
    int32_t p=(swx*1000)/(sw*(N_CH-1));
    if(p<0)p=0;if(p>1000)p=1000;
    return p;
}

int32_t pressure_pct(int32_t pk, int ch){
    int32_t range = PRESSURE_MAX_REF[ch] - TOUCH_THRESHOLD;
    int32_t raw   = pk - TOUCH_THRESHOLD;
    if(raw <= 0)     return 0;
    if(raw >= range) return 100;
    // Clamp raw strictly before sqrt to prevent Newton overshoot
    if(raw > range) raw = range;
    int32_t scaled = raw * 10000 / range; // 0..10000
    if(scaled > 10000) scaled = 10000;
    // Integer sqrt via Newton
    int32_t s = scaled / 2 + 1;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    // Hard clamp output — Newton can land at 101 due to rounding
    if(s < 0)   s = 0;
    if(s > 100) s = 100;
    return s;
}

int detect_raw(int32_t* delta,Finger out[2])
{
    out[0].active=out[1].active=false;
    int pka=0;int32_t da=0;
    for(int i=0;i<N_CH;i++)if(delta[i]>da){da=delta[i];pka=i;}
    if(da<TOUCH_THRESHOLD)return 0;
    int ws=pka-2;if(ws<0)ws=0;int we=pka+2;if(we>=N_CH)we=N_CH-1;
    int32_t pa;int pca;
    int32_t posa=centroid_window(delta,ws,we,&pa,&pca);
    out[0]={true,posa,pressure_pct(pa,pca),pca,pa};

    int32_t d2[N_CH];
    int ss=pka-2;if(ss<0)ss=0;int se=pka+2;if(se>=N_CH)se=N_CH-1;
    for(int i=0;i<N_CH;i++)d2[i]=(i>=ss&&i<=se)?0:delta[i];
    int pkb=0;int32_t db=0;
    for(int i=0;i<N_CH;i++)if(d2[i]>db){db=d2[i];pkb=i;}
    int sep=pkb-pka;if(sep<0)sep=-sep;
    if(db<TOUCH_THRESHOLD||sep<MIN_FINGER_SEP)return 1;
    int ws2=pkb-2;if(ws2<0)ws2=0;int we2=pkb+2;if(we2>=N_CH)we2=N_CH-1;
    int32_t pb;int pcb;
    int32_t posb=centroid_window(d2,ws2,we2,&pb,&pcb);
    out[1]={true,posb,pressure_pct(pb,pcb),pcb,pb};
    return 2;
}

void update_tracked(Finger* raw,int n)
{
    bool ra[2]={false,false};
    for(int slot=0;slot<2;slot++){
        if(!tracked[slot].alive)continue;
        int32_t best=MAX_POS_JUMP+1;int br=-1;
        for(int r=0;r<n;r++){
            if(ra[r])continue;
            int32_t c=raw[r].pos-tracked[slot].pos;if(c<0)c=-c;
            if(c<best){best=c;br=r;}
        }
        if(br>=0){
            tracked[slot].pos=raw[br].pos;tracked[slot].pressure=raw[br].pressure;
            tracked[slot].peak_ch=raw[br].peak_ch;tracked[slot].peak_delta=raw[br].peak_delta;
            ra[br]=true;
        } else tracked[slot].alive=false;
    }
    for(int r=0;r<n;r++){
        if(ra[r])continue;
        for(int slot=0;slot<2;slot++){
            if(!tracked[slot].alive){
                tracked[slot]={true,raw[r].pos,raw[r].pressure,raw[r].peak_ch,raw[r].peak_delta};
                ra[r]=true;break;
            }
        }
    }
}

// ── Maths helpers ─────────────────────────────────────────────────────────────
static inline float clampf(float v,float lo,float hi){
    return v<lo?lo:v>hi?hi:v;
}

// Fast tanh approximation (Padé)
static inline float fast_tanh(float x)
{
    if(x> 3.0f)return  1.0f;
    if(x<-3.0f)return -1.0f;
    float x2=x*x;
    return x*(27.0f+x2)/(27.0f+9.0f*x2);
}

// ── Note quantizer ────────────────────────────────────────────────────────────
// Returns the nearest MIDI note in SCALE[] for a raw float MIDI pitch.
float quantize_midi(float midi)
{
    int root = (int)midi;
    int pc   = root % 12;
    int oct  = root / 12;
    // find nearest scale degree
    int best_deg=0, best_dist=127;
    for(int i=0;i<SCALE_LEN;i++){
        int d=SCALE[i]-pc; if(d<0)d=-d;
        int d2=12-d; if(d2<d)d=d2;
        if(d<best_dist){best_dist=d;best_deg=SCALE[i];}
    }
    float q=(float)(oct*12+best_deg);
    // fractional semitone from the quantized note (for slide expression)
    float frac=midi-(float)root;
    return q+frac;
}

// ── Audio state (written by touch loop, read by audio callback) ───────────────
// All volatile — touched by both cores / interrupt context
volatile float g_target_freq    = 65.41f;
volatile float g_target_cutoff  = 100.0f;  // starts dark, pressure opens it
volatile float g_target_detune  = 0.0f;     // semitones osc2 offset
volatile float g_target_drive   = 1.0f;     // waveshaper input gain
volatile float g_vibrato_depth  = 0.0f;     // 0–1, LFO depth
volatile float g_bitcrush       = 0.0f;     // 0–1, amount of crush
volatile float g_ringmod        = 0.0f;     // 0–1, ring mod wet
volatile bool  g_finger_on      = false;
volatile float g_amp_target     = 0.0f;
volatile float g_led3_duty      = 0.0f;     // 0–1, sigma-delta'd to LED3 in audio cb
volatile float g_master_vol     = 1.0f;     // 0–1, master volume from FSR on A1

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

// Moog ladder filter state
static float s_flt[4]    = {0,0,0,0};

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

// Moog 4-pole ladder
static inline float moog(float in, float cutoff, float res, float sr)
{
    float f = 2.0f*cutoff/sr;
    if(f>0.98f)f=0.98f;
    float k  = 3.8f*res;
    float fb = k*s_flt[3];
    float x  = in - fb;
    s_flt[0]+=f*(fast_tanh(x)      -fast_tanh(s_flt[0]));
    s_flt[1]+=f*(fast_tanh(s_flt[0])-fast_tanh(s_flt[1]));
    s_flt[2]+=f*(fast_tanh(s_flt[1])-fast_tanh(s_flt[2]));
    s_flt[3]+=f*(fast_tanh(s_flt[2])-fast_tanh(s_flt[3]));
    return s_flt[3];
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
    float sr     = hw.AudioSampleRate();

    // Snapshot the active voice once per block, so a mid-block switch from the
    // touch loop can't tear fields across the block. All VOICE.* reads below
    // resolve against this reference. g_active_voice tracks g_voice_idx except in
    // MultiVoice mode, where the touch loop routes it per tap to a drum.
    int vi = g_active_voice;
    if(vi < 0 || vi >= NUM_VOICES) vi = 0;
    const Voice& VOICE = VOICES[vi];

    // Per-voice envelope/glide coefficients (expf isn't constexpr). Recompute
    // only when the active voice changes.
    static int   s_coeff_vi = -1;
    static float s_glide_c, s_atk_c, s_rel_c;
    static float s_penv_c;      // pitch-env decay coefficient
    static float s_penv_start;  // pitch multiplier at onset = 2^pitch_env_oct
    if(s_coeff_vi != vi){
        s_glide_c    = ms_to_coeff(VOICE.glide_ms,     sr);
        s_atk_c      = ms_to_coeff(VOICE.attack_ms,    sr);
        s_rel_c      = ms_to_coeff(VOICE.release_ms,   sr);
        s_penv_c     = ms_to_coeff(VOICE.pitch_env_ms, sr);
        s_penv_start = exp2f(VOICE.pitch_env_oct);   // 1.0 when pitch_env_oct == 0
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

    // Pitch-envelope trigger: on a note onset (amp target rising from silence),
    // jump the pitch multiplier up; it decays back to 1.0 over pitch_env_ms.
    static float s_pmult     = 1.0f;
    static float s_prev_tamp = 0.0f;
    if(tamp > 0.001f && s_prev_tamp <= 0.001f) s_pmult = s_penv_start;
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
        // Asymmetric envelope: per-voice attack (rising) and release (falling)
        float slew_amp = (tamp > s_amp) ? s_atk_c : s_rel_c;
        s_amp = s_amp * slew_amp + tamp * (1.0f - slew_amp);
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

        // Soft clip output
        float sample = fast_tanh(crushed * 1.3f) * s_amp * s_master_vol;

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
    for(int slot=0;slot<2;slot++){
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

    // Power-on grace period so the lid can be closed before baseline capture.
    System::Delay(5000);

    // ── I2C pins + MPR121 init BEFORE audio starts ────────────────────────────
    // The audio ISR firing during bit-bang I2C corrupts I2C timing.
    // Do all sensor work first, then start the audio engine.
    sda_high();
    scl_high();

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
    uint16_t           fsr_max       = 55000;
    float              fsr_range_inv = 1.0f / (float)(fsr_max - FSR_MIN);

    auto commit_fsr_avg = [&](uint32_t acc, int n) {
        if(n == 0) return;
        uint16_t avg = (uint16_t)(acc / n);
        // Sanity floor — without enough headroom over FSR_MIN the volume curve
        // becomes hyper-sensitive to noise.
        if(avg < FSR_MIN + 500) avg = FSR_MIN + 500;
        fsr_max       = avg;
        fsr_range_inv = 1.0f / (float)(fsr_max - FSR_MIN);
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
            hw.dac.WriteValue(DacHandle::Channel::ONE, (uint16_t)(led1 * 4095.0f));
            hw.dac.WriteValue(DacHandle::Channel::TWO, (uint16_t)(led2 * 4095.0f));
            g_led3_duty = led3;

            // FSR calibration window: first 2 s only.
            if(!calibrated) {
                acc += hw.adc.Get(0);
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

    hw.PrintLine("Glass Moog Synth");
    hw.PrintLine("================");
    hw.PrintLine("Voice %d/%d: %s  (%d-%dHz)",
                 g_voice_idx + 1, NUM_VOICES,
                 VOICE.name, (int)VOICE.freq_low, (int)VOICE.freq_high);
    hw.PrintLine("F1 pos=pitch  F1 prs=cutoff+vibrato");
    if(VOICE.osc2_detune)
        hw.PrintLine("F2 pos=detune F2 prs=wavefold+ringmod");
    else
        hw.PrintLine("F2 prs=wavefold+ringmod (osc2=fixed)");
    hw.PrintLine("Hold FSR pressed 5s to change voice (then every 2s)");
    hw.PrintLine("MPR121 OK — baseline captured");
    hw.PrintLine("Ready.\n");

    uint16_t data[N_CH];
    int32_t  delta[N_CH];
    char     bar[32], pos_bar[52];
    Finger   raw[2];

    uint32_t idle_since      = System::GetNow();
    uint32_t last_touch_ms   = System::GetNow();
    bool     f1_was_on       = false;
    uint32_t f1_last_seen_ms = 0;
    static constexpr uint32_t F1_REQUANTIZE_MS = 200; // re-quantize only after this long off
    float    f1_midi_base    = 60.0f; // last quantized note
    bool     f1_sliding      = false; // true after first touch settles
    float    multi_freq      = 60.0f; // MultiVoice: pitch latched at tap onset

    // Per-frame display counter (print every N frames to reduce serial spam)
    int print_every = 1;
    int frame_count = 0;

    // Smoothed LED brightness state (0..1), slewed toward per-frame targets.
    // LED1 = DAC1 (pos 0 end), LED2 = DAC2 (middle), LED3 = PWM A0 (pos 1000 end).
    float s_led1 = 0.0f, s_led2 = 0.0f, s_led3 = 0.0f;

    // FSR voice-switch gesture state.
    uint32_t fsr_hold_start  = 0;   // when the current press began (0 = released)
    uint32_t fsr_next_switch = 0;   // timestamp of the next allowed switch

    // Flash all three LEDs `count` times to confirm a voice change (count = voice
    // number). Blocks the touch loop briefly — fine, we're mid-gesture and silent.
    auto flash_voice_leds = [&](int count){
        for(int f = 0; f < count; f++){
            hw.dac.WriteValue(DacHandle::Channel::ONE, 4095);
            hw.dac.WriteValue(DacHandle::Channel::TWO, 4095);
            g_led3_duty = LED3_INTENSITY;
            System::Delay(110);
            hw.dac.WriteValue(DacHandle::Channel::ONE, 0);
            hw.dac.WriteValue(DacHandle::Channel::TWO, 0);
            g_led3_duty = 0.0f;
            System::Delay(140);
        }
    };

    while(true)
    {
        // ── FSR → master volume (no pressure = 100%, press attenuates) ───────
        uint16_t fsr_raw = hw.adc.Get(0);
        float    vol;
        if(fsr_raw <= FSR_MIN)      vol = 0.0f;
        else if(fsr_raw >= fsr_max) vol = 1.0f;
        else                        vol = (float)(fsr_raw - FSR_MIN) * fsr_range_inv;
        g_master_vol = vol;

        // ── FSR-hold voice-switch gesture ───────────────────────────────────
        // Pressing the FSR to (near) silence and holding cycles the voice: first
        // advance after 5 s, then every 2 s while still held. LEDs flash the new
        // voice number. Releasing re-arms the 5 s wait.
        bool fsr_pressed = (fsr_raw <= FSR_MIN);   // "low value" = at the mute floor
        if(!fsr_pressed){
            fsr_hold_start = 0;                    // released → disarm
        } else {
            uint32_t now = System::GetNow();
            if(fsr_hold_start == 0){
                fsr_hold_start  = now;
                fsr_next_switch = now + VOICE_SWITCH_FIRST_MS;
            }
            // Holding the FSR means no glass touch — freeze the idle/rebaseline
            // timers so they don't fire mid-gesture.
            idle_since    = now;
            last_touch_ms = now;
            if((int32_t)(now - fsr_next_switch) >= 0){
                g_voice_idx = (g_voice_idx + 1) % NUM_VOICES;
                hw.PrintLine("[voice %d/%d] %s",
                             g_voice_idx + 1, NUM_VOICES, VOICES[g_voice_idx].name);
                flash_voice_leds(g_voice_idx + 1);            // count = voice number
                fsr_next_switch = System::GetNow() + VOICE_SWITCH_REPEAT_MS; // re-time after flash
            }
        }

        read_electrodes(data);

        int32_t max_delta=1;
        for(int i=0;i<N_CH;i++){
            delta[i]=(int32_t)baseline[i]-(int32_t)data[i];
            if(delta[i]<0)delta[i]=0;
            if(delta[i]>max_delta)max_delta=delta[i];
        }

        int n_raw=detect_raw(delta,raw);
        bool touching=(n_raw>0);

        // ── Auto-rebaseline ───────────────────────────────────────────────────
        if(touching){
            idle_since=System::GetNow();
        } else {
            if(System::GetNow()-idle_since>=REBASELINE_IDLE_MS){
                tracked[0].alive=tracked[1].alive=false;
                capture_baseline(baseline);
                hw.PrintLine("[rebaseline]");
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
            hw.PrintLine("[fsr recal] fsr_max=%d", (int)fsr_max);
        }

        update_tracked(raw,n_raw);

        // ── Finger 1 → Pitch + Filter cutoff + Vibrato ───────────────────────
        bool f1_on = tracked[0].alive;
        if(f1_on) f1_last_seen_ms = System::GetNow();
        bool f1_fresh = !f1_was_on && (System::GetNow() - f1_last_seen_ms >= F1_REQUANTIZE_MS);

        // ── MultiVoice routing ───────────────────────────────────────────────
        // In Drums mode each fresh tap latches a drum (by slider zone) and a
        // pitch (interval by fine position within the zone); both hold for the
        // whole tap. Otherwise the active voice just tracks the selected one.
        bool multi = (g_voice_idx == MULTI_IDX);
        if(!multi){
            g_active_voice = g_voice_idx;
        } else if(f1_on && !f1_was_on){
            float p    = clampf((float)tracked[0].pos / 1000.0f, 0.0f, 1.0f);
            int   zone = (int)(p * 4.0f); if(zone > 3) zone = 3;
            g_active_voice = MULTI_ZONES[zone];
            float subp = p * 4.0f - (float)zone;                 // 0..1 within the zone
            int   step = (int)(subp * (float)MULTI_NINTERVALS);
            if(step >= MULTI_NINTERVALS) step = MULTI_NINTERVALS - 1;
            multi_freq = VOICES[g_active_voice].freq_low * MULTI_INTERVALS[step];
        }

        // Bind the active voice (a drum in MultiVoice mode) for the rest of the frame.
        const Voice& VOICE = VOICES[g_active_voice];

        if(f1_on)
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

            if(multi)
            {
                // MultiVoice: use the pitch latched at tap onset (zone interval).
                freq = multi_freq;
                f1_sliding = false;
            }
            else if(QUANTIZE_ENABLED && f1_fresh)
            {
                // Touch-down: snap to nearest chromatic note
                f1_midi_base = quantize_midi(midi_raw);
                // Convert quantized MIDI back to Hz
                float qe = (f1_midi_base - 69.0f) * 0.05776f;
                freq = clampf(440.0f*(1.0f+qe*(1.0f+qe*(0.5f+qe*0.1667f))), VOICE.freq_low, VOICE.freq_high);
                f1_sliding = false;
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

            g_target_freq    = freq;
            g_target_cutoff  = cutoff;
            g_vibrato_depth  = vib;
            g_target_drive   = drive;
            g_finger_on      = true;
            g_amp_target     = 0.72f;

        }
        else
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
        hw.dac.WriteValue(DacHandle::Channel::ONE, (uint16_t)(s_led1 * 4095.0f));
        hw.dac.WriteValue(DacHandle::Channel::TWO, (uint16_t)(s_led2 * 4095.0f));
        g_led3_duty = s_led3;  // audio callback sigma-delta's this onto A0

        f1_was_on = f1_on;

        // ── Finger 2 → Detune + Bitcrush + Ring mod ──────────────────────────
        if(tracked[1].alive)
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

        // ── Serial display ────────────────────────────────────────────────────
        frame_count++;
        if(frame_count < print_every){ System::Delay(10); continue; }
        frame_count = 0;

        if(g_voice_idx == MULTI_IDX)
            hw.PrintLine("VOICE %d/%d  Drums [Kick|Snare|Tom|Hat]  -> %s",
                g_voice_idx + 1, NUM_VOICES, VOICE.name);
        else
            hw.PrintLine("VOICE %d/%d  %s  (%d-%dHz)",
                g_voice_idx + 1, NUM_VOICES, VOICE.name,
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

        System::Delay(60);
    }
}
