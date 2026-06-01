#pragma once
// The voice model: the Voice struct, all presets, the VOICES[] bank, the
// Drums MultiVoice config, and the FSR-cycle helpers.
#include <cstdint>
#include "dsp.h"   // Waveform

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

    // FSR-gesture cycling: true = skip this voice in the cycle (still reachable
    // as the boot voice or via the Drums MultiVoice). Default false = in cycle.
    // The raw drums set this so the gesture cycles instruments + Drums only.
    bool no_cycle;

    // Velocity sensitivity 0..1: how much the onset hit strength (peak pressure
    // in the first few ms) scales loudness. 0 = fixed loudness (default — the
    // sustained instruments); ~0.85 on drums for accents/ghost notes. Loudness
    // floor = (1 - vel_sens), so soft taps stay audible.
    float vel_sens;

    // Re-attack speed on hold: minimum ms between bounce-retriggers (the cap on
    // how fast a held finger can repeat by pulsing pressure). SMALLER = faster
    // repeats allowed; LARGER = slower. 0 = off (only a full lift + re-tap
    // retriggers). A full lift always retriggers regardless. Default 0 keeps the
    // melodic voices free of accidental re-articulation; drums set a value.
    float retrig_ms;
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
    3.0f, 3.0f, 0.30f,           // tom — low 1-pole cutoff for a dark, round body
    1.5f,                        // drive — snap and saturation on hard hits
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.70f, 0.3f, 1.0f, 0.0f, 0.10f, 1.0f, 150.0f, 5.0f,  // noise-dominant, instant hit, short tail
    // pitch_env_oct, pitch_env_ms — tom "boww": start 1 oct up, drop in 80 ms
    1.0f, 80.0f,
    0.0f,        // noise_hp
    true,        // no_cycle — reached via the Drums MultiVoice, not the gesture
    0.85f,       // vel_sens — dynamic taps
    400.0f       // retrig_ms — dedupe strike + cap repeat rate
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
    2.0f, 45.0f,
    0.0f,        // noise_hp
    true,        // no_cycle — reached via the Drums MultiVoice
    0.85f,       // vel_sens — dynamic taps
    400.0f        // retrig_ms — dedupe strike + cap repeat rate
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
    1.0f,    0.15f, WAVE_SINE,   // osc1  — body (lower mode)
    1.8f,    0.12f, WAVE_TRI,    // osc2  — upper mode/ring (triangle = more harmonics)
    0.5f,    0.05f, WAVE_SINE,   // sub   — slight body weight
    2.0f,    0.0f,  WAVE_SINE,   // oct   — off
    false,                       // osc2 is a fixed ratio, not finger-2 detune
    18.0f, 4.0f, 0.30f,          // wide open & bright — the noise rattle cracks through
    1.5f,                        // drive — snap
    // noise, keytrack, rm_ratio, rm_max, fold_max, attack, release, glide (ms)
    0.72f, 0.3f, 1.0f, 0.0f, 0.10f, 1.0f, 140.0f, 5.0f,  // noise-dominant rattle (trimmed ~15%)
    // pitch_env_oct, pitch_env_ms — quick subtle snap
    0.7f, 30.0f,
    0.0f,        // noise_hp
    true,        // no_cycle — reached via the Drums MultiVoice
    0.85f,       // vel_sens — dynamic taps
    400.0f       // retrig_ms — dedupe strike + cap repeat rate
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
    0.0f, 0.0f, 1.0f,
    true,        // no_cycle — reached via the Drums MultiVoice
    0.85f,       // vel_sens — dynamic taps
    400.0f        // retrig_ms — dedupe strike + cap repeat rate
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

// Drums: when true, every tap is treated as full (100%) pressure — consistent
// hits regardless of how hard the glass reads. Set false for velocity dynamics
// (uses the drum's vel_sens). Drums only.
static constexpr bool  FIX_DRUM = true;

// ── Voice cycling ─────────────────────────────────────────────────────────────
// Voices with no_cycle = true are skipped by the FSR-hold gesture. These helpers
// give the count of cyclable voices and the 1-based position of an index among
// them (used for the LED flash count and the serial display).
static int cycle_total(){
    int n = 0;
    for(int i = 0; i < NUM_VOICES; i++) if(!VOICES[i].no_cycle) n++;
    return n;
}
static int cycle_pos(int idx){
    int p = 0;
    for(int i = 0; i <= idx && i < NUM_VOICES; i++) if(!VOICES[i].no_cycle) p++;
    return p;
}
// Next cyclable voice index after `idx` (wraps). Falls back to idx if none.
static int cycle_next(int idx){
    for(int step = 1; step <= NUM_VOICES; step++){
        int n = (idx + step) % NUM_VOICES;
        if(!VOICES[n].no_cycle) return n;
    }
    return idx;
}


// FSR voice-switch gesture: hold the FSR pressed (volume at/near 0) to cycle.
// First advance after FIRST_MS; while still held, keep advancing every REPEAT_MS.
static constexpr uint32_t VOICE_SWITCH_FIRST_MS  = 3000;
static constexpr uint32_t VOICE_SWITCH_REPEAT_MS = 2000;
