#pragma once
// The voice model: the Voice struct, all presets, the VOICES[] bank, the
// Drums MultiVoice config, and the FSR-cycle helpers.
#include "dsp.h" // Waveform
#include <cstdint>

struct Voice {
  const char *name;

  // Pitch range. Finger-1 position 0–1000 maps exponentially across this.
  float freq_low;
  float freq_high;
  float log_freq_ratio; // = ln(freq_high / freq_low), precomputed (logf isn't
                        // constexpr)

  // Oscillator stack. Each oscillator's pitch = fundamental * ratio, and each
  // has its own waveform. Set a level to 0 to silence an oscillator.
  float osc1_ratio, osc1_level;
  Waveform osc1_wave; // primary
  float osc2_ratio, osc2_level;
  Waveform osc2_wave; // second osc (see osc2_detune)
  float sub_ratio, sub_level;
  Waveform sub_wave; // weight below
  float oct_ratio, oct_level;
  Waveform oct_wave; // shimmer / chord top
  bool osc2_detune;  // true  → finger-2 detune controls osc2 pitch (lead)
                     // false → osc2 stays at osc2_ratio (e.g. a fixed 5th)

  // Filter character. Base cutoff = freq * cutoff_mult; pressure opens it
  // by up to cutoff_oct_max octaves on top.
  float cutoff_mult;
  float cutoff_oct_max;
  float resonance;

  // Drive — pressure-controlled waveshaper gain added on top of 1.0.
  float drive_max;

  // ── Extras (appended so existing field order stays stable) ────────────────
  float noise_level;   // white noise mixed into the oscillator stack (0 = none)
  float keytrack;      // 0..1 — how much the cutoff follows pitch (1 = full)
  float ringmod_ratio; // ring-mod carrier pitch ratio (1.0 = unison; non-int =
                       // bell/metallic)
  float ringmod_max;   // max ring-mod wet from finger-2 pressure
  float fold_max;      // max wavefold amount from finger-2 pressure
  float attack_ms;     // amp envelope attack time
  float release_ms;    // amp envelope + filter-close release time
  float glide_ms;      // portamento (pitch glide) time

  // Pitch envelope — a fast downward pitch sweep at note onset (kick "boom",
  // zap, tom thump). Left at 0 (the default for voices that omit them) = off.
  float pitch_env_oct; // octaves the pitch starts ABOVE the note at onset (0 =
                       // none)
  float pitch_env_ms;  // time the pitch sweep takes to settle to the note

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

  // Pitch quantize: true = snap finger-1 to the nearest note of `scale` on
  // touch-down, then bend freely while sliding (see quantize_midi in main.cpp).
  // false = fully continuous pitch. Default false; the Organ (minor) and Guitar
  // (major) enable it.
  bool quantize;

  // Quantize scale: the scale degrees (semitone offsets within an octave) that
  // finger-1 snaps to when quantize is true. Point at one of the SCALE_* tables
  // below (or your own). null/0 = no snap (continuous). Ignored when quantize
  // is false, so non-quantizing voices leave these at their defaults.
  const int8_t *scale;
  int scale_len;

  // Amp decay-to-sustain — for plucked sounds (e.g. the guitar). decay_ms = 0
  // (default) → NO decay: the note holds at full while the finger is down, exactly
  // as before (every other voice). decay_ms > 0 → after the attack the level falls
  // to `sustain` (0..1 fraction of full; default 0 = decays to silence, a clean
  // pluck) over decay_ms. The release still applies on lift. Appended last so the
  // positional initializers of the existing presets are unchanged.
  float decay_ms;
  float sustain;
};

// ── Quantize scales ───────────────────────────────────────────────────────────
// Semitone offsets within an octave. A voice with quantize = true snaps finger-1
// to the nearest degree of its `scale` (see Voice::scale / quantize_midi).
static constexpr int8_t SCALE_CHROMATIC[] = {0, 1, 2, 3, 4,  5,
                                             6, 7, 8, 9, 10, 11};
static constexpr int8_t SCALE_MAJOR[] = {0, 2, 4, 5, 7, 9, 11};
static constexpr int8_t SCALE_MINOR[] = {0, 2, 3, 5, 7, 8, 10};

// Perfect-fifth frequency ratio = 2^(7/12).
static constexpr float RATIO_FIFTH = 1.49831f;

// The presets below are hand-formatted one-field-per-line with section headers;
// clang-format is disabled across the bank so it can't collapse the layout.
// clang-format off

// ── Voice 0: the original glass-Moog lead ─────────────────────────────────────
//   F1 pos→pitch, F1 prs→cutoff+vibrato; F2 pos→detune, F2 prs→wavefold+ringmod.
static constexpr Voice VOICE_LEAD = {
    "Lead",
    // FREQUENCY RANGE
    50.0f,
    300.0f,
    1.79176f,            // precomputed ln(freq_high/freq_low)
    // OSCILLATOR STACK
    1.0f,
    0.50f,
    WAVE_TRI,            // primary oscillator: root
    // SECONDARY OSCILLATOR
    1.0f,
    0.22f,
    WAVE_TRI,            // secondary oscillator: detune-controlled, ratio is just the base
    // SUB OSCILLATOR
    0.5f,
    0.18f,
    WAVE_TRI,            // sub   — one octave below
    // OCTAVE OSCILLATOR
    2.0f,
    0.15f,
    WAVE_TRI,            // oct   — one octave above
    // FILTER
    true,                // osc2 follows finger-2 detune
    0.3f,
    12.0f,
    0.75f,
    // DRIVE
    2.5f,
    // NOISE
    0.0f,                // noise level
    1.0f,                // keytrack
    1.0f,                // ringmod ratio (1.0 = unison; non-int = bell/metallic)
    0.15f,               // ringmod max
    0.18f,               // fold max
    4.0f,                // attack
    400.0f,              // release
    40.0f                // glide
    // PITCH ENVELOPE
    // PITCH QUANTIZE
};

// ── Voice 1: round & dark power-chord bass (root + 5th + octave + sub) ─────────
//   Same touch mappings, but osc2 is a fixed perfect fifth instead of a detune,
//   so every note sounds as a power chord. Lower range, gentler filter, less
//   drive → deep and round rather than bright and screaming.
static constexpr Voice VOICE_BASS = {
    "Bass",
    // FREQUENCY RANGE
    40.0f,
    160.0f,
    1.38629f,            // 40–160 Hz, ln(4) — sits under the lead
    // OSCILLATOR STACK
    1.0f,
    0.45f,
    WAVE_TRI,            // osc1  — root
    // SECONDARY OSCILLATOR
    RATIO_FIFTH,
    0.30f,
    WAVE_TRI,            // osc2  — fixed perfect fifth (the power chord)
    // SUB OSCILLATOR
    0.5f,
    0.34f,
    WAVE_TRI,            // sub   — extra weight for a round bottom
    // OCTAVE OSCILLATOR
    2.0f,
    0.20f,
    WAVE_TRI,            // oct   — top of the power chord
    // FILTER
    false,               // osc2 is a fixed fifth, not finger-2 detune
    0.25f,
    7.0f,
    0.50f,               // darker: lower base cutoff, gentler sweep, less res
    // DRIVE
    1.0f,                // less drive — keep it clean and round
    // NOISE
    0.0f,                // noise level
    1.0f,                // keytrack
    1.0f,                // ringmod ratio
    0.12f,               // ringmod max
    0.12f,               // fold max
    6.0f,                // attack
    300.0f,              // release
    25.0f                // glide
    // PITCH ENVELOPE
    // PITCH QUANTIZE
};

// ── Voice 2: open power-chord bass ────────────────────────────────────────────
//   Same chord stack as VOICE_BASS, but the filter starts brighter and sweeps
//   much further with pressure — light touch is still warm, hard press opens it
//   wide. A touch more resonance and drive for presence as it opens.
static constexpr Voice VOICE_BASS_OPEN = {
    "OpenBass",
    // FREQUENCY RANGE
    40.0f,
    180.0f,
    1.50408f,            // 40–180 Hz, ln(4.5) — a little more reach up top
    // OSCILLATOR STACK
    1.0f,
    0.45f,
    WAVE_TRI,            // osc1  — root
    // SECONDARY OSCILLATOR
    RATIO_FIFTH,
    0.30f,
    WAVE_TRI,            // osc2  — fixed perfect fifth (the power chord)
    // SUB OSCILLATOR
    0.5f,
    0.30f,
    WAVE_TRI,            // sub   — slightly less so the openness reads
    // OCTAVE OSCILLATOR
    2.0f,
    0.22f,
    WAVE_TRI,            // oct   — top of the power chord
    // FILTER
    false,               // osc2 is a fixed fifth, not finger-2 detune
    0.5f,
    10.0f,
    0.65f,               // brighter base + wider pressure sweep + more resonance
    // DRIVE
    1.8f,                // more drive — grit comes in as you press
    // NOISE
    0.0f,                // noise level
    1.0f,                // keytrack
    1.0f,                // ringmod ratio
    0.15f,               // ringmod max
    0.15f,               // fold max
    5.0f,                // attack
    350.0f,              // release
    25.0f                // glide
    // PITCH ENVELOPE
    // PITCH QUANTIZE
};

// ── Voice 3: rich power-chord bass with mixed waveforms + huge filter sweep ────
//   Mixed shapes for body: saw root (harmonic-rich), square fifth (hollow), a
//   clean sine sub (tight low end), saw octave on top. Starts dark, then
//   pressure opens the cutoff dramatically (13 octaves) with high resonance —
//   a vocal, acid-style "wow" as you press harder.
static constexpr Voice VOICE_BASS_RICH = {
    "RichBass",
    // FREQUENCY RANGE
    40.0f,
    160.0f,
    1.38629f,            // 40–160 Hz, ln(4)
    // OSCILLATOR STACK
    1.0f,
    0.40f,
    WAVE_SAW,            // osc1  — saw root (rich harmonics)
    // SECONDARY OSCILLATOR
    RATIO_FIFTH,
    0.26f,
    WAVE_SQUARE,         // osc2  — square fifth (hollow contrast)
    // SUB OSCILLATOR
    0.5f,
    0.32f,
    WAVE_SINE,           // sub   — clean sine for a tight low end
    // OCTAVE OSCILLATOR
    2.0f,
    0.18f,
    WAVE_SAW,            // oct   — saw top of the power chord
    // FILTER
    false,               // osc2 is a fixed fifth, not finger-2 detune
    0.2f,
    13.0f,
    0.82f,               // dark base + enormous pressure sweep + high resonance
    // DRIVE
    2.2f,                // generous drive — grit blooms with pressure
    // NOISE
    0.0f,                // noise level
    1.0f,                // keytrack
    1.0f,                // ringmod ratio
    0.20f,               // ringmod max
    0.20f,               // fold max
    3.0f,                // attack
    250.0f,              // release
    18.0f                // glide — snappy + fast acid glide
    // PITCH ENVELOPE
    // PITCH QUANTIZE
};

// ── Voice 4: clean sine organ / flute (a melodic, non-bass departure) ─────────
//   Pure sine oscillators stacked like organ drawbars: octave-down body, root,
//   octave-up air, plus a second osc tuned a hair sharp (1.005×) for a slow,
//   warm chorus shimmer instead of a chord. Higher register than the basses, a
//   mostly-open clean filter, and just a touch of drive so pressure adds gentle
//   harmonic warmth. Sines never alias, so it stays smooth all the way up.
static constexpr Voice VOICE_ORGAN = {
    "Organ",
    // FREQUENCY RANGE
    65.0f,
    520.0f,
    2.07944f,            // 65–520 Hz, ln(8) — a melodic keyboard register
    // OSCILLATOR STACK
    1.0f,
    0.45f,
    WAVE_SINE,           // osc1  — root (8')
    // SECONDARY OSCILLATOR
    1.005f,
    0.40f,
    WAVE_SINE,           // osc2  — slightly sharp unison → slow chorus shimmer
    // SUB OSCILLATOR
    0.5f,
    0.30f,
    WAVE_SINE,           // sub   — octave-down body (16')
    // OCTAVE OSCILLATOR
    2.0f,
    0.28f,
    WAVE_SINE,           // oct   — octave-up air (4')
    // FILTER
    false,               // osc2 is a fixed chorus detune, not finger-2 detune
    2.0f,
    5.0f,
    0.20f,               // mostly-open clean filter, gentle pressure lift, low res
    // DRIVE
    0.8f,                // light drive — pressure adds subtle warmth, stays pure
    // NOISE
    0.03f,               // noise level — breath chiff
    1.0f,                // keytrack
    1.0f,                // ringmod ratio
    0.05f,               // ringmod max
    0.0f,                // fold max
    8.0f,                // attack
    120.0f,              // release
    8.0f,                // glide — fast on/off, no glide
    // PITCH ENVELOPE
    0.0f,                // pitch env octaves (off)
    0.0f,                // pitch env time
    // NOISE TONE
    0.0f,                // noise_hp (white)
    // CYCLING
    false,               // no_cycle (stays in the FSR cycle)
    // VELOCITY
    0.0f,                // vel_sens (fixed loudness)
    // RETRIGGER
    0.0f,                // retrig_ms (off)
    // PITCH QUANTIZE
    true,                // quantize — snap on touch-down (organ/keyboard feel)
    SCALE_MINOR,         // scale
    (int)(sizeof(SCALE_MINOR) / sizeof(SCALE_MINOR[0])) // scale_len — minor
};

// ── Voice 5: screaming aggressive lead ────────────────────────────────────────
//   Detuned sawtooth stack (root + a hair-sharp twin + octave) for a thick,
//   buzzing supersaw bite, pushed up into a cutting register. Heavy drive and a
//   very high filter resonance make the cutoff whistle and scream as pressure
//   sweeps it wide open. Finger-2 pressure piles on ring-mod + wavefold for
//   extra snarl. Loud and nasty — the output soft-clip keeps it from blowing up.
static constexpr Voice VOICE_SCREAM = {
    "Scream",
    // FREQUENCY RANGE
    90.0f,
    720.0f,
    2.07944f,            // 90–720 Hz, ln(8) — high, cutting register
    // OSCILLATOR STACK
    1.0f,
    0.42f,
    WAVE_SAW,            // osc1  — saw root
    // SECONDARY OSCILLATOR
    1.008f,
    0.38f,
    WAVE_SAW,            // osc2  — slightly sharp twin → thick beating buzz
    // SUB OSCILLATOR
    0.5f,
    0.20f,
    WAVE_SAW,            // sub   — a little weight underneath
    // OCTAVE OSCILLATOR
    2.0f,
    0.30f,
    WAVE_SAW,            // oct   — octave up for bright, cutting scream
    // FILTER
    false,               // osc2 is a fixed detune, not finger-2 controlled
    3.0f,
    11.0f,
    0.90f,               // bright base + wide sweep + near-self-osc resonance
    // DRIVE
    4.0f,                // heavy drive — saturated and aggressive
    // NOISE
    0.05f,               // noise level — hiss
    1.0f,                // keytrack
    2.5f,                // ringmod ratio — inharmonic metallic
    0.40f,               // ringmod max
    0.30f,               // fold max
    2.0f,                // attack
    200.0f,              // release
    35.0f                // glide
    // PITCH ENVELOPE
    // PITCH QUANTIZE
};

// ── Voice 6: electric guitar power chord ──────────────────────────────────────
//   A driven sawtooth power chord — each note is a root + fixed perfect fifth +
//   octave-up jangle over a saw octave-down body, so it plays as a chord like a
//   rock guitar. Heavy-ish drive adds overdrive crunch; the filter sits a touch
//   above the fundamental and sweeps wide open with pressure (dig in = bright,
//   pick-attack bite), with resonance for a vocal amp "quack". A fast pitch-env
//   "twang" + a little pick-grit noise on the onset, then amp decay-to-silence
//   (decay_ms/sustain below) so each note plucks and rings out over ~1.2 s while
//   held — like a struck string, not an organ. Quantized to a major scale so
//   tapped chords land in key. For a BASS guitar instead: drop freq_low/high ~½,
//   cut drive to ~1.5, use WAVE_TRI, and raise `sustain` if you want it to hold.
static constexpr Voice VOICE_GUITAR = {
    "Guitar",
    // FREQUENCY RANGE
    80.0f,
    320.0f,
    1.38629f,            // 80–320 Hz, ln(4) — E2-ish up two octaves
    // OSCILLATOR STACK
    1.0f,
    0.40f,
    WAVE_SAW,            // osc1  — saw root (bright, string-like)
    // SECONDARY OSCILLATOR
    RATIO_FIFTH,
    0.30f,
    WAVE_SAW,            // osc2  — fixed perfect fifth (the power chord)
    // SUB OSCILLATOR
    0.5f,
    0.22f,
    WAVE_SAW,            // sub   — octave-down body/weight
    // OCTAVE OSCILLATOR
    2.0f,
    0.18f,
    WAVE_SAW,            // oct   — octave-up jangle / pick brightness
    // FILTER
    false,               // osc2 is a fixed fifth, not finger-2 detune
    1.5f,
    6.0f,
    0.42f,               // bright-ish base, wide sweep, resonant mid "quack" (amp-like)
    // DRIVE
    3.6f,                // overdrive crunch — the electric-guitar grit
    // NOISE
    0.03f,               // noise level — a little pick grit (rides the amp decay)
    1.0f,                // keytrack
    1.0f,                // ringmod ratio
    0.12f,               // ringmod max
    0.15f,               // fold max
    3.0f,                // attack — fast pluck onset
    280.0f,              // release — ring tail after lift
    8.0f,                // glide — quick, distinct chord changes
    // PITCH ENVELOPE — fast pick-attack "twang": start ~0.12 oct sharp, settle 18 ms
    0.12f, 18.0f,        // pitch_env_oct, pitch_env_ms
    // noise_hp, no_cycle, vel_sens, retrig_ms (defaults)
    0.0f, false, 0.0f, 0.0f,
    // PITCH QUANTIZE — snap to a major scale on touch-down so chords land in key
    true,
    SCALE_MAJOR, (int)(sizeof(SCALE_MAJOR) / sizeof(SCALE_MAJOR[0])),
    // AMP DECAY — the pluck: ring out to silence over ~1.2 s while held
    1200.0f,             // decay_ms
    0.0f                 // sustain (0 = decays to silence)
};

// ── Voice 7: warm ensemble pad ────────────────────────────────────────────────
//   A lush, evolving pad. Two saws a hair apart (1.006×) give an ensemble/chorus
//   width, a soft triangle sub adds body, an airy sine octave sits on top. A
//   moderate, low-resonance filter keeps it warm rather than bright. The pad
//   character comes from the envelope: a slow ~2.2 s swell in and a long ~2 s
//   tail, so notes bloom and linger. A whisper of noise adds air.
static constexpr Voice VOICE_PAD = {
    "Pad",
    // FREQUENCY RANGE
    70.0f,
    560.0f,
    2.07944f,            // 70–560 Hz, ln(8) — mid melodic register
    // OSCILLATOR STACK
    1.0f,
    0.32f,
    WAVE_SAW,            // osc1  — saw root
    // SECONDARY OSCILLATOR
    1.006f,
    0.30f,
    WAVE_SAW,            // osc2  — detuned saw → lush ensemble width
    // SUB OSCILLATOR
    0.5f,
    0.22f,
    WAVE_TRI,            // sub   — soft triangle body
    // OCTAVE OSCILLATOR
    2.0f,
    0.18f,
    WAVE_SINE,           // oct   — airy sine top
    // FILTER
    false,               // osc2 is a fixed chorus detune, not finger-2
    1.2f,
    3.0f,
    0.20f,               // warm & closed: low base cutoff, gentle sweep, low res
    // DRIVE
    0.6f,                // gentle drive — stays smooth
    // NOISE
    0.015f,              // noise level — air
    1.0f,                // keytrack
    1.0f,                // ringmod ratio
    0.0f,                // ringmod max
    0.08f,               // fold max
    2200.0f,             // attack — slow swell
    2000.0f,             // release — long tail
    80.0f                // glide
    // PITCH ENVELOPE
    // PITCH QUANTIZE
};

// ── Voice 8: tom / percussion (noise-driven) ─────────────────────────────────
//   A drum, not a sustained note — PLAY IT BY TAPPING the glass. Noise over two
//   low sine modes (body + an inharmonic 1.6× ring) and a little sub thump. An
//   instant attack and a short ~150 ms release make each tap a percussive hit.
//   A closed, dark filter keeps it round and muffled (tom, not bright snare);
//   harder presses open it a touch. Finger-1 position tunes the drum.
static constexpr Voice VOICE_TOM = {
    "Tom",
    // FREQUENCY RANGE
    40.0f,
    400.0f,
    2.30259f,            // 40–400 Hz, ln(10) — finger-1 tunes from kick/tom up
    // OSCILLATOR STACK
    1.0f,
    0.22f,
    WAVE_SINE,           // osc1  — body (lower mode)
    // SECONDARY OSCILLATOR
    1.6f,
    0.15f,
    WAVE_SINE,           // osc2  — upper mode (inharmonic ring)
    // SUB OSCILLATOR
    0.5f,
    0.10f,
    WAVE_SINE,           // sub   — a little thump
    // OCTAVE OSCILLATOR
    2.0f,
    0.0f,
    WAVE_SINE,           // oct   — off
    // FILTER
    false,               // osc2 is a fixed ratio, not finger-2 detune
    3.0f,
    3.0f,
    0.30f,               // tom — low 1-pole cutoff for a dark, round body
    // DRIVE
    1.5f,                // drive — snap and saturation on hard hits
    // NOISE
    0.70f,               // noise level — noise-dominant
    0.3f,                // keytrack
    1.0f,                // ringmod ratio
    0.0f,                // ringmod max
    0.10f,               // fold max
    1.0f,                // attack — instant hit
    150.0f,              // release — short tail
    5.0f,                // glide
    // PITCH ENVELOPE
    1.0f,                // pitch env octaves — tom "boww": start 1 oct up
    80.0f,               // pitch env time — drop in 80 ms
    // NOISE TONE
    0.0f,                // noise_hp (white)
    // CYCLING
    true,                // no_cycle — reached via the Drums MultiVoice
    // VELOCITY
    0.85f,               // vel_sens — dynamic taps
    // RETRIGGER
    400.0f               // retrig_ms — dedupe strike + cap repeat rate
    // PITCH QUANTIZE
};

// ── Voice 9: kick drum (noise-driven) ────────────────────────────────────────
//   A deep, punchy thud — PLAY IT BY TAPPING. A low sine body with a sub-octave
//   for weight and a quiet octave "click" for beater attack; a whisper of noise
//   adds beater texture. Very closed dark filter + a little drive keep it round
//   and punchy. Instant attack, short tail. Finger-1 position tunes the kick.
//   A pitch envelope (start 2 oct up, drop in 45 ms) gives the classic "boom".
static constexpr Voice VOICE_KICK = {
    "Kick",
    // FREQUENCY RANGE
    30.0f,
    80.0f,
    0.98083f,            // 30–80 Hz, ln(2.67) — deep, position tunes the kick
    // OSCILLATOR STACK
    1.0f,
    0.55f,
    WAVE_SINE,           // osc1  — deep sine body
    // SECONDARY OSCILLATOR
    2.0f,
    0.10f,
    WAVE_SINE,           // osc2  — octave "click" for attack punch
    // SUB OSCILLATOR
    0.5f,
    0.15f,
    WAVE_SINE,           // sub   — sub-octave rumble
    // OCTAVE OSCILLATOR
    2.0f,
    0.0f,
    WAVE_SINE,           // oct   — off
    // FILTER
    false,               // osc2 is a fixed ratio, not finger-2 detune
    3.0f,
    2.0f,
    0.20f,               // very closed/dark — round; tiny pressure sweep
    // DRIVE
    1.8f,                // drive — punch and saturation
    // NOISE
    0.08f,               // noise level — soft beater click
    0.5f,                // keytrack
    1.0f,                // ringmod ratio
    0.0f,                // ringmod max
    0.0f,                // fold max
    1.0f,                // attack — instant
    160.0f,              // release — short
    5.0f,                // glide
    // PITCH ENVELOPE
    2.0f,                // pitch env octaves — kick "boom": start 2 oct up
    45.0f,               // pitch env time — drop in 45 ms
    // NOISE TONE
    0.0f,                // noise_hp (white)
    // CYCLING
    true,                // no_cycle — reached via the Drums MultiVoice
    // VELOCITY
    0.85f,               // vel_sens — dynamic taps
    // RETRIGGER
    400.0f               // retrig_ms — dedupe strike + cap repeat rate
    // PITCH QUANTIZE
};

// ── Voice 10: snare (noise-driven) ───────────────────────────────────────────
//   The bright, noisy cousin of the Tom — PLAY IT BY TAPPING. Broadband noise
//   (the "wires" rattle) dominates over two tonal modes (body + a 1.8× ring); a
//   wide-open filter keeps it crisp and cracking. Instant attack, short tail,
//   and a quick subtle pitch snap (0.7 oct in 30 ms) for the hit. Harder presses
//   open it further. Finger-1 position tunes the drum.
static constexpr Voice VOICE_SNARE = {
    "Snare",
    // FREQUENCY RANGE
    150.0f,
    500.0f,
    1.20397f,            // 150–500 Hz, ln(3.33) — snare tuning range
    // OSCILLATOR STACK
    1.0f,
    0.15f,
    WAVE_SINE,           // osc1  — body (lower mode)
    // SECONDARY OSCILLATOR
    1.8f,
    0.12f,
    WAVE_TRI,            // osc2  — upper mode/ring (triangle = more harmonics)
    // SUB OSCILLATOR
    0.5f,
    0.05f,
    WAVE_SINE,           // sub   — slight body weight
    // OCTAVE OSCILLATOR
    2.0f,
    0.0f,
    WAVE_SINE,           // oct   — off
    // FILTER
    false,               // osc2 is a fixed ratio, not finger-2 detune
    18.0f,
    4.0f,
    0.30f,               // wide open & bright — the noise rattle cracks through
    // DRIVE
    1.5f,                // drive — snap
    // NOISE
    0.72f,               // noise level — noise-dominant rattle
    0.3f,                // keytrack
    1.0f,                // ringmod ratio
    0.0f,                // ringmod max
    0.10f,               // fold max
    1.0f,                // attack — instant
    140.0f,              // release — short
    5.0f,                // glide
    // PITCH ENVELOPE
    0.7f,                // pitch env octaves — quick subtle snap
    30.0f,               // pitch env time — 30 ms
    // NOISE TONE
    0.0f,                // noise_hp (white)
    // CYCLING
    true,                // no_cycle — reached via the Drums MultiVoice
    // VELOCITY
    0.85f,               // vel_sens — dynamic taps
    // RETRIGGER
    400.0f               // retrig_ms — dedupe strike + cap repeat rate
    // PITCH QUANTIZE
};

// ── Voice 11: hi-hat (noise-driven) ──────────────────────────────────────────
//   A crisp "tss" — PLAY IT BY TAPPING. Almost pure high-frequency noise through
//   a wide-open filter, with three inharmonic square partials for metallic edge
//   and no low end. Instant attack and a very short ~55 ms tail = a tight closed
//   hat. Finger-1 position tunes the metallic pitch. (Raise release_ms for an
//   open-hat feel.)
static constexpr Voice VOICE_HIHAT = {
    "HiHat",
    // FREQUENCY RANGE
    4000.0f,
    11000.0f,
    1.01160f,            // 4000–11000 Hz, ln(2.75) — metallic edge pitch
    // OSCILLATOR STACK
    1.0f,
    0.10f,
    WAVE_SQUARE,         // osc1  — quiet metallic edge (the tss noise leads)
    // SECONDARY OSCILLATOR
    1.68f,
    0.08f,
    WAVE_SQUARE,         // osc2  — inharmonic metallic partial
    // SUB OSCILLATOR
    0.5f,
    0.0f,
    WAVE_SINE,           // sub   — off (no low end on a hat)
    // OCTAVE OSCILLATOR
    2.0f,
    0.06f,
    WAVE_SQUARE,         // oct   — high partial (2.0× keeps it under Nyquist)
    // FILTER
    false,               // osc2 is a fixed ratio, not finger-2 detune
    25.0f,
    2.0f,
    0.20f,               // wide open
    // DRIVE
    1.2f,                // light drive
    // NOISE
    0.80f,               // noise level — noise-led
    0.2f,                // keytrack
    1.0f,                // ringmod ratio
    0.0f,                // ringmod max
    0.05f,               // fold max
    1.0f,                // attack — instant
    160.0f,              // release — sizzly tail
    3.0f,                // glide
    // PITCH ENVELOPE
    0.0f,                // pitch env octaves (off)
    0.0f,                // pitch env time
    // NOISE TONE
    1.0f,                // noise_hp — high-passed bright "tsss"
    // CYCLING
    true,                // no_cycle — reached via the Drums MultiVoice
    // VELOCITY
    0.85f,               // vel_sens — dynamic taps
    // RETRIGGER
    400.0f               // retrig_ms — dedupe strike + cap repeat rate
    // PITCH QUANTIZE
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
    // FREQUENCY RANGE
    30.0f,
    80.0f,
    0.98083f,
    // OSCILLATOR STACK
    1.0f,
    0.55f,
    WAVE_SINE,
    // SECONDARY OSCILLATOR
    1.0f,
    0.0f,
    WAVE_SINE,
    // SUB OSCILLATOR
    0.5f,
    0.0f,
    WAVE_SINE,
    // OCTAVE OSCILLATOR
    2.0f,
    0.0f,
    WAVE_SINE,
    // FILTER
    false,
    3.0f,
    2.0f,
    0.20f,
    // DRIVE
    1.0f,
    // NOISE
    0.0f,                // noise level
    0.5f,                // keytrack
    1.0f,                // ringmod ratio
    0.0f,                // ringmod max
    0.0f,                // fold max
    1.0f,                // attack
    160.0f,              // release
    5.0f,                // glide
    // PITCH ENVELOPE
    0.0f,                // pitch env octaves (placeholder — zone drums play)
    0.0f,                // pitch env time
    // NOISE TONE
    0.0f                 // noise_hp
    // CYCLING
    // VELOCITY
    // RETRIGGER
    // PITCH QUANTIZE
};

// clang-format on

// ── Voice bank ────────────────────────────────────────────────────────────────
// All voices, in cycle order. Holding the FSR pressed steps through these live
// (see the voice-switch gesture in the main loop). Edit the order here to taste.
static constexpr Voice VOICES[] = {
    VOICE_LEAD,  VOICE_BASS,   VOICE_BASS_OPEN,   VOICE_BASS_RICH,
    VOICE_ORGAN, VOICE_SCREAM, VOICE_GUITAR,      VOICE_PAD,
    VOICE_TOM,   VOICE_KICK,   VOICE_SNARE,       VOICE_HIHAT,
    VOICE_DRUMS, // MultiVoice — must stay last (MULTI_IDX = NUM_VOICES-1)
};
static constexpr int NUM_VOICES = (int)(sizeof(VOICES) / sizeof(VOICES[0]));

// ── MultiVoice "Drums" config ─────────────────────────────────────────────────
// MULTI_IDX is the bank slot of VOICE_DRUMS (kept last). When that voice is
// selected, the slider splits into 4 equal zones mapped to these drum voices
// (left→right), and the fine position within a zone snaps the pitch to one of
// MULTI_INTERVALS above the drum's base pitch (freq_low).
static constexpr int MULTI_IDX = NUM_VOICES - 1;
static constexpr int MULTI_ZONES[4] = {
    9, 10, 8, 11}; // Kick, Snare, Tom, HiHat (bank indices)
static constexpr float MULTI_INTERVALS[] = {1.0f, 1.33484f, 1.49831f,
                                            2.0f}; // root, 4th, 5th, octave
static constexpr int MULTI_NINTERVALS =
    (int)(sizeof(MULTI_INTERVALS) / sizeof(MULTI_INTERVALS[0]));

// Drums: when true, every tap is treated as full (100%) pressure — consistent
// hits regardless of how hard the glass reads. Set false for velocity dynamics
// (uses the drum's vel_sens). Drums only.
static constexpr bool FIX_DRUM = true;

// ── Voice cycling ─────────────────────────────────────────────────────────────
// Voices with no_cycle = true are skipped by the FSR-hold gesture. These helpers
// give the count of cyclable voices and the 1-based position of an index among
// them (used for the LED flash count and the serial display).
static int cycle_total() {
  int n = 0;
  for (int i = 0; i < NUM_VOICES; i++)
    if (!VOICES[i].no_cycle)
      n++;
  return n;
}
static int cycle_pos(int idx) {
  int p = 0;
  for (int i = 0; i <= idx && i < NUM_VOICES; i++)
    if (!VOICES[i].no_cycle)
      p++;
  return p;
}
// Next cyclable voice index after `idx` (wraps). Falls back to idx if none.
static int cycle_next(int idx) {
  for (int step = 1; step <= NUM_VOICES; step++) {
    int n = (idx + step) % NUM_VOICES;
    if (!VOICES[n].no_cycle)
      return n;
  }
  return idx;
}

// FSR voice-select gesture (tap-the-glass to cycle). Press the FSR fully (to the
// mute floor, fsr_raw <= FSR_MIN) while NOT touching the glass for ENTER_MS to
// enter select mode (LEDs blink the current voice's position). Then, with the FSR
// still held, each GLASS TAP advances one voice (wrapping), blinking N = the new
// position and previewing it. Release the FSR (it climbs back up) to keep the
// shown voice (persisted) and exit. TAP_GAP_MS debounces the glass taps.
static constexpr uint32_t VOICE_SELECT_ENTER_MS   = 2000;
static constexpr uint32_t VOICE_SELECT_TAP_GAP_MS = 150;
