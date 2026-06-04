"""The voice field model — the single source of truth shared by the CLI and GUI.

SPEC lists every Voice field in exact src/voice.h struct order (so it doubles as
the C++ export order). Each entry: (name, type, lo, hi, gui)
  type: 'name'    string literal (the voice name)
        'f'       float
        'i'       int
        'b'       bool
        'w'       Waveform   (one of WAVES)
        's'       scale      (one of SCALES; also implies scale_len)
        'derived' computed on-device (log_freq_ratio) — not user-editable
  lo,hi: slider range for 'f'/'i' (ignored otherwise)
  gui:   show as an editable control in the GUI

Keep in sync with the Voice struct in src/voice.h.
"""

WAVES = {"tri": "WAVE_TRI", "sine": "WAVE_SINE", "square": "WAVE_SQUARE", "saw": "WAVE_SAW"}
SCALES = {"chromatic": ("SCALE_CHROMATIC", 12), "major": ("SCALE_MAJOR", 7), "minor": ("SCALE_MINOR", 7)}

# VOICES[] bank order (src/voice.h) — for the GUI's voice picker / `select <n>`.
VOICE_NAMES = [
    "Lead", "Bass", "OpenBass", "RichBass", "Organ", "Scream", "Guitar", "Pad",
    "Tom", "Kick", "Snare", "HiHat", "SH-101 min", "SH-101 maj", "Drums",
]

# The Drums meta-voice (last slot, NUM_VOICES-1 in firmware). `select`ing it
# remaps the engine to the Kick slot, so it can't be addressed directly via the
# tuning protocol — bank backup/restore skips it (its fields are an unused
# placeholder; the 4 raw drums it plays are ordinary slots that ARE included).
MULTI_IDX = len(VOICE_NAMES) - 1

# (name, type, lo, hi, gui)
SPEC = [
    ("name",            "name",    0, 0,      False),
    ("freq_low",        "f",      20, 2000,   True),
    ("freq_high",       "f",      50, 8000,   True),
    ("log_freq_ratio",  "derived", 0, 0,      False),
    ("osc1_ratio",      "f",    0.25, 4.0,    True),
    ("osc1_level",      "f",     0.0, 1.2,    True),
    ("osc1_wave",       "w",       0, 0,      True),
    ("osc2_ratio",      "f",    0.25, 4.0,    True),
    ("osc2_level",      "f",     0.0, 1.2,    True),
    ("osc2_wave",       "w",       0, 0,      True),
    ("sub_ratio",       "f",    0.25, 4.0,    True),
    ("sub_level",       "f",     0.0, 1.2,    True),
    ("sub_wave",        "w",       0, 0,      True),
    ("oct_ratio",       "f",    0.25, 4.0,    True),
    ("oct_level",       "f",     0.0, 1.2,    True),
    ("oct_wave",        "w",       0, 0,      True),
    ("osc2_detune",     "b",       0, 0,      True),
    ("cutoff_mult",     "f",     0.1, 4.0,    True),
    ("cutoff_oct_max",  "f",     0.0, 14.0,   True),
    ("resonance",       "f",     0.0, 0.99,   True),
    ("drive_max",       "f",     0.0, 5.0,    True),
    ("noise_level",     "f",     0.0, 1.0,    True),
    ("keytrack",        "f",     0.0, 1.0,    True),
    ("ringmod_ratio",   "f",     0.5, 4.0,    True),
    ("ringmod_max",     "f",     0.0, 1.0,    True),
    ("fold_max",        "f",     0.0, 1.0,    True),
    ("attack_ms",       "f",     0.0, 2000,   True),
    ("release_ms",      "f",     0.0, 2000,   True),
    ("glide_ms",        "f",     0.0, 500,    True),
    ("pitch_env_oct",   "f",     0.0, 4.0,    True),
    ("pitch_env_ms",    "f",     0.0, 500,    True),
    ("noise_hp",        "f",     0.0, 1.0,    True),
    ("no_cycle",        "b",       0, 0,      True),
    ("vel_sens",        "f",     0.0, 1.0,    True),
    ("retrig_ms",       "f",     0.0, 1000,   True),
    ("quantize",        "b",       0, 0,      True),
    ("scale",           "s",       0, 0,      True),
    ("decay_ms",        "f",     0.0, 3000,   True),
    ("sustain",         "f",     0.0, 1.0,    True),
    ("chords",          "b",       0, 0,      True),
    ("chord_level",     "f",     0.0, 1.5,    True),
    ("chord_spread",    "i",       0, 3,      True),
    ("reverb_send",     "f",     0.0, 1.0,    True),
    ("delay_send",      "f",     0.0, 1.0,    True),
    ("lfo_rate",        "f",     0.0, 12.0,   True),
    ("lfo_pitch",       "f",     0.0, 1.0,    True),
    ("lfo_filter",      "f",     0.0, 1.0,    True),
    ("lfo_amp",         "f",     0.0, 1.0,    True),
]

# Global (non-Voice) effect params: (name, lo, hi). Set live, shared by all voices.
GLOBAL_SPEC = [
    ("reverb_decay",   0.0, 0.97),
    ("reverb_level",   0.0, 3.0),
    ("delay_time_ms",  0.0, 750),
    ("delay_feedback", 0.0, 0.95),
    ("delay_level",    0.0, 1.0),
]

GLOBALS = [g[0] for g in GLOBAL_SPEC]
# Field names the firmware `set` accepts (voice fields + scale + globals).
SETTABLE = [n for n, t, *_ in SPEC if t in ("f", "i", "b", "w")] + ["scale"] + GLOBALS

# Logical grouping for the GUI (mirrors the Voice struct sections / docs).
# (title, [field names]). Covers every gui-editable field + the globals once.
GROUPS = [
    ("Range",            ["freq_low", "freq_high"]),
    ("Osc 1",            ["osc1_ratio", "osc1_level", "osc1_wave"]),
    ("Osc 2",            ["osc2_ratio", "osc2_level", "osc2_wave", "osc2_detune"]),
    ("Sub",              ["sub_ratio", "sub_level", "sub_wave"]),
    ("Octave",           ["oct_ratio", "oct_level", "oct_wave"]),
    ("Filter",           ["cutoff_mult", "cutoff_oct_max", "resonance", "keytrack"]),
    ("Drive / Noise",    ["drive_max", "noise_level", "noise_hp"]),
    ("Ring / Fold",      ["ringmod_ratio", "ringmod_max", "fold_max"]),
    ("Envelope",         ["attack_ms", "release_ms", "glide_ms", "decay_ms", "sustain"]),
    ("Pitch Env",        ["pitch_env_oct", "pitch_env_ms"]),
    ("Dynamics",         ["vel_sens", "retrig_ms", "no_cycle"]),
    ("Quantize / Chords", ["quantize", "scale", "chords", "chord_level", "chord_spread"]),
    ("Sends",            ["reverb_send", "delay_send"]),
    ("LFO",              ["lfo_rate", "lfo_pitch", "lfo_filter", "lfo_amp"]),
    ("Global FX",        ["reverb_decay", "reverb_level", "delay_time_ms",
                          "delay_feedback", "delay_level"]),
]

# One-line description of each group (the (i) tooltip next to the section title).
GROUP_HELP = {
    "Range": "Pitch range finger-1 sweeps across the glass — exponential, low → high.",
    "Osc 1": "Primary oscillator: pitch ratio vs the note, mix level and waveform.",
    "Osc 2": "Second oscillator — detuned by finger-2 (osc2_detune on) or holding a "
             "fixed interval / chorus detune (off).",
    "Sub": "Sub-oscillator, usually one octave down for low-end weight.",
    "Octave": "Extra oscillator layered on top (often an octave up for body/jangle).",
    "Filter": "Moog 4-pole ladder lowpass: base cutoff, the pressure→brightness sweep, "
              "resonance and key tracking.",
    "Drive / Noise": "Waveshaper drive plus a white-noise layer (breath/hiss; noise_hp "
                     "sets the noise tone).",
    "Ring / Fold": "Finger-2 timbre effects — ring modulation and wavefolding ceilings, "
                   "and the ring carrier ratio.",
    "Envelope": "Amp envelope (attack / decay / sustain / release) and pitch glide "
                "(portamento).",
    "Pitch Env": "Pitch envelope: the note starts above pitch and decays down — the "
                 "'boom' / zap on percussion.",
    "Dynamics": "Velocity sensitivity, the held re-attack rate cap, and whether this "
                "voice joins the FSR-hold cycle gesture.",
    "Quantize / Chords": "Pitch quantize + scale, and diatonic-triad chord mode "
                         "(chord level + voicing width).",
    "Sends": "Per-voice aux sends into the shared reverb and delay.",
    "LFO": "One triangle LFO per voice. Set a destination's depth above 0 to route "
           "it: pitch = vibrato, filter = auto-wah, amp = tremolo (combinable).",
    "Global FX": "Shared reverb + delay settings (tail, level, time, feedback). Global — "
                 "they shape every voice's sends.",
}

# Per-field tooltip — what the parameter does (mirrors the Voice-struct docs).
HELP = {
    "freq_low":       "Lowest pitch finger-1 reaches (left end of the slide). The range is "
                      "swept exponentially up to freq_high.",
    "freq_high":      "Highest pitch finger-1 reaches (right end of the slide).",
    "osc1_ratio":     "Osc 1 pitch ratio vs the played note. 1.0 = root, 0.5 = octave down, "
                      "2.0 = octave up.",
    "osc1_level":     "Osc 1 mix level. 0 silences it.",
    "osc1_wave":      "Osc 1 waveform: tri / sine / square / saw (square & saw alias high up, "
                      "fine for bass).",
    "osc2_ratio":     "Osc 2 pitch ratio vs the root, used when osc2_detune is off "
                      "(e.g. a fixed 5th, or 1.005 for a chorus detune).",
    "osc2_level":     "Osc 2 mix level. 0 silences it.",
    "osc2_wave":      "Osc 2 waveform.",
    "sub_ratio":      "Sub-oscillator pitch ratio (often 0.5 = an octave down).",
    "sub_level":      "Sub-oscillator mix level.",
    "sub_wave":       "Sub-oscillator waveform.",
    "oct_ratio":      "Octave oscillator pitch ratio.",
    "oct_level":      "Octave oscillator mix level.",
    "oct_wave":       "Octave oscillator waveform.",
    "osc2_detune":    "On: finger-2 position detunes osc2 (expressive lead). "
                      "Off: osc2 holds its osc2_ratio (fixed interval / chorus).",
    "cutoff_mult":    "Filter base cutoff = played frequency × this. Higher = brighter base.",
    "cutoff_oct_max": "Octaves the Moog cutoff opens across finger-1 pressure "
                      "(the pressure → brightness sweep).",
    "resonance":      "Moog ladder resonance (Q). ~0.3 smooth, ~0.9 near self-oscillation "
                      "scream.",
    "drive_max":      "Extra waveshaper drive added on top of 1.0 as finger-1 presses harder "
                      "(grit / saturation).",
    "noise_level":    "White noise mixed into the stack (breath / chiff / hiss). 0 = none.",
    "keytrack":       "How much the cutoff base follows pitch. 1 = full; <1 keeps the cutoff "
                      "lower as pitch rises.",
    "ringmod_ratio":  "Ring-mod carrier pitch ratio. 1.0 = unison; non-integer (e.g. 2.5) = "
                      "inharmonic bell / metallic.",
    "ringmod_max":    "Max ring-mod amount from finger-2 pressure (engages above ~85% press).",
    "fold_max":       "Max wavefold amount driven by finger-2 pressure (adds harmonics).",
    "attack_ms":      "Amp attack time (ms) — how fast the note fades in.",
    "release_ms":     "Amp release + filter-close time (ms) after the finger lifts.",
    "glide_ms":       "Portamento: glide time between pitches (ms). 0 = instant.",
    "pitch_env_oct":  "Pitch-envelope depth: octaves the pitch starts ABOVE the note at onset, "
                      "decaying back. 0 = off (kick boom, zap).",
    "pitch_env_ms":   "Time for the pitch envelope to settle to the note.",
    "noise_hp":       "Noise tone: 0 = white ('shhh'), 1 = high-passed ('tsss', hats/cymbals).",
    "no_cycle":       "On: skip this voice in the FSR-hold cycle gesture (still reachable as "
                      "the boot voice / via Drums).",
    "vel_sens":       "Velocity sensitivity 0..1: onset hit strength scales loudness "
                      "(floor = 1 − vel_sens). 0 = fixed loudness.",
    "retrig_ms":      "Held re-attack rate cap (ms): min gap between pressure-bounce repeats "
                      "while held. 0 = no held re-attack.",
    "quantize":       "On: snap pitch to the scale below. Off: continuous pitch (still under "
                      "the firmware master quantize switch).",
    "scale":          "Scale for quantize + chords: chromatic / major / minor.",
    "decay_ms":       "Amp decay-to-sustain time (ms). 0 = off (note holds full while gated); "
                      ">0 = plucked feel.",
    "sustain":        "Level the amp decays to when decay_ms > 0 (0..1). 0 = decays to silence. "
                      "Ignored when decay_ms = 0.",
    "chords":         "On: play a 3-note diatonic triad (root + scale 3rd + 5th) instead of one "
                      "note. Needs a musical scale.",
    "chord_level":    "Level of the two added chord notes vs the root (chords only). <1 keeps the "
                      "root dominant; must be >0 to hear them.",
    "chord_spread":   "Voicing width (chords only): 0 = close cluster, 1 = open "
                      "(root–5th–10th), 2+ wider.",
    "reverb_send":    "Per-voice send into the shared reverb (0..1). 0 = dry; ~0.3 a hint; "
                      "~0.7 a wash.",
    "delay_send":     "Per-voice send into the shared delay/echo (0..1). Repeats darken as "
                      "they decay.",
    "lfo_rate":       "LFO speed in Hz, shared by all three destinations. 0 = the default "
                      "6 Hz. ~5 for vibrato/tremolo, slower for a sweeping wah.",
    "lfo_pitch":      "Vibrato: LFO → pitch depth 0..1 (adds to the finger-pressure "
                      "vibrato). ~0.2 a gentle wobble; 0 = off.",
    "lfo_filter":     "Auto-wah: LFO → Moog cutoff depth 0..1 (sweeps ±~90% around the "
                      "current cutoff at full depth). 0 = off.",
    "lfo_amp":        "Tremolo: LFO → amplitude depth 0..1 (1 = dips to silence on each "
                      "trough; ~0.3 a subtle throb). 0 = off.",
    "reverb_decay":   "Global reverb tail length (0..0.97). Higher = longer.",
    "reverb_level":   "Global reverb output (wet) level.",
    "delay_time_ms":  "Global delay (echo) time in ms.",
    "delay_feedback": "Global delay feedback (0..0.95). Higher = more repeats; the feedback "
                      "path is HF-damped so they darken.",
    "delay_level":    "Global delay output (wet) level.",
}
