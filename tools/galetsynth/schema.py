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
    ("Global FX",        ["reverb_decay", "reverb_level", "delay_time_ms",
                          "delay_feedback", "delay_level"]),
]
