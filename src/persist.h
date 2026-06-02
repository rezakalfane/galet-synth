#pragma once
// Editable, persistent voice bank. The factory presets live in the constexpr
// VOICES[] (voice.h, read-only). At boot we copy them into a mutable RAM bank
// `g_bank[]` (what the engine actually plays), then override it with whatever was
// saved to QSPI flash. The tuner can edit a voice and `save` it back; `revert`
// restores a slot (or all) from the factory VOICES[].
//
// Voice holds two pointers (name, scale) that can't go to flash verbatim, so the
// stored form keeps a name string + a scale id and fixes the pointers up on load
// (each bank voice's name points into a RAM buffer so it's editable).
#include "voice.h"

#define VOICE_NAME_MAX 24

// The live bank the engine/control loop/tuner read (defined in persist.cpp;
// also extern'd on the engine seam for the audio path).
extern Voice g_bank[NUM_VOICES];

// Build the factory bank, then load any saved overrides from flash and restore
// the persisted voice index. Call once at boot (replaces the old voice-only
// PersistentStorage). Needs hw.qspi → call after hw.Init().
void persist_init();

// Serialize g_bank (+ g_voice_idx) to flash. Writes only if changed (wear).
void persist_save_bank();

// Copy `src` (with `name`) into bank slot idx, repointing the slot's name at its
// RAM buffer. Used by the tuner's `save` to commit the live voice before writing.
void bank_set(int idx, const Voice &src, const char *name);

// Restore a slot (or the whole bank) from the factory VOICES[].
void bank_revert_voice(int idx);
void bank_revert_all();
