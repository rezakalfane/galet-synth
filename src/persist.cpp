#include "persist.h"
#include "engine.h"                  // hw, VOICES, NUM_VOICES, MULTI_*, g_voice_idx, g_active_voice
#include "util/PersistentStorage.h"
#include <string.h>                  // memcmp / strncpy

using namespace daisy;

// The live bank + RAM backing for the editable names.
Voice       g_bank[NUM_VOICES];
int         g_boot_voice = 2;            // persisted power-on voice (mirrors flash)
static char g_names[NUM_VOICES][VOICE_NAME_MAX];

// ── Flash format ──────────────────────────────────────────────────────────────
// Bump BANK_MAGIC whenever the Voice struct layout changes — a mismatch makes the
// stored blob be ignored (falls back to factory), so old data can't be misread.
static constexpr uint32_t BANK_MAGIC = 0x47564232;  // 'GVB2' (bumped: Voice gained the lfo_* fields)

struct StoredVoice {
    Voice   v;                       // raw; v.name / v.scale are fixed up on load
    char    name[VOICE_NAME_MAX];    // the real (portable) name
    uint8_t scale_id;                // 0 none, 1 chromatic, 2 major, 3 minor
};
struct StoredBank {
    uint32_t    magic;
    int32_t     voice_idx;
    StoredVoice voices[NUM_VOICES];
    bool operator!=(const StoredBank &o) const { return memcmp(this, &o, sizeof(*this)) != 0; }
};

static PersistentStorage<StoredBank> *g_store = nullptr;

// ── scale <-> id ──────────────────────────────────────────────────────────────
static uint8_t scale_id(const int8_t *s) {
    if(s == SCALE_CHROMATIC) return 1;
    if(s == SCALE_MAJOR)     return 2;
    if(s == SCALE_MINOR)     return 3;
    return 0;
}
static const int8_t *id_scale(uint8_t id) {
    switch(id) {
        case 1: return SCALE_CHROMATIC;
        case 2: return SCALE_MAJOR;
        case 3: return SCALE_MINOR;
        default: return nullptr;
    }
}
static int id_scale_len(uint8_t id) {
    switch(id) {
        case 1: return (int)(sizeof(SCALE_CHROMATIC) / sizeof(SCALE_CHROMATIC[0]));
        case 2: return (int)(sizeof(SCALE_MAJOR) / sizeof(SCALE_MAJOR[0]));
        case 3: return (int)(sizeof(SCALE_MINOR) / sizeof(SCALE_MINOR[0]));
        default: return 0;
    }
}

static void set_name(int i, const char *src) {
    strncpy(g_names[i], src ? src : "", VOICE_NAME_MAX - 1);
    g_names[i][VOICE_NAME_MAX - 1] = 0;
    g_bank[i].name = g_names[i];
}

// ── (de)serialization ─────────────────────────────────────────────────────────
static void fill_factory_bank() {
    for(int i = 0; i < NUM_VOICES; i++) {
        g_bank[i] = VOICES[i];
        set_name(i, VOICES[i].name);     // point at the RAM buffer (editable)
    }
}
static void serialize(StoredBank &b) {
    b.magic = BANK_MAGIC;
    b.voice_idx = g_voice_idx;
    for(int i = 0; i < NUM_VOICES; i++) {
        b.voices[i].v = g_bank[i];
        strncpy(b.voices[i].name, g_bank[i].name, VOICE_NAME_MAX - 1);
        b.voices[i].name[VOICE_NAME_MAX - 1] = 0;
        b.voices[i].scale_id = scale_id(g_bank[i].scale);
    }
}
static void deserialize(const StoredBank &b) {
    for(int i = 0; i < NUM_VOICES; i++) {
        g_bank[i] = b.voices[i].v;                 // raw (pointers fixed up next)
        set_name(i, b.voices[i].name);
        g_bank[i].scale = id_scale(b.voices[i].scale_id);
        g_bank[i].scale_len = id_scale_len(b.voices[i].scale_id);
    }
}

void persist_init() {
    fill_factory_bank();
    static PersistentStorage<StoredBank> store(hw.qspi);
    g_store = &store;
    StoredBank def;
    serialize(def);                  // factory blob = the default
    store.Init(def);
    StoredBank &s = store.GetSettings();
    if(s.magic == BANK_MAGIC) {
        deserialize(s);
        if(s.voice_idx >= 0 && s.voice_idx < NUM_VOICES) g_voice_idx = s.voice_idx;
    } else {
        // Foreign / old-format data at this offset → reset to factory.
        s = def;
        store.Save();
    }
    g_active_voice = (g_voice_idx == MULTI_IDX) ? MULTI_ZONES[0] : g_voice_idx;
    g_boot_voice   = g_voice_idx;    // what we just restored is the persisted default
}

void persist_save_bank() {
    if(!g_store) return;
    serialize(g_store->GetSettings());
    g_store->Save();                 // erases/writes QSPI only if the blob changed
    g_boot_voice = g_voice_idx;      // a save commits g_voice_idx → the new default
}

void bank_set(int idx, const Voice &src, const char *name) {
    if(idx < 0 || idx >= NUM_VOICES) return;
    g_bank[idx] = src;
    set_name(idx, name);             // repoint name at the slot's RAM buffer
}

void bank_revert_voice(int idx) {
    if(idx < 0 || idx >= NUM_VOICES) return;
    g_bank[idx] = VOICES[idx];
    set_name(idx, VOICES[idx].name);
}

void bank_revert_all() {
    for(int i = 0; i < NUM_VOICES; i++) bank_revert_voice(i);
}
