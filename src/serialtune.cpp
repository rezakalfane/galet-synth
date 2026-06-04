#include "serialtune.h"
#include "engine.h"   // hw, NUM_VOICES, MULTI_*, g_* control + tuning seam
#include "persist.h"  // g_bank, bank_set, bank_revert_*, persist_save_bank, VOICE_NAME_MAX
#include "usb_glue.h" // usb_log (CDC TX) + usb_cdc_read_avail (CDC RX) — replaces hw.PrintLine
#include "usb_audio.h" // usb_audio_is_streaming — for the `astat` diagnostic command
#include <stddef.h>   // offsetof
#include <string.h>   // strcmp / strtok
#include <math.h>     // logf

using namespace daisy;

// Editable name buffer backing g_live_voice.name (so `set name` can rewrite it).
static char g_live_name[VOICE_NAME_MAX];

// Tiny number parsers — newlib's atof/strtod pull ~16 KB of formatting code into
// internal flash (which is only 128 KB here), so we roll our own. Good enough for
// the simple decimals the protocol sends; no exponent/locale handling.
static float to_f(const char *s) {
    while(*s == ' ' || *s == '\t') s++;
    float sign = 1.0f;
    if(*s == '-') { sign = -1.0f; s++; } else if(*s == '+') s++;
    float r = 0.0f;
    while(*s >= '0' && *s <= '9') { r = r * 10.0f + (float)(*s - '0'); s++; }
    if(*s == '.') {
        s++;
        float f = 0.1f;
        while(*s >= '0' && *s <= '9') { r += (float)(*s - '0') * f; f *= 0.1f; s++; }
    }
    return r * sign;
}
static int to_i(const char *s) {
    while(*s == ' ' || *s == '\t') s++;
    int sign = 1;
    if(*s == '-') { sign = -1; s++; } else if(*s == '+') s++;
    int r = 0;
    while(*s >= '0' && *s <= '9') { r = r * 10 + (*s - '0'); s++; }
    return r * sign;
}

// ── Tuning state (the seam declared in engine.h) ──────────────────────────────
Voice             g_live_voice = VOICE_LEAD;   // snapshotted from the active voice on `tune 1`
volatile bool     g_live_tune  = false;
volatile uint32_t g_live_rev   = 0;
volatile bool     g_mon        = false;
volatile bool     g_serial_quiet = false;

// Suppress autonomous serial when a host has gone away (g_serial_quiet) or while
// tuning with the dashboard off. See g_serial_quiet in serialtune.h for why
// printing to a dead port hangs the synth.
bool serial_display_muted() { return g_serial_quiet || (g_live_tune && !g_mon); }

// ── USB CDC receive ───────────────────────────────────────────────────────────
// TinyUSB buffers incoming bytes in its own RX FIFO; the control loop drains them
// in serial_tune_poll() via usb_cdc_read_avail(). No ISR ring buffer needed now.

// ── Float key=value print (no stdio: the logger has no %f, and pulling in
// snprintf adds ~16 KB to internal flash). Decompose to integer parts and use
// only the logger-supported %c/%d/%04d specifiers. 4 decimals, rounded. ───────
static void print_kv_f(const char *key, float v) {
    char sign = v < 0.0f ? '-' : ' ';
    if(v < 0.0f) v = -v;
    int ip = (int)v;
    int fp = (int)((v - (float)ip) * 10000.0f + 0.5f);
    if(fp >= 10000) { ip++; fp -= 10000; }
    usb_log("%s=%c%d.%04d", key, sign, ip, fp);
}

// ── Field table (name → offset in Voice) ──────────────────────────────────────
enum FType { FT_F, FT_I, FT_B, FT_W };
struct FDef { const char *name; uint16_t off; uint8_t type; };
#define VF(n, t) { #n, (uint16_t)offsetof(Voice, n), (uint8_t)t }
static const FDef FIELDS[] = {
    VF(freq_low, FT_F),    VF(freq_high, FT_F),     VF(log_freq_ratio, FT_F),
    VF(osc1_ratio, FT_F),  VF(osc1_level, FT_F),    VF(osc1_wave, FT_W),
    VF(osc2_ratio, FT_F),  VF(osc2_level, FT_F),    VF(osc2_wave, FT_W),
    VF(sub_ratio, FT_F),   VF(sub_level, FT_F),     VF(sub_wave, FT_W),
    VF(oct_ratio, FT_F),   VF(oct_level, FT_F),     VF(oct_wave, FT_W),
    VF(osc2_detune, FT_B),
    VF(cutoff_mult, FT_F), VF(cutoff_oct_max, FT_F), VF(resonance, FT_F),
    VF(drive_max, FT_F),
    VF(noise_level, FT_F), VF(keytrack, FT_F),      VF(ringmod_ratio, FT_F),
    VF(ringmod_max, FT_F), VF(fold_max, FT_F),
    VF(attack_ms, FT_F),   VF(release_ms, FT_F),    VF(glide_ms, FT_F),
    VF(pitch_env_oct, FT_F), VF(pitch_env_ms, FT_F),
    VF(noise_hp, FT_F),    VF(no_cycle, FT_B),      VF(vel_sens, FT_F),
    VF(retrig_ms, FT_F),   VF(quantize, FT_B),
    VF(decay_ms, FT_F),    VF(sustain, FT_F),
    VF(chords, FT_B),      VF(chord_level, FT_F),   VF(chord_spread, FT_I),
    VF(reverb_send, FT_F), VF(delay_send, FT_F),
    VF(lfo_rate, FT_F),    VF(lfo_pitch, FT_F),
    VF(lfo_filter, FT_F),  VF(lfo_amp, FT_F),
};
static constexpr int NFIELDS = (int)(sizeof(FIELDS) / sizeof(FIELDS[0]));

static const char *wave_name(Waveform w) {
    switch(w) {
        case WAVE_SINE:   return "sine";
        case WAVE_SQUARE: return "square";
        case WAVE_SAW:    return "saw";
        default:          return "tri";
    }
}
static bool parse_wave(const char *s, Waveform &w) {
    if(!strcmp(s, "tri"))         w = WAVE_TRI;
    else if(!strcmp(s, "sine"))   w = WAVE_SINE;
    else if(!strcmp(s, "square")) w = WAVE_SQUARE;
    else if(!strcmp(s, "saw"))    w = WAVE_SAW;
    else return false;
    return true;
}
static const char *scale_name(const int8_t *s) {
    if(s == SCALE_CHROMATIC) return "chromatic";
    if(s == SCALE_MAJOR)     return "major";
    if(s == SCALE_MINOR)     return "minor";
    return s ? "custom" : "none";
}

// Global (non-Voice) effect params, settable by name.
static bool set_global(const char *f, float val) {
    if(!strcmp(f, "reverb_decay"))   { g_reverb_decay   = val; return true; }
    if(!strcmp(f, "reverb_level"))   { g_reverb_level   = val; return true; }
    if(!strcmp(f, "delay_time_ms"))  { g_delay_time_ms  = val; return true; }
    if(!strcmp(f, "delay_feedback")) { g_delay_feedback = val; return true; }
    if(!strcmp(f, "delay_level"))    { g_delay_level    = val; return true; }
    return false;
}

// Apply `set <field> <value>` to the live voice (or a global). Returns false for
// an unknown field or a bad value.
static bool set_param(const char *f, const char *v) {
    if(!strcmp(f, "scale")) {
        if(!strcmp(v, "chromatic")) {
            g_live_voice.scale = SCALE_CHROMATIC;
            g_live_voice.scale_len = (int)(sizeof(SCALE_CHROMATIC) / sizeof(SCALE_CHROMATIC[0]));
        } else if(!strcmp(v, "major")) {
            g_live_voice.scale = SCALE_MAJOR;
            g_live_voice.scale_len = (int)(sizeof(SCALE_MAJOR) / sizeof(SCALE_MAJOR[0]));
        } else if(!strcmp(v, "minor")) {
            g_live_voice.scale = SCALE_MINOR;
            g_live_voice.scale_len = (int)(sizeof(SCALE_MINOR) / sizeof(SCALE_MINOR[0]));
        } else return false;
        return true;
    }
    if(set_global(f, to_f(v))) return true;

    char *base = (char *)&g_live_voice;
    for(int i = 0; i < NFIELDS; i++) {
        if(strcmp(FIELDS[i].name, f)) continue;
        void *p = base + FIELDS[i].off;
        switch(FIELDS[i].type) {
            case FT_F: *(float *)p = to_f(v); break;
            case FT_I: *(int *)p   = to_i(v);        break;
            case FT_B: *(bool *)p  = (to_i(v) != 0); break;
            case FT_W: { Waveform w; if(!parse_wave(v, w)) return false; *(Waveform *)p = w; } break;
        }
        // freq_low / freq_high drive the exponential pitch map → keep its
        // precomputed log ratio in sync.
        if(!strcmp(f, "freq_low") || !strcmp(f, "freq_high"))
            g_live_voice.log_freq_ratio = logf(g_live_voice.freq_high / g_live_voice.freq_low);
        return true;
    }
    return false;
}

static void dump_voice() {
    usb_log("dump name=%s", g_live_voice.name);
    usb_log("idx=%d", g_voice_idx);   // bank slot, so the host syncs by index (rename-proof)
    usb_log("boot=%d", g_boot_voice); // persisted power-on voice → GUI marks the default
    const char *base = (const char *)&g_live_voice;
    for(int i = 0; i < NFIELDS; i++) {
        const void *p = base + FIELDS[i].off;
        switch(FIELDS[i].type) {
            case FT_F: print_kv_f(FIELDS[i].name, *(const float *)p); break;
            case FT_I: usb_log("%s=%d", FIELDS[i].name, *(const int *)p); break;
            case FT_B: usb_log("%s=%d", FIELDS[i].name, *(const bool *)p ? 1 : 0); break;
            case FT_W: usb_log("%s=%s", FIELDS[i].name, wave_name(*(const Waveform *)p)); break;
        }
    }
    usb_log("scale=%s", scale_name(g_live_voice.scale));
    print_kv_f("reverb_decay",   g_reverb_decay);
    print_kv_f("reverb_level",   g_reverb_level);
    print_kv_f("delay_time_ms",  g_delay_time_ms);
    print_kv_f("delay_feedback", g_delay_feedback);
    print_kv_f("delay_level",    g_delay_level);
    usb_log("end");
}

// Snapshot the currently-selected bank voice into the live working copy. The
// name is copied into our own editable buffer so `set name` can rewrite it
// without touching the bank until `save`.
static void snapshot_active() {
    int vi = g_active_voice;
    if(vi < 0 || vi >= NUM_VOICES) vi = 0;
    g_live_voice = g_bank[vi];
    strncpy(g_live_name, g_bank[vi].name ? g_bank[vi].name : "", VOICE_NAME_MAX - 1);
    g_live_name[VOICE_NAME_MAX - 1] = 0;
    g_live_voice.name = g_live_name;
    g_live_rev++;
}

static void handle_command(char *line) {
    char *cmd = strtok(line, " \t");
    if(!cmd) return;

    if(!strcmp(cmd, "tune")) {
        char *a = strtok(NULL, " \t");
        if(a && to_i(a)) { g_serial_quiet = false;   // a host is here
                           snapshot_active(); g_live_tune = true;
                           usb_log("ok tune=1 voice=%s", g_live_voice.name); }
        else             { g_live_tune = false; usb_log("ok tune=0"); }
        return;
    }
    if(!strcmp(cmd, "bye")) {
        // Host disconnecting: leave tune mode (so the synth plays the bank + the
        // idle chase resumes) and go silent so no autonomous print blocks on the
        // now-dead USB port. Cleared again by the next `tune 1`.
        g_live_tune = false;
        usb_log("ok bye");      // last line out while the host is still here
        g_serial_quiet = true;
        return;
    }
    if(!strcmp(cmd, "set")) {
        char *f = strtok(NULL, " \t");
        if(!f) { usb_log("err set <field> <value>"); return; }
        if(!strcmp(f, "name")) {
            // Name is the rest of the line (may contain spaces, e.g. "SH-101 min").
            char *rest = strtok(NULL, "");
            while(rest && (*rest == ' ' || *rest == '\t')) rest++;
            strncpy(g_live_name, rest ? rest : "", VOICE_NAME_MAX - 1);
            g_live_name[VOICE_NAME_MAX - 1] = 0;
            g_live_voice.name = g_live_name;
            usb_log("ok name=%s", g_live_name);
            return;
        }
        char *v = strtok(NULL, " \t");
        if(!v) { usb_log("err set <field> <value>"); return; }
        if(set_param(f, v)) { g_live_rev++; usb_log("ok %s=%s", f, v); }
        else usb_log("err unknown field '%s'", f);
        return;
    }
    if(!strcmp(cmd, "select")) {
        char *a = strtok(NULL, " \t");
        int n = a ? to_i(a) : -1;
        if(n >= 0 && n < NUM_VOICES) {
            g_voice_idx   = n;
            g_active_voice = (n == MULTI_IDX) ? MULTI_ZONES[0] : n;
            if(g_live_tune) snapshot_active();   // tune the newly selected voice
            usb_log("ok select=%d %s", n, g_bank[n].name);
        } else usb_log("err select 0..%d", NUM_VOICES - 1);
        return;
    }
    if(!strcmp(cmd, "dump"))  { dump_voice(); return; }
    if(!strcmp(cmd, "names")) {
        // List every bank slot's (possibly edited) name so the host can label its
        // voice picker from flash, not just the factory defaults.
        for(int i = 0; i < NUM_VOICES; i++) usb_log("name %d %s", i, g_bank[i].name);
        usb_log("end");
        return;
    }
    if(!strcmp(cmd, "mon"))   { char *a = strtok(NULL, " \t"); g_mon = (a && to_i(a));
                                usb_log("ok mon=%d", g_mon ? 1 : 0); return; }
    if(!strcmp(cmd, "save")) {
        // Commit the live voice (incl. name) into its bank slot, then persist the
        // whole bank to flash — survives power-off.
        int vi = g_active_voice; if(vi < 0 || vi >= NUM_VOICES) vi = 0;
        bank_set(vi, g_live_voice, g_live_name);
        persist_save_bank();
        usb_log("ok saved %d %s", vi, g_bank[vi].name);
        return;
    }
    if(!strcmp(cmd, "copy")) {
        // `copy <n> [name]` — duplicate the current live voice into another slot
        // and persist, without changing the active voice / interrupting audio.
        // An optional name (rest of the line, spaces ok) names the copy; omitted
        // → keep the live voice's name.
        char *a = strtok(NULL, " \t");
        int n = a ? to_i(a) : -1;
        if(n >= 0 && n < NUM_VOICES) {
            char *nm = strtok(NULL, "");                 // rest of line = optional name
            while(nm && (*nm == ' ' || *nm == '\t')) nm++;
            bank_set(n, g_live_voice, (nm && *nm) ? nm : g_live_name);
            persist_save_bank();
            usb_log("ok copy=%d %s", n, g_bank[n].name);
        } else usb_log("err copy 0..%d", NUM_VOICES - 1);
        return;
    }
    if(!strcmp(cmd, "astat")) {
        // USB-audio capture diagnostics: ring fill (frames), starved/overrun counts,
        // pre-load calls, streaming flag.
        extern volatile uint32_t g_aud_over, g_aud_under, g_aud_calls, g_aud_fill;
        usb_log("astat streaming=%d fill=%lu calls=%lu under=%lu over=%lu",
                usb_audio_is_streaming(), (unsigned long)g_aud_fill,
                (unsigned long)g_aud_calls, (unsigned long)g_aud_under, (unsigned long)g_aud_over);
        return;
    }
    if(!strcmp(cmd, "bootvoice")) {
        // Persist the CURRENT selection as the power-on voice, without committing
        // any live edits to the bank (unlike `save`). Just writes g_voice_idx (+ the
        // already-committed bank) to flash. The GUI's "Set as default" button.
        persist_save_bank();
        int vi = (g_voice_idx < 0 || g_voice_idx >= NUM_VOICES) ? 0 : g_voice_idx;
        usb_log("ok bootvoice=%d %s", g_voice_idx, g_bank[vi].name);
        return;
    }
    if(!strcmp(cmd, "factory")) {
        // `factory`     → revert the active voice to its source default
        // `factory all` → revert the whole bank
        char *a = strtok(NULL, " \t");
        if(a && !strcmp(a, "all")) {
            bank_revert_all();
        } else {
            int vi = g_active_voice; if(vi < 0 || vi >= NUM_VOICES) vi = 0;
            bank_revert_voice(vi);
        }
        persist_save_bank();
        if(g_live_tune) snapshot_active();   // reload the live copy from the reverted bank
        usb_log("ok factory %s", (a && !strcmp(a, "all")) ? "all" : g_bank[g_active_voice].name);
        return;
    }
    if(!strcmp(cmd, "help")) {
        usb_log("cmds: tune 0|1 | set <field> <val> | set name <text> | select <n>");
        usb_log("      dump | save | copy <n> [name] | bootvoice | factory [all] | mon 0|1 | bye");
        for(int i = 0; i < NFIELDS; i++) usb_log("  %s", FIELDS[i].name);
        usb_log("  name scale(chromatic|major|minor) reverb_decay reverb_level delay_time_ms delay_feedback delay_level");
        return;
    }
    usb_log("err unknown cmd '%s'", cmd);
}

void serial_tune_init() {
    // USB (incl. the CDC RX FIFO) is brought up by usb_fs_init() in main; nothing
    // to wire here now that TinyUSB owns the port.
}

void serial_tune_poll() {
    // NOTE: tud_task() is pumped from the audio callback (~2 kHz), not here — the
    // control loop is too slow for the isochronous audio endpoint. We only drain
    // already-received CDC bytes here.
    static char line[160];
    static int  li = 0;
    char chunk[64];
    int  n;
    while((n = usb_cdc_read_avail(chunk, (int)sizeof(chunk))) > 0) {
        for(int i = 0; i < n; i++) {
            char c = chunk[i];
            // Either CR or LF ends a line — terminals (screen) send '\r' on Enter,
            // others send '\n', and CRLF just yields one empty line we skip (li == 0).
            if(c == '\r' || c == '\n') { line[li] = 0; if(li > 0) handle_command(line); li = 0; }
            else if(li < (int)sizeof(line) - 1) line[li++] = c;
        }
    }
}
