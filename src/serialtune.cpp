#include "serialtune.h"
#include "engine.h"   // hw, NUM_VOICES, MULTI_*, g_* control + tuning seam
#include "persist.h"  // g_bank, bank_set, bank_revert_*, persist_save_bank, VOICE_NAME_MAX
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

// ── USB CDC receive ring buffer ───────────────────────────────────────────────
// The receive callback runs in USB interrupt context, so it only copies bytes
// into this ring; serial_tune_poll() (control-loop thread) parses whole lines.
static constexpr int     RX_SZ = 512;
static volatile char     s_rx[RX_SZ];
static volatile uint16_t s_rx_head = 0, s_rx_tail = 0;

static void rx_cb(uint8_t *buf, uint32_t *len) {
    uint32_t n = *len;
    for(uint32_t i = 0; i < n; i++) {
        uint16_t nh = (uint16_t)((s_rx_head + 1) % RX_SZ);
        if(nh == s_rx_tail) break;   // ring full → drop the rest
        s_rx[s_rx_head] = (char)buf[i];
        s_rx_head = nh;
    }
}

// ── Float key=value print (no stdio: the logger has no %f, and pulling in
// snprintf adds ~16 KB to internal flash). Decompose to integer parts and use
// only the logger-supported %c/%d/%04d specifiers. 4 decimals, rounded. ───────
static void print_kv_f(const char *key, float v) {
    char sign = v < 0.0f ? '-' : ' ';
    if(v < 0.0f) v = -v;
    int ip = (int)v;
    int fp = (int)((v - (float)ip) * 10000.0f + 0.5f);
    if(fp >= 10000) { ip++; fp -= 10000; }
    hw.PrintLine("%s=%c%d.%04d", key, sign, ip, fp);
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
    hw.PrintLine("dump name=%s", g_live_voice.name);
    hw.PrintLine("idx=%d", g_voice_idx);   // bank slot, so the host syncs by index (rename-proof)
    const char *base = (const char *)&g_live_voice;
    for(int i = 0; i < NFIELDS; i++) {
        const void *p = base + FIELDS[i].off;
        switch(FIELDS[i].type) {
            case FT_F: print_kv_f(FIELDS[i].name, *(const float *)p); break;
            case FT_I: hw.PrintLine("%s=%d", FIELDS[i].name, *(const int *)p); break;
            case FT_B: hw.PrintLine("%s=%d", FIELDS[i].name, *(const bool *)p ? 1 : 0); break;
            case FT_W: hw.PrintLine("%s=%s", FIELDS[i].name, wave_name(*(const Waveform *)p)); break;
        }
    }
    hw.PrintLine("scale=%s", scale_name(g_live_voice.scale));
    print_kv_f("reverb_decay",   g_reverb_decay);
    print_kv_f("reverb_level",   g_reverb_level);
    print_kv_f("delay_time_ms",  g_delay_time_ms);
    print_kv_f("delay_feedback", g_delay_feedback);
    print_kv_f("delay_level",    g_delay_level);
    hw.PrintLine("end");
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
                           hw.PrintLine("ok tune=1 voice=%s", g_live_voice.name); }
        else             { g_live_tune = false; hw.PrintLine("ok tune=0"); }
        return;
    }
    if(!strcmp(cmd, "bye")) {
        // Host disconnecting: leave tune mode (so the synth plays the bank + the
        // idle chase resumes) and go silent so no autonomous print blocks on the
        // now-dead USB port. Cleared again by the next `tune 1`.
        g_live_tune = false;
        hw.PrintLine("ok bye");      // last line out while the host is still here
        g_serial_quiet = true;
        return;
    }
    if(!strcmp(cmd, "set")) {
        char *f = strtok(NULL, " \t");
        if(!f) { hw.PrintLine("err set <field> <value>"); return; }
        if(!strcmp(f, "name")) {
            // Name is the rest of the line (may contain spaces, e.g. "SH-101 min").
            char *rest = strtok(NULL, "");
            while(rest && (*rest == ' ' || *rest == '\t')) rest++;
            strncpy(g_live_name, rest ? rest : "", VOICE_NAME_MAX - 1);
            g_live_name[VOICE_NAME_MAX - 1] = 0;
            g_live_voice.name = g_live_name;
            hw.PrintLine("ok name=%s", g_live_name);
            return;
        }
        char *v = strtok(NULL, " \t");
        if(!v) { hw.PrintLine("err set <field> <value>"); return; }
        if(set_param(f, v)) { g_live_rev++; hw.PrintLine("ok %s=%s", f, v); }
        else hw.PrintLine("err unknown field '%s'", f);
        return;
    }
    if(!strcmp(cmd, "select")) {
        char *a = strtok(NULL, " \t");
        int n = a ? to_i(a) : -1;
        if(n >= 0 && n < NUM_VOICES) {
            g_voice_idx   = n;
            g_active_voice = (n == MULTI_IDX) ? MULTI_ZONES[0] : n;
            if(g_live_tune) snapshot_active();   // tune the newly selected voice
            hw.PrintLine("ok select=%d %s", n, g_bank[n].name);
        } else hw.PrintLine("err select 0..%d", NUM_VOICES - 1);
        return;
    }
    if(!strcmp(cmd, "dump"))  { dump_voice(); return; }
    if(!strcmp(cmd, "names")) {
        // List every bank slot's (possibly edited) name so the host can label its
        // voice picker from flash, not just the factory defaults.
        for(int i = 0; i < NUM_VOICES; i++) hw.PrintLine("name %d %s", i, g_bank[i].name);
        hw.PrintLine("end");
        return;
    }
    if(!strcmp(cmd, "mon"))   { char *a = strtok(NULL, " \t"); g_mon = (a && to_i(a));
                                hw.PrintLine("ok mon=%d", g_mon ? 1 : 0); return; }
    if(!strcmp(cmd, "save")) {
        // Commit the live voice (incl. name) into its bank slot, then persist the
        // whole bank to flash — survives power-off.
        int vi = g_active_voice; if(vi < 0 || vi >= NUM_VOICES) vi = 0;
        bank_set(vi, g_live_voice, g_live_name);
        persist_save_bank();
        hw.PrintLine("ok saved %d %s", vi, g_bank[vi].name);
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
        hw.PrintLine("ok factory %s", (a && !strcmp(a, "all")) ? "all" : g_bank[g_active_voice].name);
        return;
    }
    if(!strcmp(cmd, "help")) {
        hw.PrintLine("cmds: tune 0|1 | set <field> <val> | set name <text> | select <n>");
        hw.PrintLine("      dump | save | factory [all] | mon 0|1 | bye");
        for(int i = 0; i < NFIELDS; i++) hw.PrintLine("  %s", FIELDS[i].name);
        hw.PrintLine("  name scale(chromatic|major|minor) reverb_decay reverb_level delay_time_ms delay_feedback delay_level");
        return;
    }
    hw.PrintLine("err unknown cmd '%s'", cmd);
}

void serial_tune_init() {
    hw.usb_handle.SetReceiveCallback(rx_cb, UsbHandle::FS_INTERNAL);
}

void serial_tune_poll() {
    static char line[160];
    static int  li = 0;
    while(s_rx_tail != s_rx_head) {
        char c = s_rx[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1) % RX_SZ);
        // Either CR or LF ends a line — terminals (screen) send '\r' on Enter,
        // others send '\n', and CRLF just yields one empty line we skip (li == 0).
        if(c == '\r' || c == '\n') { line[li] = 0; if(li > 0) handle_command(line); li = 0; }
        else if(li < (int)sizeof(line) - 1) line[li++] = c;
    }
}
