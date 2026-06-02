#pragma once
// Live voice tuning over the USB serial port. The Daisy already logs status text
// over USB CDC (write-only); this adds a *receive* path and a tiny line-based
// command protocol so a laptop (or `screen`) can reshape the active voice in real
// time and read its parameters back.
//
// Protocol (ASCII, one command per line, '\n'-terminated). Replies are prefixed
// `ok`/`err` so a host can parse them:
//   tune 1|0            enter/leave tune mode (entering snapshots the active voice)
//   set <field> <val>   set a Voice field (or a global effect param) live
//   select <n>          switch the active voice (0..NUM_VOICES-1)
//   dump                print every field as `field=value`
//   mon 1|0             force the status display on/off while tuning
//   help                list the field names
// `set` understands floats, ints, bools (0/1), waveforms (tri|sine|square|saw)
// and the scale (chromatic|major|minor). freq_low/freq_high also recompute
// log_freq_ratio on-device. Global effect params: reverb_decay, reverb_level,
// delay_time_ms, delay_feedback, delay_level.
//
// The tuning state lives on the engine seam (see engine.h: g_live_voice /
// g_live_tune / g_live_rev) so the audio path can read it without a dependency
// on this module.

// Register the USB CDC receive callback. Call once, after hw.StartLog().
void serial_tune_init();

// Drain the receive ring buffer and execute any complete command lines. Call
// once per control-loop iteration (cheap; does nothing until a line arrives).
void serial_tune_poll();

// True while tuning with the status display suppressed — main.cpp gates its
// periodic status print on this so the protocol channel stays clean (`mon 1`
// re-enables it). Defined in serialtune.cpp.
extern volatile bool g_mon;
bool serial_display_muted();
