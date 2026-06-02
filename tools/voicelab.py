#!/usr/bin/env python3
"""voicelab — live voice tuner for GaletSynth over USB serial.

Talks the line protocol served by src/serialtune.cpp. Lets you reshape the
active voice on the real hardware (with the real glass) and export the result as
a paste-ready C++ Voice{...} block.

    pip install pyserial
    python3 tools/voicelab.py              # auto-detect /dev/tty.usbmodem*
    python3 tools/voicelab.py --port /dev/tty.usbmodemXYZ

REPL commands:
    tune 1|0            enter/leave tune mode (entering snapshots the active voice)
    set <field> <val>   set a voice field or global effect param, live
    select <n>          switch the active voice
    dump                print every field
    mon 1|0             force the status display while tuning
    help                firmware help (field list)
  client-side (handled here, not sent verbatim):
    export [NAME] [file.h]   dump the live voice and emit a C++ Voice block
    save <file.txt>          save the live voice as set-commands you can replay
    load <file.txt>          replay a saved file (one command per line)
    quit / exit              leave

Field names tab-complete. You see what you type (unlike raw `screen`).
"""
import argparse, glob, sys, threading, time, queue

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not installed — run:  pip install pyserial")

try:
    import readline  # tab-completion + line editing (stdlib on macOS/Linux)
except ImportError:
    readline = None

# Voice fields in exact src/voice.h struct order, with their C++ type, for the
# `export` code generator. Keep in sync with the Voice struct. Type codes:
#   f=float (emit with 'f' suffix), i=int, b=bool, w=Waveform, s=scale (special)
STRUCT = [
    ("name", "name"),  # special: string literal
    ("freq_low", "f"), ("freq_high", "f"), ("log_freq_ratio", "f"),
    ("osc1_ratio", "f"), ("osc1_level", "f"), ("osc1_wave", "w"),
    ("osc2_ratio", "f"), ("osc2_level", "f"), ("osc2_wave", "w"),
    ("sub_ratio", "f"), ("sub_level", "f"), ("sub_wave", "w"),
    ("oct_ratio", "f"), ("oct_level", "f"), ("oct_wave", "w"),
    ("osc2_detune", "b"),
    ("cutoff_mult", "f"), ("cutoff_oct_max", "f"), ("resonance", "f"),
    ("drive_max", "f"),
    ("noise_level", "f"), ("keytrack", "f"), ("ringmod_ratio", "f"),
    ("ringmod_max", "f"), ("fold_max", "f"),
    ("attack_ms", "f"), ("release_ms", "f"), ("glide_ms", "f"),
    ("pitch_env_oct", "f"), ("pitch_env_ms", "f"),
    ("noise_hp", "f"), ("no_cycle", "b"), ("vel_sens", "f"),
    ("retrig_ms", "f"), ("quantize", "b"),
    ("scale", "s"),  # special: emits scale + scale_len
    ("decay_ms", "f"), ("sustain", "f"),
    ("chords", "b"), ("chord_level", "f"), ("chord_spread", "i"),
    ("reverb_send", "f"), ("delay_send", "f"),
]
SETTABLE = [n for n, t in STRUCT if t not in ("name", "s")] + ["scale"]
GLOBALS = ["reverb_decay", "reverb_level", "delay_time_ms", "delay_feedback", "delay_level"]
WAVES = {"tri": "WAVE_TRI", "sine": "WAVE_SINE", "square": "WAVE_SQUARE", "saw": "WAVE_SAW"}
SCALES = {"chromatic": ("SCALE_CHROMATIC", 12), "major": ("SCALE_MAJOR", 7), "minor": ("SCALE_MINOR", 7)}


def find_port():
    cands = sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))
    return cands[0] if cands else None


class Link:
    """Serial link with a background reader. Normally prints incoming lines;
    can capture a `dump` response on demand."""
    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.alive = True
        self.cap = None              # list while capturing a dump, else None
        self.cap_done = threading.Event()
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    def _reader(self):
        buf = b""
        while self.alive:
            try:
                buf += self.ser.read(256)
            except Exception:
                break
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip("\r ")
                if not line:
                    continue
                if self.cap is not None:
                    self.cap.append(line)
                    if line == "end" or line.startswith("err"):
                        self.cap_done.set()
                else:
                    sys.stdout.write("\r< " + line + "\n")
                    sys.stdout.flush()

    def send(self, s):
        self.ser.write((s + "\n").encode())

    def dump(self, timeout=2.0):
        self.cap = []
        self.cap_done.clear()
        self.send("dump")
        self.cap_done.wait(timeout)
        lines, self.cap = self.cap, None
        kv = {}
        for ln in lines:
            if ln.startswith("dump name="):
                kv["name"] = ln.split("=", 1)[1]
            elif "=" in ln and ln != "end":
                k, v = ln.split("=", 1)
                kv[k.strip()] = v.strip()
        return kv

    def close(self):
        self.alive = False
        time.sleep(0.15)
        self.ser.close()


def gen_cpp(kv, name):
    """Turn a dumped field map into a C++ Voice{...} positional initializer."""
    out = [f"static constexpr Voice VOICE_{name.upper()} = {{"]
    for field, typ in STRUCT:
        if typ == "name":
            out.append(f'    "{name}",')
            continue
        if typ == "s":
            sc = kv.get("scale", "chromatic")
            cname, clen = SCALES.get(sc, ("SCALE_CHROMATIC", 12))
            out.append(f"    {cname}, {clen},   // scale, scale_len")
            continue
        v = kv.get(field)
        if v is None:
            out.append(f"    /* {field}: MISSING */ 0,")
            continue
        v = v.strip()
        if typ == "f":
            out.append(f"    {float(v):g}f,   // {field}")
        elif typ == "i":
            out.append(f"    {int(float(v))},   // {field}")
        elif typ == "b":
            out.append(f"    {'true' if v not in ('0', '0.0') else 'false'},   // {field}")
        elif typ == "w":
            out.append(f"    {WAVES.get(v, 'WAVE_TRI')},   // {field}")
    out.append("};")
    glob_lines = [f"// globals (set live, not part of the Voice): " +
                  ", ".join(f"{g}={kv[g]}" for g in GLOBALS if g in kv)]
    return "\n".join(out) + "\n" + ("\n".join(glob_lines) if any(g in kv for g in GLOBALS) else "")


def setup_readline():
    if not readline:
        return
    words = (["tune", "set", "select", "dump", "mon", "help", "export", "save", "load", "quit", "exit"]
             + SETTABLE + GLOBALS + list(WAVES) + list(SCALES))

    def completer(text, state):
        opts = [w for w in words if w.startswith(text)]
        return opts[state] if state < len(opts) else None
    readline.set_completer(completer)
    readline.parse_and_bind("tab: complete")


def main():
    ap = argparse.ArgumentParser(description="GaletSynth live voice tuner")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No serial port found. Plug in the Daisy, or pass --port.")
    print(f"voicelab → {port}  (type 'help', or 'quit' to exit)")
    link = Link(port)
    setup_readline()
    # Auto-enter tune mode so edits take effect and the channel is quiet.
    link.send("tune 1")

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            cmd, *rest = line.split()
            if cmd in ("quit", "exit"):
                break
            if cmd == "export":
                name = rest[0] if rest else "tuned"
                kv = link.dump()
                if not kv:
                    print("! no dump received (is tune mode on?)")
                    continue
                cpp = gen_cpp(kv, name)
                fname = next((r for r in rest[1:] if r.endswith((".h", ".txt", ".cpp"))), None)
                if fname:
                    open(fname, "w").write(cpp + "\n")
                    print(f"wrote {fname}")
                else:
                    print(cpp)
                continue
            if cmd == "save":
                if not rest:
                    print("! save <file>"); continue
                kv = link.dump()
                with open(rest[0], "w") as f:
                    for field, _ in STRUCT:
                        if field in ("name",) or field not in kv:
                            continue
                        f.write(f"set {field} {kv[field]}\n")
                    f.write(f"set scale {kv.get('scale','chromatic')}\n")
                    for g in GLOBALS:
                        if g in kv:
                            f.write(f"set {g} {kv[g]}\n")
                print(f"saved {rest[0]}")
                continue
            if cmd == "load":
                if not rest:
                    print("! load <file>"); continue
                for cl in open(rest[0]):
                    cl = cl.strip()
                    if cl and not cl.startswith("#"):
                        link.send(cl); time.sleep(0.01)
                print(f"loaded {rest[0]}")
                continue
            # otherwise: pass straight through to the firmware
            link.send(line)
            time.sleep(0.05)
    finally:
        link.close()
        print("\nbye")


if __name__ == "__main__":
    main()
