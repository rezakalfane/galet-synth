#!/usr/bin/env python3
"""voicelab — command-line live voice tuner for GaletSynth.

A thin REPL over the shared `galetsynth` core (serial link + field schema + C++
export). The PySide6 GUI (voicelab_gui.py) sits on the same core, so the protocol
and codegen live in exactly one place.

    pip install pyserial
    python3 tools/voicelab.py                 # auto-detect /dev/tty.usbmodem*
    python3 tools/voicelab.py --port /dev/...

REPL — firmware commands pass straight through:
    tune 1|0 | set <field> <val> | select <n> | dump | mon 1|0 | help
client-side commands (handled here):
    export [NAME] [file.h]   dump the live voice → C++ Voice{...} block
    save <file.txt>          save as replayable set-commands
    load <file.txt>          replay a saved file
    quit / exit
Field names tab-complete; you see what you type.
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])  # find the galetsynth package
from galetsynth import Link, find_port, gen_cpp           # noqa: E402
from galetsynth import schema                             # noqa: E402

try:
    import readline
except ImportError:
    readline = None

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed — run:  pip install pyserial")


def setup_readline():
    if not readline:
        return
    words = (["tune", "set", "select", "dump", "mon", "help", "export", "save",
              "load", "factory", "name", "quit", "exit"]
             + schema.SETTABLE + list(schema.WAVES) + list(schema.SCALES))

    def completer(text, state):
        opts = [w for w in words if w.startswith(text)]
        return opts[state] if state < len(opts) else None
    readline.set_completer(completer)
    readline.parse_and_bind("tab: complete")


def main():
    ap = argparse.ArgumentParser(description="GaletSynth live voice tuner (CLI)")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No serial port found. Plug in the Daisy, or pass --port.")
    print(f"voicelab → {port}  (type 'help', or 'quit' to exit)")
    try:
        link = Link(port, on_line=lambda s: print("\r< " + s))
    except serial.SerialException as e:
        if "busy" in str(e).lower() or "resource" in str(e).lower():
            sys.exit(f"Port {port} is busy — close any serial monitor "
                     f"(in screen: Ctrl-A K y) and try again.")
        sys.exit(f"Could not open {port}: {e}")

    setup_readline()
    link.send("tune 1")  # auto-enter tune mode (edits take effect; channel quiet)

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
                    with open(fname, "w") as f:
                        f.write(cpp + "\n")
                    print(f"wrote {fname}")
                else:
                    print(cpp)
                continue

            if cmd == "save":
                if not rest:
                    link.send("save")      # no arg → persist the bank to flash (firmware)
                    time.sleep(0.05)
                    continue
                kv = link.dump()
                with open(rest[0], "w") as f:
                    for field, typ, *_ in schema.SPEC:
                        if typ in ("name", "derived", "s") or field not in kv:
                            continue
                        f.write(f"set {field} {kv[field]}\n")
                    f.write(f"set scale {kv.get('scale', 'chromatic')}\n")
                    for g in schema.GLOBALS:
                        if g in kv:
                            f.write(f"set {g} {kv[g]}\n")
                print(f"saved {rest[0]}")
                continue

            if cmd == "load":
                if not rest:
                    print("! load <file>")
                    continue
                for cl in open(rest[0]):
                    cl = cl.strip()
                    if cl and not cl.startswith("#"):
                        link.send(cl)
                        time.sleep(0.01)
                print(f"loaded {rest[0]}")
                continue

            link.send(line)  # pass through to firmware
            time.sleep(0.05)
    finally:
        link.close()
        print("\nbye")


if __name__ == "__main__":
    main()
