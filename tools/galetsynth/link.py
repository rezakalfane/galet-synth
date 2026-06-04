"""Serial link to the GaletSynth firmware tuning protocol (src/serialtune.cpp).

Shared by the CLI (voicelab.py) and the GUI. Owns the port, a background reader
thread, and the request/response `dump`. Incoming lines that aren't part of a
dump are handed to an `on_line` callback (the CLI prints them; the GUI logs them).
"""
import glob
import threading
import time

try:
    import serial  # pyserial
except ImportError as e:  # pragma: no cover
    raise ImportError("pyserial not installed — run:  pip install pyserial") from e


def find_port():
    """First likely Daisy serial device, or None."""
    cands = sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))
    return cands[0] if cands else None


class Link:
    def __init__(self, port, on_line=None):
        # Raises serial.SerialException if the port is missing/busy.
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.port = port
        self.on_line = on_line or (lambda s: None)
        self.alive = True
        self._cap = None                 # list while capturing a dump, else None
        self._cap_done = threading.Event()
        self._wlock = threading.Lock()
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
                # Drop the line terminator + trailing padding only — KEEP leading
                # spaces: the status dashboard right-aligns its columns with them,
                # and stripping both ends collapsed the alignment.
                line = raw.decode("utf-8", "replace").rstrip("\r\n ")
                if not line:
                    continue
                if self._cap is not None:
                    self._cap.append(line)
                    if line == "end" or line.startswith("err"):
                        self._cap_done.set()
                else:
                    self.on_line(line)

    def send(self, s):
        # Survive a disconnect (e.g. the synth was power-cycled): mark the link
        # dead instead of throwing, so the GUI/CLI can reconnect.
        try:
            with self._wlock:
                self.ser.write((s + "\n").encode())
            return True
        except Exception:
            self.alive = False
            return False

    def _capture(self, cmd, timeout=2.0):
        """Send a command and collect its reply lines up to the `end` sentinel."""
        self._cap = []
        self._cap_done.clear()
        self.send(cmd)
        self._cap_done.wait(timeout)
        lines, self._cap = self._cap, None
        return lines

    def dump(self, timeout=2.0):
        """Send `dump` and parse the field=value reply into a dict.

        The capture window can also pick up late async replies from earlier
        commands (e.g. `ok select=…`, the `tune` ack). The firmware emits a dump
        as one contiguous block led by `dump name=…`, so reset on that marker:
        the returned dict is exactly one dump's fields, never a mix of an `idx`
        from one moment and a `name` from another.

        A USB framing glitch can also merge two replies into one captured line
        (e.g. `osc1_ratio=1.0000\\r$$oct_wave=sine`); split on any embedded CR/LF
        so each `key=value` parses cleanly instead of feeding a `float()` crash
        downstream."""
        kv = {}
        for ln in self._capture("dump", timeout):
            for seg in ln.replace("\r", "\n").split("\n"):
                seg = seg.strip()
                if seg.startswith("dump name="):
                    kv = {"name": seg.split("=", 1)[1]}   # fresh block — drop prior noise
                elif "=" in seg and seg != "end":
                    k, v = seg.split("=", 1)
                    kv[k.strip()] = v.strip()
        return kv

    def names(self, timeout=2.0):
        """Send `names` → {index: name} for every bank slot (names may contain
        spaces, e.g. 'SH-101 min')."""
        out = {}
        for ln in self._capture("names", timeout):
            if ln.startswith("name "):
                p = ln.split(" ", 2)
                if len(p) >= 3 and p[1].isdigit():
                    out[int(p[1])] = p[2]
        return out

    def close(self):
        self.alive = False
        time.sleep(0.15)
        try:
            self.ser.close()
        except Exception:
            pass
