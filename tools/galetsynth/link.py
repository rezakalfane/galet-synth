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
                line = raw.decode("utf-8", "replace").strip("\r ")
                if not line:
                    continue
                if self._cap is not None:
                    self._cap.append(line)
                    if line == "end" or line.startswith("err"):
                        self._cap_done.set()
                else:
                    self.on_line(line)

    def send(self, s):
        with self._wlock:
            self.ser.write((s + "\n").encode())

    def dump(self, timeout=2.0):
        """Send `dump` and parse the field=value reply into a dict."""
        self._cap = []
        self._cap_done.clear()
        self.send("dump")
        self._cap_done.wait(timeout)
        lines, self._cap = self._cap, None
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
        try:
            self.ser.close()
        except Exception:
            pass
