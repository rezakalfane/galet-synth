#!/usr/bin/env python3
"""voicelab_gui — PySide6 live voice tuner for GaletSynth.

A desktop window with a control per Voice field. Sits on the same `galetsynth`
core as the CLI (voicelab.py), so the serial protocol, the field schema and the
C++ exporter are never duplicated. Edits go to the real hardware live; play the
glass to hear them. Export writes a paste-ready C++ Voice{...} block.

    pip install pyserial PySide6
    python3 tools/voicelab_gui.py            # auto-detect /dev/tty.usbmodem*
"""
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])  # find the galetsynth package
from galetsynth import Link, find_port, gen_cpp          # noqa: E402
from galetsynth import schema                            # noqa: E402

try:
    import serial
    from PySide6 import QtCore, QtWidgets
except ImportError as e:
    sys.exit(f"Missing dependency ({e}). Run:  pip install pyserial PySide6")


class Bridge(QtCore.QObject):
    """Marshals reader-thread lines onto the GUI thread."""
    line = QtCore.Signal(str)


class FieldRow:
    """One editable control for a SPEC field. Knows how to push its value as a
    `set` command and how to load a value from a dump without echoing back."""
    def __init__(self, field, typ, lo, hi, send):
        self.field, self.typ, self.lo, self.hi, self.send = field, typ, lo, hi, send
        self._loading = False
        if typ in ("f", "i"):
            self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.slider.setRange(0, 1000)
            if typ == "i":
                self.spin = QtWidgets.QSpinBox()
                self.spin.setRange(int(lo), int(hi))
            else:
                self.spin = QtWidgets.QDoubleSpinBox()
                self.spin.setRange(lo, hi)
                self.spin.setDecimals(4)
                self.spin.setSingleStep(max((hi - lo) / 200.0, 0.0001))
            self.spin.setFixedWidth(90)
            self.slider.valueChanged.connect(self._slider_moved)
            self.spin.valueChanged.connect(self._spin_changed)
            w = QtWidgets.QWidget()
            lay = QtWidgets.QHBoxLayout(w)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.addWidget(self.slider, 1)
            lay.addWidget(self.spin)
            self.widget = w
        elif typ == "b":
            self.check = QtWidgets.QCheckBox()
            self.check.stateChanged.connect(self._fire)
            self.widget = self.check
        elif typ in ("w", "s"):
            self.combo = QtWidgets.QComboBox()
            self.combo.addItems(list(schema.WAVES if typ == "w" else schema.SCALES))
            self.combo.currentTextChanged.connect(self._fire)
            self.widget = self.combo

    # ── value <-> slider mapping for floats/ints ──
    def _to_slider(self, v):
        return int(round((v - self.lo) / (self.hi - self.lo) * 1000)) if self.hi > self.lo else 0

    def _from_slider(self, s):
        return self.lo + (self.hi - self.lo) * s / 1000.0

    def _slider_moved(self, s):
        if self._loading:
            return
        v = self._from_slider(s)
        self._loading = True
        self.spin.setValue(int(v) if self.typ == "i" else v)
        self._loading = False
        self._send(v)

    def _spin_changed(self, v):
        if self._loading:
            return
        self._loading = True
        self.slider.setValue(self._to_slider(float(v)))
        self._loading = False
        self._send(v)

    def _fire(self, *_):
        if self._loading:
            return
        if self.typ == "b":
            self._send(1 if self.check.isChecked() else 0)
        else:
            self._send(self.combo.currentText())

    def _send(self, v):
        self.send(f"set {self.field} {v}")

    def load(self, kv):
        """Set the control from a dump dict, without emitting a `set`."""
        if self.field not in kv and self.typ != "s":
            return
        self._loading = True
        try:
            if self.typ in ("f", "i"):
                v = float(kv[self.field])
                self.spin.setValue(int(v) if self.typ == "i" else v)
                self.slider.setValue(self._to_slider(v))
            elif self.typ == "b":
                self.check.setChecked(kv[self.field] not in ("0", "0.0"))
            elif self.typ == "w":
                self.combo.setCurrentText(kv.get(self.field, "tri"))
            elif self.typ == "s":
                self.combo.setCurrentText(kv.get("scale", "chromatic"))
        finally:
            self._loading = False


class Tuner(QtWidgets.QMainWindow):
    def __init__(self, link):
        super().__init__()
        self.link = link
        self.setWindowTitle(f"GaletSynth Voice Tuner — {link.port}")
        self.rows = {}

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # ── Top bar: voice picker + actions ──
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Voice:"))
        self.voice = QtWidgets.QComboBox()
        self.voice.addItems([f"{i}: {n}" for i, n in enumerate(schema.VOICE_NAMES)])
        self.voice.activated.connect(self._select_voice)
        top.addWidget(self.voice)
        for label, slot in (("Refresh", self.refresh), ("Export…", self.export)):
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(slot)
            top.addWidget(b)
        self.mon = QtWidgets.QCheckBox("mon")
        self.mon.toggled.connect(lambda on: self.link.send(f"mon {1 if on else 0}"))
        top.addWidget(self.mon)
        top.addStretch(1)
        root.addLayout(top)

        # ── Scrollable field grid (voice fields + globals) ──
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(inner)
        for field, typ, lo, hi, gui in schema.SPEC:
            if not gui:
                continue
            row = FieldRow(field, typ, lo, hi, self.link.send)
            self.rows[field] = row
            form.addRow(field, row.widget)
        form.addRow(QtWidgets.QLabel("— global effects —"))
        for field, lo, hi in schema.GLOBAL_SPEC:
            row = FieldRow(field, "f", lo, hi, self.link.send)
            self.rows[field] = row
            form.addRow(field, row.widget)
        scroll.setWidget(inner)
        root.addWidget(scroll, 1)

        # ── Log pane ──
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setFixedHeight(110)
        root.addWidget(self.log)

        self.resize(560, 760)
        QtCore.QTimer.singleShot(300, self.refresh)  # populate once connected

    def _select_voice(self, idx):
        self.link.send(f"select {idx}")
        QtCore.QTimer.singleShot(120, self.refresh)

    def refresh(self):
        kv = self.link.dump()
        if not kv:
            self.append_log("! no dump (is the synth connected / in tune mode?)")
            return
        for row in self.rows.values():
            row.load(kv)
        if "name" in kv:
            self.append_log(f"loaded {kv['name']}")

    def export(self):
        kv = self.link.dump()
        if not kv:
            self.append_log("! no dump to export")
            return
        name = kv.get("name", "tuned").replace("-", "_").replace(" ", "_")
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Voice", f"VOICE_{name.upper()}.h", "C++ header (*.h)")
        if not fn:
            return
        with open(fn, "w") as f:
            f.write(gen_cpp(kv, name) + "\n")
        self.append_log(f"exported → {fn}")

    def append_log(self, s):
        self.log.appendPlainText(s)


def main():
    port = (sys.argv[sys.argv.index("--port") + 1]
            if "--port" in sys.argv else find_port())
    if not port:
        sys.exit("No serial port found. Plug in the Daisy, or pass --port.")

    app = QtWidgets.QApplication(sys.argv)
    bridge = Bridge()
    try:
        link = Link(port, on_line=bridge.line.emit)
    except serial.SerialException as e:
        msg = (f"Port {port} is busy — close any serial monitor (screen: Ctrl-A K y)."
               if "busy" in str(e).lower() or "resource" in str(e).lower()
               else f"Could not open {port}: {e}")
        QtWidgets.QMessageBox.critical(None, "GaletSynth Voice Tuner", msg)
        sys.exit(msg)

    link.send("tune 1")  # enter tune mode so edits take effect + channel is quiet
    win = Tuner(link)
    bridge.line.connect(win.append_log)
    win.show()
    code = app.exec()
    link.close()
    sys.exit(code)


if __name__ == "__main__":
    main()
