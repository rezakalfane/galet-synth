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
import time

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
    def __init__(self, port_pref=None):
        super().__init__()
        self.port_pref = port_pref      # explicit --port, or None to auto-detect
        self.link = None
        self.bridge = Bridge()
        self.bridge.line.connect(self.append_log)
        self.setWindowTitle("GaletSynth Voice Tuner — connecting…")
        self.rows = {}
        self._block = None   # accumulates the current mon status frame (or None)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # ── Top bar: voice picker + name + actions ──
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Voice:"))
        self.voice = QtWidgets.QComboBox()
        self.voice.addItems([f"{i}: {n}" for i, n in enumerate(schema.VOICE_NAMES)])
        self.voice.activated.connect(self._select_voice)
        top.addWidget(self.voice)
        top.addWidget(QtWidgets.QLabel("Name:"))
        self.name = QtWidgets.QLineEdit()
        self.name.setMaxLength(23)
        self.name.setFixedWidth(140)
        self.name.editingFinished.connect(self._rename)
        top.addWidget(self.name)
        for label, slot in (("Refresh", self.refresh), ("Save to flash", self._save),
                            ("Revert", self._revert), ("Revert all", self._revert_all),
                            ("Export…", self.export)):
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(slot)
            top.addWidget(b)
        self.mon = QtWidgets.QCheckBox("mon")
        self.mon.toggled.connect(self._toggle_mon)
        top.addWidget(self.mon)
        top.addStretch(1)
        root.addLayout(top)

        # ── Grouped field columns (see everything at once) ──
        # Per-field metadata lookup: voice fields (from SPEC) + globals.
        meta = {n: (t, lo, hi) for n, t, lo, hi, gui in schema.SPEC if gui}
        for n, lo, hi in schema.GLOBAL_SPEC:
            meta[n] = ("f", lo, hi)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QtWidgets.QWidget()
        NCOL = 3
        cols = [QtWidgets.QVBoxLayout() for _ in range(NCOL)]
        load = [0] * NCOL   # rough row count per column, for balancing
        for title, fields in schema.GROUPS:
            box = QtWidgets.QGroupBox(title)
            gform = QtWidgets.QFormLayout(box)
            gform.setLabelAlignment(QtCore.Qt.AlignRight)
            gform.setContentsMargins(8, 6, 8, 6)
            gform.setVerticalSpacing(4)
            for field in fields:
                typ, lo, hi = meta[field]
                row = FieldRow(field, typ, lo, hi, self._send)
                self.rows[field] = row
                gform.addRow(field, row.widget)
            c = load.index(min(load))           # drop into the shortest column
            cols[c].addWidget(box)
            load[c] += len(fields) + 1
        colrow = QtWidgets.QHBoxLayout(inner)
        for col in cols:
            col.addStretch(1)
            holder = QtWidgets.QWidget()
            holder.setLayout(col)
            colrow.addWidget(holder)
        scroll.setWidget(inner)

        # ── Log pane — in a draggable splitter so it's freely resizable ──
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.log.setMinimumHeight(36)
        # Monospace so the mon status bars/columns line up.
        self.log.setFont(QtWidgets.QApplication.font())
        f = self.log.font(); f.setFamily("Menlo"); f.setStyleHint(f.StyleHint.Monospace)
        self.log.setFont(f)

        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.splitter.addWidget(scroll)
        self.splitter.addWidget(self.log)
        self.splitter.setStretchFactor(0, 1)   # fields take the extra space
        self.splitter.setStretchFactor(1, 0)
        self.splitter.setChildrenCollapsible(False)
        root.addWidget(self.splitter, 1)

        self.resize(1080, 760)
        self.splitter.setSizes([640, 120])
        # Connection watchdog: connect now, and reconnect if the synth drops (e.g.
        # power-cycled while saving/testing) so the GUI re-syncs to the loaded
        # voice instead of crashing on a dead port.
        self._watch = QtCore.QTimer(self)
        self._watch.timeout.connect(self._watchdog)
        self._watch.start(1000)
        QtCore.QTimer.singleShot(150, self._watchdog)

    # ── Connection ──
    def _watchdog(self):
        if not (self.link and self.link.alive):
            self._connect()

    def _connect(self):
        port = self.port_pref or find_port()
        if not port:
            self.setWindowTitle("GaletSynth Voice Tuner — no device")
            return
        try:
            self.link = Link(port, on_line=self.bridge.line.emit)
        except serial.SerialException:
            self.link = None
            return
        self.setWindowTitle(f"GaletSynth Voice Tuner — {port}")
        self.append_log(f"connected {port}")
        self.link.send("tune 1")                       # tune mode: edits take effect
        # Sync to the loaded voice, retrying until the device answers (the first
        # dump right after connect/enumeration often races) — otherwise the GUI is
        # left on the default voice with an empty name.
        QtCore.QTimer.singleShot(250, lambda: self._sync_retry(8))

    def _sync_retry(self, tries):
        if not (self.link and self.link.alive):
            return
        if self.refresh():
            self._load_names()      # label the whole picker from flash, not factory
            return
        if tries > 0:
            QtCore.QTimer.singleShot(400, lambda: self._sync_retry(tries - 1))

    def _load_names(self):
        """Label every picker entry with the device's (possibly edited) bank names
        so switching voices shows the saved names, not just the factory defaults."""
        if not (self.link and self.link.alive):
            return
        for i, n in self.link.names().items():
            if 0 <= i < self.voice.count():
                self.voice.setItemText(i, f"{i}: {n}")

    def _require_link(self):
        if self.link and self.link.alive:
            return True
        self.append_log("! not connected")
        return False

    def _send(self, s):
        """Stable sender for the field rows — safe before connect / after a drop
        (the link is created/replaced by the watchdog, not at row-build time)."""
        if self.link and self.link.alive:
            self.link.send(s)

    def _log_lines_height(self, lines):
        """Pixel height that shows exactly `lines` rows of the log font."""
        fm = self.log.fontMetrics()
        return int(fm.lineSpacing() * lines) + 14

    def _fit_log(self, lines):
        """Resize the splitter so the log shows ~`lines` rows (smoothly, leaving
        the field area the rest). The handle stays draggable afterward."""
        total = self.splitter.size().height()
        if total <= 0:
            total = self.height()
        h = min(self._log_lines_height(lines), max(total - 160, 80))
        self.splitter.setSizes([max(total - h, 120), h])

    def _toggle_mon(self, on):
        if self.link and self.link.alive:
            self.link.send(f"mon {1 if on else 0}")
        self._block = None
        if on:
            self.log.clear()
        # The firmware status frame is ~22 lines; size the pane to fit it when mon
        # is on, shrink back to a few log lines when off.
        self._fit_log(24 if on else 6)

    def _select_voice(self, idx):
        if not self._require_link():
            return
        self.link.send(f"select {idx}")
        QtCore.QTimer.singleShot(120, self.refresh)

    def _rename(self):
        if not self._require_link():
            return
        nm = self.name.text().strip()
        self.link.send(f"set name {nm}")
        i = self.voice.currentIndex()
        self.voice.setItemText(i, f"{i}: {nm}")        # reflect the new name in the picker

    def _save(self):
        if not self._require_link():
            return
        # Push the current name first — don't rely on the line-edit's
        # editingFinished having fired (clicking a button doesn't reliably emit
        # it before the click handler on macOS), or `save` commits the stale
        # live name. The two lines are parsed FIFO, so `set name` lands first.
        nm = self.name.text().strip()
        self.link.send(f"set name {nm}")
        self.link.send("save")
        i = self.voice.currentIndex()
        self.voice.setItemText(i, f"{i}: {nm}")
        self.append_log(f"saved → flash ({nm})")

    def _revert(self):
        if not self._require_link():
            return
        if QtWidgets.QMessageBox.question(
                self, "Revert to factory",
                f"Reset '{self.name.text().strip()}' to its source default and "
                f"overwrite the saved version in flash?") != QtWidgets.QMessageBox.Yes:
            return
        self.link.send("factory")
        # Repaint the name immediately from the known factory name (the device is
        # busy writing flash, so a refresh can race); then reload params.
        i = self.voice.currentIndex()
        if 0 <= i < len(schema.VOICE_NAMES):
            self.name.setText(schema.VOICE_NAMES[i])
            self.voice.setItemText(i, f"{i}: {schema.VOICE_NAMES[i]}")
        QtCore.QTimer.singleShot(300, self.refresh)

    def _revert_all(self):
        if not self._require_link():
            return
        if QtWidgets.QMessageBox.question(
                self, "Revert ALL to factory",
                "Reset every voice in the bank to its source default and wipe all "
                "saved edits in flash?") != QtWidgets.QMessageBox.Yes:
            return
        self.link.send("factory all")
        for i, n in enumerate(schema.VOICE_NAMES):   # repaint every label to factory
            self.voice.setItemText(i, f"{i}: {n}")
        i = self.voice.currentIndex()
        if 0 <= i < len(schema.VOICE_NAMES):
            self.name.setText(schema.VOICE_NAMES[i])
        QtCore.QTimer.singleShot(300, self.refresh)

    def _on_hw_voice(self, idx):
        """A voice change made on the hardware (FSR gesture) — sync the picker and
        reload that voice's parameters."""
        if not (0 <= idx < self.voice.count()):
            return
        if idx != self.voice.currentIndex():
            self.voice.blockSignals(True)
            self.voice.setCurrentIndex(idx)
            self.voice.blockSignals(False)
        self.append_log(f"hardware → voice {idx}")
        self.refresh()

    def refresh(self):
        # Reload the *current* voice's parameters from the device. `tune 1`
        # re-snapshots the active bank voice into the live working copy (reverting
        # any live edits to that voice's stored preset); then pull the values into
        # the controls. Without the re-snapshot, dump would just echo back the
        # edits already shown — which is why Refresh looked like a no-op.
        if not self._require_link():
            return False
        self.link.send("tune 1")
        time.sleep(0.06)            # let the snapshot apply + the reply drain
        kv = self.link.dump()
        if not kv:
            self.append_log("! no dump (is the synth connected?)")
            return False
        for row in self.rows.values():
            row.load(kv)
        name = kv.get("name", "")
        self.name.setText(name)        # load the editable name (no signal on setText)
        # Sync the picker to the device's reported slot. Prefer the index (works
        # even for renamed voices); fall back to matching the factory name.
        idx = self.voice.currentIndex()
        if kv.get("idx", "").lstrip("-").isdigit():
            idx = int(kv["idx"])
        elif name in schema.VOICE_NAMES:
            idx = schema.VOICE_NAMES.index(name)
        if 0 <= idx < self.voice.count():
            self.voice.blockSignals(True)
            self.voice.setCurrentIndex(idx)
            self.voice.blockSignals(False)
            self.voice.setItemText(idx, f"{idx}: {name}")   # reflect (possibly renamed) name
        self.append_log(f"reloaded {name}" if name else "reloaded")
        return True

    def export(self):
        if not self._require_link():
            return
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
        # Hardware voice-change event ("voice <n>") from the FSR gesture on the
        # Galet — follow it: switch the picker and reload that voice's params.
        if s.startswith("voice ") and s[6:].strip().isdigit():
            self._on_hw_voice(int(s[6:].strip()))
            return
        # mon status frames start with a "VOICE " header and repeat ~8 Hz. Render
        # each completed frame in place (replace) so it reads as a smooth, stable
        # dashboard instead of an endless scroll; other lines (ok/err, app
        # messages) append normally when no frame is in progress.
        if s.startswith("VOICE "):
            if self._block:
                self.log.setPlainText("\n".join(self._block))
                self.log.verticalScrollBar().setValue(0)
            self._block = [s]
        elif self._block is not None:
            self._block.append(s)
        else:
            self.log.appendPlainText(s)


def main():
    port = (sys.argv[sys.argv.index("--port") + 1]
            if "--port" in sys.argv else None)   # None → auto-detect + reconnect
    app = QtWidgets.QApplication(sys.argv)
    win = Tuner(port)        # owns the link; connects + reconnects via watchdog
    win.show()
    code = app.exec()
    if win.link:
        win.link.close()
    sys.exit(code)


if __name__ == "__main__":
    main()
