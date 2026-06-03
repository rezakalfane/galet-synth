#!/usr/bin/env python3
"""voicelab_gui — PySide6 live voice tuner for GaletSynth.

A desktop window with a control per Voice field. Sits on the same `galetsynth`
core as the CLI (voicelab.py), so the serial protocol, the field schema and the
C++ exporter are never duplicated. Edits go to the real hardware live; play the
glass to hear them. Export writes a paste-ready C++ Voice{...} block.

    pip install pyserial PySide6
    python3 tools/voicelab_gui.py            # auto-detect /dev/tty.usbmodem*
"""
import html
import math
import sys
import threading
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])  # find the galetsynth package
from galetsynth import Link, find_port, gen_cpp          # noqa: E402
from galetsynth import schema, bank                      # noqa: E402

try:
    import serial
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError as e:
    sys.exit(f"Missing dependency ({e}). Run:  pip install pyserial PySide6")


def info_tip(title, body):
    """Format a tooltip as a small rich-text info box: a bold header rule above a
    word-wrapped body. Making the tooltip rich text is what gets Qt to wrap it into
    a tidy rectangle instead of one long line; everything is HTML-escaped because
    several descriptions contain '<' / '>' (e.g. '<1', '>0')."""
    return (f"<div style='max-width:300px'>"
            f"<b>{html.escape(title)}</b>"
            f"<hr style='margin:3px 0'>"
            f"{html.escape(body)}</div>")


# A small monochrome glyph per parameter group, shown in its section title (no
# bundled assets; renders in the theme colour like the toolbar icons). Distinct
# per oscillator. Keyed by schema.GROUPS titles.
GROUP_ICONS = {
    "Range":             "↔",
    "Osc 1":             "∿",
    "Osc 2":             "≈",
    "Sub":               "↓",
    "Octave":            "↑",
    "Filter":            "▽",
    "Drive / Noise":     "▒",
    "Ring / Fold":       "◎",
    "Envelope":          "◢",
    "Pitch Env":         "↗",
    "Dynamics":          "⇅",
    "Quantize / Chords": "♫",
    "Sends":             "↪",
    "Global FX":         "✦",
}


class Bridge(QtCore.QObject):
    """Marshals reader-thread lines onto the GUI thread."""
    line = QtCore.Signal(str)
    # Backup/restore run in a worker thread; these marshal its progress + result
    # back to the GUI thread. progress(done, total, name); finished(op, payload).
    progress = QtCore.Signal(int, int, str)
    finished = QtCore.Signal(str, object)


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

    def value(self):
        """Current control value as a string, in the same shape `dump` produces —
        so the C++ export can be built from the GUI alone, no device needed."""
        if self.typ in ("f", "i"):
            v = self.spin.value()
            return str(int(v)) if self.typ == "i" else f"{v:.4f}"
        if self.typ == "b":
            return "1" if self.check.isChecked() else "0"
        return self.combo.currentText()   # w / s

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


class RestorePicker(QtWidgets.QDialog):
    """Modal: choose which voices (and whether the global FX) to import from a
    backup file. The selected voices overwrite the synth's flash."""
    def __init__(self, data, filename, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Restore — choose voices")
        self.setModal(True)
        lay = QtWidgets.QVBoxLayout(self)
        head = QtWidgets.QLabel(
            f"Import from <b>{html.escape(filename)}</b>.<br>"
            f"<span style='color:#c33'>Checked voices overwrite the synth's "
            f"flash — their saved edits are lost.</span>")
        head.setWordWrap(True)
        lay.addWidget(head)

        # One checkbox per voice (skip the Drums meta-voice — not restorable).
        self._boxes = {}
        listw = QtWidgets.QWidget()
        vb = QtWidgets.QVBoxLayout(listw)
        vb.setContentsMargins(4, 4, 4, 4)
        vb.setSpacing(2)
        for v in data.get("voices", []):
            idx = v.get("idx")
            if not isinstance(idx, int) or idx == schema.MULTI_IDX:
                continue
            cb = QtWidgets.QCheckBox(f"{idx}: {v.get('name', '')}")
            cb.setChecked(True)
            self._boxes[idx] = cb
            vb.addWidget(cb)
        vb.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(listw)
        scroll.setMinimumHeight(280)
        lay.addWidget(scroll)

        sel = QtWidgets.QHBoxLayout()
        for label, on in (("Select all", True), ("Select none", False)):
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(lambda _=False, x=on: self._set_all(x))
            sel.addWidget(b)
        sel.addStretch(1)
        lay.addLayout(sel)

        has_globals = bool(data.get("globals"))
        self.globals_cb = QtWidgets.QCheckBox("Also import global effects (reverb / delay)")
        self.globals_cb.setChecked(has_globals)
        self.globals_cb.setEnabled(has_globals)
        lay.addWidget(self.globals_cb)

        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.button(QtWidgets.QDialogButtonBox.Ok).setText("Import")
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        lay.addWidget(bb)
        self.resize(380, 480)

    def _set_all(self, on):
        for cb in self._boxes.values():
            cb.setChecked(on)

    def selected(self):
        return {idx for idx, cb in self._boxes.items() if cb.isChecked()}

    def do_globals(self):
        return self.globals_cb.isChecked()


class CopyDialog(QtWidgets.QDialog):
    """Choose a target slot and a name for a voice copy."""
    def __init__(self, src_name, targets, parent=None):
        super().__init__(parent)               # targets: list of (idx, label)
        self.setWindowTitle("Copy voice")
        self.setModal(True)
        form = QtWidgets.QFormLayout(self)
        self.combo = QtWidgets.QComboBox()
        self._idx = []
        for idx, label in targets:
            self.combo.addItem(label)
            self._idx.append(idx)
        form.addRow("Copy into:", self.combo)
        self.name = QtWidgets.QLineEdit(src_name)
        self.name.setMaxLength(23)
        self.name.selectAll()                  # easy to type a fresh name
        form.addRow("Name:", self.name)
        bb = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.button(QtWidgets.QDialogButtonBox.Ok).setText("Copy")
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        form.addRow(bb)
        self.resize(340, 120)

    def target(self):
        return self._idx[self.combo.currentIndex()]

    def chosen_name(self):
        return self.name.text().strip()


class Tuner(QtWidgets.QMainWindow):
    def __init__(self, port_pref=None):
        super().__init__()
        self.port_pref = port_pref      # explicit --port, or None to auto-detect
        self.link = None
        self.bridge = Bridge()
        self.bridge.line.connect(self.append_log)
        self.bridge.progress.connect(self._on_progress)
        self.bridge.finished.connect(self._on_finished)
        self.setWindowTitle("GaletSynth Voice Tuner — connecting…")
        self.rows = {}
        self._block = None   # accumulates the current mon status frame (or None)
        self._busy = False   # a backup/restore is in flight (blocks re-entry)
        self._progress = None

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        # ── Top bar: voice picker + name + action toolbar ──
        SP = QtWidgets.QStyle.StandardPixmap

        def icon(theme, sp):
            # Prefer a native theme icon (Linux); fall back to the Qt style's
            # built-in pixmaps (macOS/Windows) so no image assets are bundled.
            ic = QtGui.QIcon.fromTheme(theme)
            return ic if not ic.isNull() else self.style().standardIcon(sp)

        def vsep():
            # 3px of space on each side of the divider line. The margins go on a
            # wrapper — putting a stylesheet on the QFrame itself would suppress
            # the VLine rendering.
            w = QtWidgets.QWidget()
            lay = QtWidgets.QHBoxLayout(w)
            lay.setContentsMargins(3, 0, 3, 0)
            lay.setSpacing(0)
            line = QtWidgets.QFrame()
            line.setFrameShape(QtWidgets.QFrame.VLine)
            line.setFrameShadow(QtWidgets.QFrame.Sunken)
            lay.addWidget(line)
            return w

        top = QtWidgets.QHBoxLayout()
        top.setSpacing(4)
        top.addWidget(QtWidgets.QLabel("Voice:"))
        self.voice = QtWidgets.QComboBox()
        self.voice.addItems([f"{i}: {n}" for i, n in enumerate(schema.VOICE_NAMES)])
        self.voice.activated.connect(self._select_voice)
        self.voice.setToolTip(info_tip(
            "Voice", "Pick which voice to edit — also switches the active voice on "
            "the synth so you hear it on the glass."))
        top.addWidget(self.voice)
        top.addWidget(QtWidgets.QLabel("Name:"))
        self.name = QtWidgets.QLineEdit()
        self.name.setMaxLength(23)
        self.name.setFixedWidth(140)
        self.name.editingFinished.connect(self._rename)
        self.name.setToolTip(info_tip(
            "Name", "Rename the current voice (spaces allowed). "
            "Click “Save to flash” to persist it."))
        top.addWidget(self.name)
        top.addWidget(vsep())

        # Action toolbar — compact icon+text buttons, grouped by feature with a
        # vertical separator between groups: (label, slot, theme icon, style icon, tip).
        groups = [
            [("Refresh", self.refresh, "view-refresh", SP.SP_BrowserReload,
              "Reload this voice's parameters from the synth — re-snapshots the saved "
              "preset, discarding unsaved live edits.")],
            [("Save to flash", self._save, "document-save", SP.SP_DriveHDIcon,
              "Commit this voice (parameters + name) to its bank slot in QSPI flash. "
              "Survives power-off."),
             ("Copy to…", self._copy_to, "edit-copy", SP.SP_FileDialogNewFolder,
              "Duplicate this voice (current edits + name) into another bank slot and "
              "save it to flash — the active voice is left untouched."),
             ("Revert", self._revert, "edit-undo", SP.SP_DialogResetButton,
              "Reset this voice to its factory default and overwrite the saved version "
              "in flash."),
             ("Revert all", self._revert_all, "edit-clear", SP.SP_RestoreDefaultsButton,
              "Reset every voice in the bank to factory defaults and wipe all saved "
              "edits in flash.")],
            [("Backup…", self._backup, "document-save-as", SP.SP_DialogSaveButton,
              "Read the whole bank off the synth and save it to a JSON file "
              "(you confirm the filename at the end)."),
             ("Restore…", self._restore, "document-open", SP.SP_DialogOpenButton,
              "Load a JSON bank backup and write it back to the synth, persisting each "
              "voice to flash.")],
            [("Export…", self.export, "text-x-generic", SP.SP_FileIcon,
              "Export this voice as a paste-ready C++ Voice{…} block for src/voice.h.")],
        ]
        for gi, group in enumerate(groups):
            if gi:
                top.addWidget(vsep())
            for label, slot, theme, sp, tip in group:
                b = QtWidgets.QToolButton()
                b.setText(label)
                b.setIcon(icon(theme, sp))
                b.setIconSize(QtCore.QSize(16, 16))
                b.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
                b.setAutoRaise(True)
                b.clicked.connect(slot)
                b.setToolTip(info_tip(label.rstrip("…"), tip))
                top.addWidget(b)

        top.addSpacing(12)        # extra gap after the Export group …
        top.addStretch(1)
        top.addSpacing(12)        # … and before the mon toggle (holds even when
        self.mon = QtWidgets.QCheckBox("mon")   # the window is narrow and the stretch collapses)
        self.mon.toggled.connect(self._toggle_mon)
        self.mon.setToolTip(info_tip(
            "mon", "Stream the synth's live status dashboard (touch position, "
            "pressure, audio params) into the log below."))
        top.addWidget(self.mon)
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
        for col in cols:
            col.setContentsMargins(0, 0, 0, 0)
            col.setSpacing(14)              # clear gap above each group title
        load = [0] * NCOL   # rough row count per column, for balancing
        for title, fields in schema.GROUPS:
            # A per-group icon + the title; the "ⓘ" marks the section as documented
            # (hovering it, or the box title, shows what the group is for).
            box = QtWidgets.QGroupBox(f"{GROUP_ICONS.get(title, '')}  {title}  ⓘ")
            box.setToolTip(info_tip(title, schema.GROUP_HELP.get(title, "")))
            gform = QtWidgets.QFormLayout(box)
            gform.setLabelAlignment(QtCore.Qt.AlignRight)
            gform.setContentsMargins(8, 6, 8, 6)
            gform.setVerticalSpacing(4)
            for field in fields:
                typ, lo, hi = meta[field]
                row = FieldRow(field, typ, lo, hi, self._send)
                self.rows[field] = row
                tip = info_tip(field, schema.HELP.get(field, ""))
                lbl = QtWidgets.QLabel(field)        # explicit label so it can carry a tooltip
                lbl.setToolTip(tip)
                row.widget.setToolTip(tip)
                gform.addRow(lbl, row.widget)
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

    def _copy_to(self):
        if not self._require_link():
            return
        src = self.voice.currentIndex()
        src_name = self.name.text().strip() or schema.VOICE_NAMES[src]
        # Targets: every slot except this one and the Drums meta-voice.
        targets = [(i, self.voice.itemText(i)) for i in range(self.voice.count())
                   if i != src and i != schema.MULTI_IDX]
        if not targets:
            return
        dlg = CopyDialog(src_name, targets, self)
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return
        dst = dlg.target()
        name = dlg.chosen_name() or src_name
        # The firmware duplicates the live voice (current edits) into the slot under
        # this name + persists, leaving the active voice / source name untouched —
        # so we just relabel the target here.
        self.link.send(f"copy {dst} {name}")
        self.voice.setItemText(dst, f"{dst}: {name}")
        self.append_log(f"copied “{src_name}” → {dst}: {name} (saved to flash)")

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
        # Built straight from the on-screen controls — no device needed (the
        # widgets already hold everything shown). log_freq_ratio is the one Voice
        # field with no control (it's precomputed); derive it from the freq range.
        kv = {f: row.value() for f, row in self.rows.items()}
        try:
            lo, hi = float(kv.get("freq_low", 0)), float(kv.get("freq_high", 0))
            if lo > 0 and hi > 0:
                kv["log_freq_ratio"] = f"{math.log(hi / lo):.6f}"
        except ValueError:
            pass
        name = self.name.text().strip() or "tuned"
        safe = name.replace("-", "_").replace(" ", "_")
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export Voice", f"VOICE_{safe.upper()}.h", "C++ header (*.h)")
        if not fn:
            return
        with open(fn, "w") as f:
            f.write(gen_cpp(kv, name) + "\n")
        self.append_log(f"exported → {fn}")

    # ── Whole-bank backup / restore (JSON) ──
    # Both run on a worker thread (a few seconds of serial traffic) so the window
    # stays live and the progress bar animates; results marshal back via the
    # bridge. A modal QProgressDialog blocks the controls meanwhile.
    def _backup(self):
        if not self._require_link() or self._busy:
            return
        self._busy = True
        self.mon.setChecked(False)            # quiet the channel so dumps are clean
        total = len(bank.backup_slots())
        self._progress = QtWidgets.QProgressDialog(
            "Reading voices from the synth…", None, 0, total, self)
        self._progress.setWindowTitle("Backup")
        self._progress.setWindowModality(QtCore.Qt.WindowModal)
        self._progress.setMinimumDuration(0)
        self._progress.setValue(0)
        threading.Thread(target=self._backup_worker, daemon=True).start()

    def _backup_worker(self):
        try:
            data = bank.backup_bank(
                self.link, log=self.bridge.line.emit,
                progress=lambda d, t, n: self.bridge.progress.emit(d, t, n))
            self.bridge.finished.emit("backup", data)
        except Exception as e:                # noqa: BLE001 — report any failure
            self.bridge.finished.emit("backup", e)

    def _save_backup(self, data):
        # Confirm filename + location at the end, defaulting to a timestamped name.
        default = time.strftime("%Y%m%d%H%M%S-Galet-Backup.json")
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save bank backup", default, "JSON backup (*.json)")
        if not fn:
            self.append_log("backup read OK — not saved (cancelled)")
            return
        bank.write_bank(fn, data)
        self.append_log(f"backed up {len(data['voices'])} voices → {fn}")

    def _restore(self):
        if not self._require_link() or self._busy:
            return
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Restore bank backup", "", "JSON backup (*.json)")
        if not fn:
            return
        try:
            data = bank.read_bank(fn)
        except (OSError, ValueError) as e:
            self.append_log(f"! {e}")
            return
        if data.get("format") != bank.BANK_FORMAT:
            self.append_log(f"! not a {bank.BANK_FORMAT} file")
            return
        # Modal pick list: choose which voices (+ globals) to import.
        dlg = RestorePicker(data, fn.rsplit("/", 1)[-1], self)
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return
        only = dlg.selected()
        if not only:
            self.append_log("restore cancelled (no voices selected)")
            return
        do_globals = dlg.do_globals()
        self._busy = True
        self.mon.setChecked(False)
        self._progress = QtWidgets.QProgressDialog(
            "Writing voices to the synth…", None, 0, len(only), self)
        self._progress.setWindowTitle("Restore")
        self._progress.setWindowModality(QtCore.Qt.WindowModal)
        self._progress.setMinimumDuration(0)
        self._progress.setValue(0)
        threading.Thread(target=self._restore_worker,
                         args=(data, only, do_globals), daemon=True).start()

    def _restore_worker(self, data, only, do_globals):
        try:
            n = bank.restore_bank(
                self.link, data, log=self.bridge.line.emit,
                only=only, do_globals=do_globals,
                progress=lambda d, t, nm: self.bridge.progress.emit(d, t, nm))
            self.bridge.finished.emit("restore", n)
        except Exception as e:                # noqa: BLE001 — report any failure
            self.bridge.finished.emit("restore", e)

    def _on_progress(self, done, total, name):
        if self._progress:
            self._progress.setMaximum(total)
            self._progress.setValue(done)
            self._progress.setLabelText(f"{done}/{total}  {name}")

    def _on_finished(self, op, payload):
        if self._progress:
            self._progress.reset()
            self._progress = None
        self._busy = False
        if isinstance(payload, Exception):
            self.append_log(f"! {op} failed: {payload}")
            return
        if op == "backup":
            self._save_backup(payload)
        elif op == "restore":
            self.append_log(f"restored {payload} voices from flash")
            self._load_names()                # picker labels follow restored names
            self.refresh()                    # reload the current voice's controls

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
    # 4px breathing room inside every tooltip's frame (inline CSS padding isn't
    # honored by Qt's rich-text engine; a QToolTip stylesheet is).
    app.setStyleSheet("QToolTip { padding: 4px; }")
    win = Tuner(port)        # owns the link; connects + reconnects via watchdog
    win.show()
    code = app.exec()
    if win.link:
        win.link.send("bye")      # leave tune mode + go silent so the synth resumes
        time.sleep(0.05)          # normal play and doesn't block on the closed port
        win.link.close()
    sys.exit(code)


if __name__ == "__main__":
    main()
