#!/usr/bin/env python3
"""Generate the VoiceLab macOS app icon (tools/voicelab.icns).

Draws a synth-voice glyph — a glowing oscillator waveform on a violet→indigo
rounded-square — with PySide6's painter (already a dependency), renders the
macOS iconset sizes, and packs them into an .icns via `iconutil`.

    .venv/bin/python tools/make_icon.py
"""
import math
import os
import subprocess
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")   # no display needed
from PySide6 import QtCore, QtGui                        # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_ICNS = os.path.join(ROOT, "tools", "voicelab.icns")
WORK = os.path.join(ROOT, "build-macapp", "iconset", "VoiceLab.iconset")


def render(size):
    """Render the icon at `size`×`size` into a QImage."""
    img = QtGui.QImage(size, size, QtGui.QImage.Format_ARGB32)
    img.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(img)
    p.setRenderHint(QtGui.QPainter.Antialiasing, True)
    s = float(size)

    # Rounded-square body (macOS icon grid: ~824/1024 with ~22% corner radius).
    pad = s * 0.098
    rect = QtCore.QRectF(pad, pad, s - 2 * pad, s - 2 * pad)
    radius = rect.width() * 0.2237
    body = QtGui.QPainterPath()
    body.addRoundedRect(rect, radius, radius)

    grad = QtGui.QLinearGradient(rect.topLeft(), rect.bottomLeft())
    grad.setColorAt(0.0, QtGui.QColor("#7B45D6"))   # violet
    grad.setColorAt(1.0, QtGui.QColor("#1E0F46"))   # deep indigo
    p.fillPath(body, QtGui.QBrush(grad))

    # Soft top gloss for a bit of depth.
    gloss = QtGui.QLinearGradient(rect.topLeft(),
                                  QtCore.QPointF(rect.left(), rect.center().y()))
    gloss.setColorAt(0.0, QtGui.QColor(255, 255, 255, 42))
    gloss.setColorAt(1.0, QtGui.QColor(255, 255, 255, 0))
    p.fillPath(body, QtGui.QBrush(gloss))

    # Oscillator waveform across the middle (clipped to the body).
    p.setClipPath(body)
    left, right = rect.left() + s * 0.16, rect.right() - s * 0.16
    midy, amp = rect.center().y(), s * 0.165
    wave = QtGui.QPainterPath()
    N = 240
    for i in range(N + 1):
        t = i / N
        x = left + (right - left) * t
        # a sine with a faint second harmonic — a touch of synth "character"
        y = midy - amp * (math.sin(t * 2.0 * 2 * math.pi)
                          + 0.18 * math.sin(t * 4.0 * 2 * math.pi))
        wave.moveTo(x, y) if i == 0 else wave.lineTo(x, y)

    def stroke(color, width):
        pen = QtGui.QPen(color, width, QtCore.Qt.SolidLine,
                         QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin)
        p.setPen(pen)
        p.drawPath(wave)

    stroke(QtGui.QColor(82, 240, 255, 80), s * 0.090)   # cyan glow/halo
    stroke(QtGui.QColor("#B7FBFF"), s * 0.052)          # bright core
    p.end()
    return img


def main():
    master = render(1024)
    os.makedirs(WORK, exist_ok=True)
    # macOS iconset: each logical size at 1x and 2x.
    for sz in (16, 32, 128, 256, 512):
        for scale, suffix in ((1, ""), (2, "@2x")):
            px = sz * scale
            out = os.path.join(WORK, f"icon_{sz}x{sz}{suffix}.png")
            master.scaled(px, px, QtCore.Qt.KeepAspectRatio,
                          QtCore.Qt.SmoothTransformation).save(out, "PNG")
    # Also drop the 1024 master next to the icns for a quick preview.
    master.save(os.path.join(ROOT, "build-macapp", "iconset", "voicelab-1024.png"), "PNG")

    subprocess.run(["iconutil", "-c", "icns", WORK, "-o", OUT_ICNS], check=True)
    print(f"wrote {OUT_ICNS}")


if __name__ == "__main__":
    sys.exit(main())
