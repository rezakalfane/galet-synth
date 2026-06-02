"""Whole-bank backup / restore to a local JSON file.

The firmware keeps an editable, flash-backed voice bank (src/persist.cpp). These
helpers read every editable slot off the device (`select` + `dump`) into a
portable JSON document, and replay one back (`select` + `set` + `save` per slot,
so each lands in flash). Host-side only — no firmware support beyond the
existing tuning protocol.

The Drums meta-voice (schema.MULTI_IDX, the last slot) is skipped: `select`ing it
remaps the engine to the Kick slot, so it can't be addressed directly — and its
fields are an unused placeholder anyway. Reset it with `factory` if needed. The
4 raw drums (Kick/Snare/Tom/Hat) — the sounds the kit actually plays — are
ordinary slots and ARE included.
"""
import json
import time

from . import schema

BANK_FORMAT = "galetsynth-bank"
BANK_VERSION = 1

# Per-voice settable fields (everything the firmware `set` accepts that lives on
# the Voice) + scale. Globals (reverb_*/delay_*) are shared by all voices, so
# they're stored once at the document level, not per voice.
VOICE_FIELDS = [n for n, t, *_ in schema.SPEC if t in ("f", "i", "b", "w")] + ["scale"]


def backup_slots():
    """Bank slots backup/restore touches, in order (every editable slot)."""
    return [i for i in range(len(schema.VOICE_NAMES)) if i != schema.MULTI_IDX]


def backup_bank(link, log=print, progress=None):
    """Read every editable slot off the device into a JSON-able dict.

    `progress(done, total, name)` is called after each slot is read (for a
    progress bar). Leaves the synth on whatever voice it was already showing.
    """
    link.send("tune 1")
    time.sleep(0.05)
    start = link.dump()
    selected = (int(start["idx"]) if start.get("idx", "").lstrip("-").isdigit()
                else 0)

    slots = backup_slots()
    total = len(slots)
    voices, globals_ = [], {}
    for i in slots:
        link.send(f"select {i}")
        time.sleep(0.08)                       # let the snapshot apply
        kv = link.dump()
        if not kv:
            log(f"! slot {i}: no response")
            continue
        if not globals_:                       # globals are identical across slots
            globals_ = {g: kv[g] for g in schema.GLOBALS if g in kv}
        name = kv.get("name", schema.VOICE_NAMES[i])
        voices.append({
            "idx":    i,
            "name":   name,
            "fields": {f: kv[f] for f in VOICE_FIELDS if f in kv},
        })
        if progress:
            progress(len(voices), total, name)

    link.send(f"select {selected}")            # leave the synth where we found it
    return {
        "format":   BANK_FORMAT,
        "version":  BANK_VERSION,
        "selected": selected,
        "globals":  globals_,
        "voices":   voices,
    }


def restore_bank(link, data, log=print, progress=None, only=None, do_globals=True,
                 settle=0.012, save_wait=0.2):
    """Replay a backup dict onto the device, persisting each slot to flash.

    `only` (a set/iterable of voice indices) restricts which slots are written;
    None = every voice in the file. `do_globals` applies the stored reverb/delay
    params. `progress(done, total, name)` is called after each slot is written.
    `settle` paces individual `set` lines so the firmware's small RX ring can't
    overflow; `save_wait` lets each QSPI write finish before the next slot.
    Returns the number of voices written.
    """
    if data.get("format") != BANK_FORMAT:
        raise ValueError(f"not a {BANK_FORMAT} file")
    sel = set(only) if only is not None else None

    link.send("tune 1")
    time.sleep(0.05)
    writable = [v for v in data.get("voices", [])
                if isinstance(v.get("idx"), int) and v["idx"] != schema.MULTI_IDX
                and 0 <= v["idx"] < len(schema.VOICE_NAMES)
                and (sel is None or v["idx"] in sel)]
    total = len(writable)
    n = 0
    for v in writable:
        idx = v["idx"]
        link.send(f"select {idx}")
        time.sleep(settle)
        name = v.get("name", schema.VOICE_NAMES[idx])
        link.send(f"set name {name}")
        time.sleep(settle)
        for f, val in v.get("fields", {}).items():
            if f in schema.SETTABLE:
                link.send(f"set {f} {val}")
                time.sleep(settle)
        link.send("save")                      # commit this slot to flash
        time.sleep(save_wait)
        n += 1
        if progress:
            progress(n, total, name)

    # Globals are live-only (not part of the flash bank) — apply them once.
    if do_globals:
        for g, val in data.get("globals", {}).items():
            if g in schema.GLOBALS:
                link.send(f"set {g} {val}")
                time.sleep(settle)

    sel = data.get("selected", 0)
    if isinstance(sel, int) and 0 <= sel < len(schema.VOICE_NAMES):
        link.send(f"select {sel}")
        time.sleep(settle)
        link.send("save")                      # persist the selected-voice index
        time.sleep(save_wait)
    return n


def write_bank(path, data):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def read_bank(path):
    with open(path) as f:
        return json.load(f)
