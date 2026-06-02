"""galetsynth — shared core for the GaletSynth voice tuners (CLI + GUI).

One protocol (link), one field model (schema), one C++ exporter (codegen), so the
command-line REPL and the PySide6 GUI never duplicate that logic.
"""
from .link import Link, find_port
from .codegen import gen_cpp
from . import schema

__all__ = ["Link", "find_port", "gen_cpp", "schema"]
