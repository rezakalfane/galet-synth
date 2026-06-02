"""Turn a dumped field map into a paste-ready C++ Voice{...} block.

Shared by the CLI's `export` and the GUI's Export button. Walks SPEC in struct
order so the positional initializer matches src/voice.h.
"""
from .schema import SPEC, GLOBAL_SPEC, WAVES, SCALES


def gen_cpp(kv, name):
    out = [f"static constexpr Voice VOICE_{name.upper()} = {{"]
    for field, typ, *_ in SPEC:
        if typ == "name":
            out.append(f'    "{name}",')
        elif typ == "s":
            sc = kv.get("scale", "chromatic")
            cname, clen = SCALES.get(sc, ("SCALE_CHROMATIC", 12))
            out.append(f"    {cname}, {clen},   // scale, scale_len")
        elif field not in kv:
            out.append(f"    /* {field}: MISSING */ 0,")
        else:
            v = kv[field].strip()
            if typ in ("f", "derived"):
                out.append(f"    {float(v):g}f,   // {field}")
            elif typ == "i":
                out.append(f"    {int(float(v))},   // {field}")
            elif typ == "b":
                out.append(f"    {'true' if v not in ('0', '0.0') else 'false'},   // {field}")
            elif typ == "w":
                out.append(f"    {WAVES.get(v, 'WAVE_TRI')},   // {field}")
    out.append("};")

    glob = [f"{g}={kv[g]}" for g, *_ in GLOBAL_SPEC if g in kv]
    block = "\n".join(out)
    if glob:
        block += "\n// globals (live, not part of the Voice): " + ", ".join(glob)
    return block
