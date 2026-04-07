# GaletSynth — Claude Code Context

## Project

Glass touch bass synth for Daisy Seed + MPR121 capacitive sensor.
- Main firmware: `src/main.cpp`
- Diagnostic / calibration tools: `tools/*.cpp`

## Build

libDaisy lives at `/Users/username/Workspaces/Daisy/DaisyExamples/libDaisy`.
Always pass all three flags — libDaisy's own Makefile requires them:

```bash
LIBDAISY=/Users/username/Workspaces/Daisy/DaisyExamples/libDaisy

make TARGET=src/main \
  libdaisy_dir=$LIBDAISY \
  SYSTEM_FILES_DIR=$LIBDAISY/core \
  LIBDAISY_DIR=$LIBDAISY
```

For a tool:
```bash
make TARGET=tools/test-slider ...same flags...
```

After `make clean`, always recreate build directories before building:
```bash
mkdir -p build/src build/tools
```

## Flash

Put Daisy in DFU mode (hold BOOT, tap RESET), then:

```bash
dfu-util -a 0 -s 0x08000000:leave -D build/src/main.bin
# or for a tool:
dfu-util -a 0 -s 0x08000000:leave -D build/tools/<name>.bin
```

The `dfu-util: Error during download get_status` at the end is normal — the device booted successfully.

## Serial monitor

```bash
screen /dev/tty.usbmodem* 115200
```

The main synth does **not** print to serial during play (removed to keep the touch loop fast).
Use `tools/test-slider` for live sensor display.

## Known gotcha — Makefile LDFLAGS

The `Makefile` must **not** contain:
```makefile
LDFLAGS = -specs=nano.specs -specs=nosys.specs
```
libDaisy already adds these flags. Duplicating them causes a fatal linker error:
`attempt to rename spec 'link' to already defined spec 'nano_link'`

If this error appears, remove the `LDFLAGS` line from `Makefile`.

## Key constants in src/main.cpp

| Constant | Line ~| Purpose |
|---|---|---|
| `TOUCH_THRESHOLD` | 39 | Min delta to register touch |
| `PRESSURE_MAX_REF[12]` | 40 | Per-electrode 100% pressure delta |
| `PRESSURE_DEAD_ZONE[12]` | 44 | Per-electrode 0% pressure delta |
| `MIN_FINGER_SEP` | 48 | Min electrode gap for two-finger detection |
| `QUANTIZE_ENABLED` | 59 | `false` = fully continuous pitch, no snapping |
| `SLEW_FREQ` | ~323 | Pitch glide speed (higher = slower) |
| `REG_CONFIG1 / CONFIG2` | ~137 | MPR121 AFE registers (FFI, CDC, CDT, ESI) |

## Hardware

- **MCU**: Daisy Seed (STM32H750)
- **Sensor**: MPR121 at I2C address `0x5A`, bit-bang on SDA=D12, SCL=D11
- **Glass**: 3mm soda-lime over copper electrodes

## Branching convention

Feature branches: `feature/<name>` → merge to `main` → delete branch.
Update `CHANGELOG.md` after merging.
