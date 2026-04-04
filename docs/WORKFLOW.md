# Development Workflow

## First-time setup

```bash
# Clone libDaisy alongside this repo
git clone https://github.com/electro-smith/libDaisy ../libDaisy
cd ../libDaisy && make && cd -

# Build the synth
make

# Flash (hold BOOT button, press RESET, release BOOT, then:)
make program-dfu
```

## Debugging a new glass / electrode setup

1. **Verify I2C** — flash `tools/i2c_scanner.cpp`. Should report `FOUND: 0x5A`.

2. **Check raw signal** — flash `tools/mpr121_reader.cpp`. Touch each electrode and
   check that deltas appear. Note peak delta at firm press.

3. **Run sensitivity sweep** — flash `tools/mpr121_glass_tuner.cpp`. Follow prompts.
   Note best CDC/CDT combination and update CONFIG1/CONFIG2 in `src/main.cpp`.

4. **Run FFI/ESI sweep** — flash `tools/mpr121_glass_tuner2.cpp` with winning CDC/CDT.
   Note best FFI/ESI and update CONFIG1/CONFIG2.

5. **Calibrate pressure** — flash `tools/mpr121_reader.cpp` again. Record:
   - Minimum delta at touch detection → set `TOUCH_THRESHOLD` and `PRESSURE_DEAD_ZONE`
   - Maximum delta at firm press → set `PRESSURE_MAX_REF`

6. **Verify audio** — flash `tools/osc_test.cpp`. Should hear 440 Hz tone.

7. **Build synth** — `make`, flash, play.

## Switching between tools

```bash
make TARGET=tools/i2c_scanner && make program-dfu
make TARGET=tools/mpr121_reader && make program-dfu
make TARGET=tools/osc_test && make program-dfu
make                           && make program-dfu   # back to synth
```
