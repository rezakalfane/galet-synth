# Tuning Log

Record of parameter changes and their results during development.
Use this as a reference when re-tuning for different glass or electrode configurations.

## Hardware Setup

- MCU: Daisy Seed
- Touch IC: MPR121 at 0x5A
- Glass: 3mm soda-lime (IKEA drinking glass wall)
- Electrodes: 12 copper pads under glass

## MPR121 AFE Sweep Results

Swept CDC (charge current) × CDT (charge time) — best results:

| Config | CDC | CDT | Touch delta | Verdict |
|--------|-----|-----|-------------|---------|
| Best   | 16µA | 16µs | ~35 counts | Good SNR |
| Good   | 16µA | 32µs | ~33 counts | Good |
| Poor   | 32µA+ | any | ~0–2 counts | Over-driven |

FFI/ESI sweep result: FFI=10 iterations, ESI=1ms gave best balance of
speed and stability. Higher FFI added no measurable improvement.

Final AFE config:
- CONFIG1 = 0x50 (FFI=10, CDC=16µA)
- CONFIG2 = 0x6C (CDT=16µs, SFI=18, ESI=1ms)
- Touch threshold: 5 counts
- Release threshold: 2 counts

## Pressure Calibration

Through 3mm glass:
- Touch detection (TOUCH_THRESHOLD): delta = 5 counts
- Typical light touch: delta = 8–12 counts
- Firm press: delta = 18–24 counts
- Maximum observed: delta = 24 counts

Mapping: sqrt curve, dead zone = 5, max ref = 24
→ 0% at delta=5, 100% at delta=24

## Parameter History

| Date | Parameter | Old | New | Reason |
|------|-----------|-----|-----|--------|
| — | TOUCH_THRESHOLD | 10 | 5 | Better sensitivity through glass |
| — | PRESSURE_MAX_REF | 40 | 24 | Glass signal physically caps at ~24 counts |
| — | SLEW_AMP_R | 0.9940 | 0.999971 | Eliminate release clicks, 5s tail |
| — | SLEW_AMP_A | 0.8000 | 0.9700 | Eliminate onset clicks |
| — | Resonance | 0.30 | 0.75 | Audible Moog filter peak |
| — | Cutoff sweep | 6 oct | 12 oct | More dramatic expression |
| — | Pressure curve | linear | sqrt | More control at low pressure |
| — | Cutoff curve | power-3 | smootherstep | Smoother sweep feel |

## Notes

- D12=SDA, D11=SCL (not the other way around — caused NAK issues)
- StartLog(false) required for standalone operation without USB serial
- MPR121 init must happen BEFORE StartAudio() to avoid I2C timing corruption
- Software baseline (64-sample average) more reliable than hardware baseline register
  for glass applications — hardware baseline drifts and gives wrong deltas
