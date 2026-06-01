#pragma once
// Compile-time tuning constants: touch sensing, LEDs, musical mapping.
#include <cstdint>

static constexpr int      N_CH     = 12;

// ── Touch tuning ──────────────────────────────────────────────────────────────
static constexpr int32_t  TOUCH_THRESHOLD    = 10;
static constexpr int32_t  PRESSURE_MAX_REF[12] = {
    42,35,34,34,34,34,
    34,34,33,32,29,30
};
static constexpr int32_t  MIN_FINGER_SEP     = 4;
static constexpr uint32_t REBASELINE_IDLE_MS = 2000;
static constexpr int      REBASELINE_SAMPLES = 32;
static constexpr int32_t  MAX_POS_JUMP       = 200;

// LED slew at ~60Hz touch loop. Closer to 1.0 = smoother/slower fade.
// 0.0 = no smoothing (snap), 0.7 = ~80ms fade, 0.9 = ~250ms fade.
static constexpr float    LED_SMOOTH         = 0.45f;
// LED3 (software sigma-delta PWM on A0 / ADC0 / pin 22) max-brightness scale.
static constexpr float    LED3_INTENSITY     = 0.1f;
// Smoothing coefficient used for LED3 only when no finger is touching. Smaller
// than LED_SMOOTH = faster fade-out on lift. 0.0 = instant snap to 0.
static constexpr float    LED3_RELEASE_SMOOTH = 0.10f;

// ── Musical mapping ───────────────────────────────────────────────────────────
// Set to false to disable all quantization (fully continuous pitch)
static constexpr bool QUANTIZE_ENABLED = false;
// Quantize to chromatic scale (all 12 semitones)
// Change to {0,2,4,5,7,9,11} for major scale, {0,2,3,5,7,8,10} for minor, etc.
static const int SCALE[] = {0,1,2,3,4,5,6,7,8,9,10,11}; // chromatic
static constexpr int SCALE_LEN = 12;
