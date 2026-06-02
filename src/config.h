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
// Per-LED brightness levels (0..1). Lowering an LED's level cuts its current,
// which reduces the LED noise coupled into the audio (esp. on battery).
//   LED1 = DAC1 (middle), LED2 = DAC2 (pos-0 end), LED3 = PWM on A0 (pos-1 end).
static constexpr float    LED1_LEVEL         = 1.00f;
static constexpr float    LED2_LEVEL         = 1.00f;
static constexpr float    LED3_LEVEL         = 0.50f;
// Derived DAC ceilings (full-scale = 4095) for the two analog LEDs.
static constexpr float    LED1_MAX           = 4095.0f * LED1_LEVEL;
static constexpr float    LED2_MAX           = 4095.0f * LED2_LEVEL;
// LED3 PWM is much brighter per unit duty, so 100% is pre-scaled to 0.1 duty.
static constexpr float    LED3_INTENSITY     = 0.1f * LED3_LEVEL;
// Smoothing coefficient used for LED3 only when no finger is touching. Smaller
// than LED_SMOOTH = faster fade-out on lift. 0.0 = instant snap to 0.
static constexpr float    LED3_RELEASE_SMOOTH = 0.10f;

// ── Musical mapping ───────────────────────────────────────────────────────────
// Master quantize switch. Per-voice quantize (and its scale) live on the Voice
// (Voice::quantize / Voice::scale — only the Organ enables it); this gate must
// ALSO be true for any snapping to happen. False = continuous pitch everywhere.
static constexpr bool QUANTIZE_ENABLED = true;

// Voice-select audio preview. When true, tapping/sliding the glass while cycling
// plays the selected voice live at full pressure (a kick+snare for the Drums kit)
// so you recognize it by ear. Set false for silent cycling — LED feedback only.
static constexpr bool VOICE_PREVIEW_ENABLED = true;
