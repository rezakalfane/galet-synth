/**
 * dac_breathing_led.cpp — Breathing LEDs on DAC1, DAC2, and ADC0 (chained)
 * Daisy Seed
 *
 * Three LEDs chained as a rolling wave. Each LED does a rise → hold →
 * fall breath; each new LED starts the moment the previous one reaches
 * 100%.
 *
 *   phase   0    R    2R    3R   3R+H   4R+H=1
 *   LED1    0 → 100 → 100 → 0
 *   LED2         0 → 100 → 100 → 0
 *   LED3              0 → 100 → 100 → 0
 *
 * Outputs:
 *   LED1 → DAC OUT 1 (pin 30, analog 0–3.3V)
 *   LED2 → DAC OUT 2 (pin 31, analog 0–3.3V)
 *   LED3 → A0 / ADC0 (pin 22) — software PWM, GPIO toggled at ~2 kHz
 *
 * Wiring (each LED): anode → output pin → ~330R → cathode → GND.
 */

#include "daisy_seed.h"
#include <math.h>

using namespace daisy;

DaisySeed hw;
GPIO      led3;  // software-PWM LED on A0 (ADC0 / D15 / pin 22)

// ── Tweak these ───────────────────────────────────────────────────────────────
static constexpr float PERIOD_S      = 2.0f;   // full cycle length, seconds
// Per-LED shape: rise (RISE_FRAC), hold at peak (HOLD_FRAC), fall (RISE_FRAC).
// Three chained LEDs → constraint: 4*RISE_FRAC + HOLD_FRAC = 1.0.
// Lower RISE_FRAC = longer bright overlap.
static constexpr float RISE_FRAC     = 0.2f;
static constexpr float HOLD_FRAC     = 1.0f - 4.0f * RISE_FRAC;

// ── Probably leave these alone ────────────────────────────────────────────────
static constexpr float UPDATE_HZ     = 200.0f; // DAC refresh rate
static constexpr float GAMMA         = 0.6f;   // <1 boosts brightness, 1.0 = linear, 2.2 = perceptual
static constexpr uint16_t DAC_MAX    = 4095;   // 12-bit
static constexpr uint32_t PWM_PERIOD_US = 500; // 2 kHz software PWM on LED3
static constexpr float LED3_INTENSITY = 0.15f; // scales LED3 max brightness (1.0 = full)

int main()
{
    hw.Init();

    DacHandle::Config cfg;
    cfg.bitdepth   = DacHandle::BitDepth::BITS_12;
    cfg.buff_state = DacHandle::BufferState::ENABLED;
    cfg.mode       = DacHandle::Mode::POLLING;
    cfg.chn        = DacHandle::Channel::BOTH;  // DAC1 → pin 30, DAC2 → pin 31
    hw.dac.Init(cfg);

    led3.Init(seed::A0, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
    led3.Write(false);

    const float    dt        = 1.0f / UPDATE_HZ;
    const float    phase_inc = dt / PERIOD_S;                       // turns per tick
    const uint32_t total_us  = (uint32_t)(1000000.0f / UPDATE_HZ);  // budget per tick
    const uint32_t pwm_cycles = total_us / PWM_PERIOD_US;           // PWM cycles per tick

    float phase = 0.0f;
    while (true)
    {
        // Each LED: smooth rise → hold at 1.0 → smooth fall.
        // Each successive LED starts the moment the previous one hits its hold.
        //   LED1: rise [0, R],    hold [R, R+H],     fall [R+H, 2R+H]
        //   LED2: rise [R, 2R],   hold [2R, 2R+H],   fall [2R+H, 3R+H]
        //   LED3: rise [2R, 3R],  hold [3R, 3R+H],   fall [3R+H, 4R+H=1]
        const float R = RISE_FRAC;
        const float H = HOLD_FRAC;
        float t1 = 0.0f, t2 = 0.0f, t3 = 0.0f;

        if (phase < R) {
            float p = phase / R;
            t1 = 0.5f - 0.5f * cosf(p * 3.14159f);          // rise
        } else if (phase < R + H) {
            t1 = 1.0f;                                       // hold
        } else if (phase < 2.0f * R + H) {
            float p = (phase - R - H) / R;
            t1 = 0.5f + 0.5f * cosf(p * 3.14159f);          // fall
        }

        if (phase >= R && phase < 2.0f * R) {
            float p = (phase - R) / R;
            t2 = 0.5f - 0.5f * cosf(p * 3.14159f);          // rise
        } else if (phase >= 2.0f * R && phase < 2.0f * R + H) {
            t2 = 1.0f;                                       // hold
        } else if (phase >= 2.0f * R + H && phase < 3.0f * R + H) {
            float p = (phase - 2.0f * R - H) / R;
            t2 = 0.5f + 0.5f * cosf(p * 3.14159f);          // fall
        }

        if (phase >= 2.0f * R && phase < 3.0f * R) {
            float p = (phase - 2.0f * R) / R;
            t3 = 0.5f - 0.5f * cosf(p * 3.14159f);          // rise
        } else if (phase >= 3.0f * R && phase < 3.0f * R + H) {
            t3 = 1.0f;                                       // hold
        } else if (phase >= 3.0f * R + H && phase < 4.0f * R + H) {
            float p = (phase - 3.0f * R - H) / R;
            t3 = 0.5f + 0.5f * cosf(p * 3.14159f);          // fall
        }

        uint16_t v1 = (uint16_t)(powf(t1, GAMMA) * DAC_MAX);
        uint16_t v2 = (uint16_t)(powf(t2, GAMMA) * DAC_MAX);
        hw.dac.WriteValue(DacHandle::Channel::ONE, v1);
        hw.dac.WriteValue(DacHandle::Channel::TWO, v2);

        // LED3: software PWM. Duty = gamma-corrected t3. The busy-loop below
        // also fills the per-tick time budget, replacing System::Delay.
        float    duty   = powf(t3, GAMMA) * LED3_INTENSITY;
        if (duty < 0.0f) duty = 0.0f; else if (duty > 1.0f) duty = 1.0f;
        uint32_t on_us  = (uint32_t)(duty * PWM_PERIOD_US);
        uint32_t off_us = PWM_PERIOD_US - on_us;
        for (uint32_t c = 0; c < pwm_cycles; c++) {
            if (on_us  > 0) { led3.Write(true);  System::DelayUs(on_us);  }
            if (off_us > 0) { led3.Write(false); System::DelayUs(off_us); }
        }

        phase += phase_inc;
        if (phase >= 1.0f) phase -= 1.0f;
    }
}
