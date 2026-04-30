/**
 * osc_test.cpp — Stereo sine wave oscillator test
 * Daisy Seed
 *
 * Outputs 440Hz on both channels.
 * Works with TS (mono) or TRS (stereo) jack.
 */

#include "daisy_seed.h"

using namespace daisy;

DaisySeed hw;

static float phase_l = 0.0f;
static float phase_r = 0.0f;

static inline float sine_approx(float phase)
{
    // Bhaskara approximation, phase 0..1
    float x = phase * 6.28318f;
    if (x < 3.14159f)
        return (16.0f * x * (3.14159f - x)) /
               (49.348f - 4.0f * x * (3.14159f - x));
    else {
        x -= 3.14159f;
        return -((16.0f * x * (3.14159f - x)) /
                 (49.348f - 4.0f * x * (3.14159f - x)));
    }
}

void AudioCallback(AudioHandle::InputBuffer,
                   AudioHandle::OutputBuffer out,
                   size_t size)
{
    float sr    = hw.AudioSampleRate();
    float inc = 440.0f / sr;  // A4

    for (size_t i = 0; i < size; i++)
    {
        float s = sine_approx(phase_l) * 0.9f;
        // Same signal on both channels — works with TS (mono) or TRS (stereo) jack
        out[0][i] = s;
        out[1][i] = s;

        phase_l += inc; if (phase_l >= 1.0f) phase_l -= 1.0f;
    }
}

int main()
{
    hw.Init();
    hw.SetAudioBlockSize(4);
    hw.StartAudio(AudioCallback);

    while (true)
    {
        hw.SetLed(true);  System::Delay(500);
        hw.SetLed(false); System::Delay(500);
    }
}
