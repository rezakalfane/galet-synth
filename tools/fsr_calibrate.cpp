/**
 * fsr_calibrate.cpp — FSR min/max pressure calibrator on ADC1
 * Daisy Seed
 *
 * Reads an FSR on A1 (ADC1 / D16 / pin 23). After capturing a brief idle
 * baseline, waits for the first touch to begin the 15-second calibration
 * window. Vary your pressure across the full range during that window —
 * release, press lightly, press as hard as you'll ever play, etc.
 *
 * At the end prints both extremes AND speaks them through the audio output
 * ("minimum: 1 2 3 4 5 ; maximum: 6 7 8 9 0"). Speech samples are 8 kHz mono
 * 8-bit PCM baked from macOS `say -v Samantha`, upsampled 6× for the 48 kHz
 * audio engine.
 *
 * Open `screen /dev/tty.usbmodem* 115200` to view text output.
 */

#include "daisy_seed.h"
#include "speech_data.h"

using namespace daisy;

DaisySeed hw;

// ── Audio playback state (set by main, consumed by audio callback) ────────────
// Speech samples: 6 kHz mono 4-bit packed (2 samples per byte, high nibble first).
// Upsample 8× to reach the 48 kHz audio engine.
static volatile const uint8_t* g_play_data    = nullptr;
static volatile uint32_t       g_play_samples = 0;
static volatile uint32_t       g_play_pos     = 0;  // source sample index
static volatile uint32_t       g_play_sub     = 0;  // 0..7 (8× upsample)

void AudioCallback(AudioHandle::InputBuffer,
                   AudioHandle::OutputBuffer out, size_t size)
{
    const uint8_t* data = (const uint8_t*)g_play_data;
    uint32_t       pos  = g_play_pos;
    uint32_t       sub  = g_play_sub;
    uint32_t       n    = g_play_samples;
    for(size_t i = 0; i < size; i++)
    {
        float v = 0.0f;
        if(data)
        {
            uint8_t byte   = data[pos >> 1];
            uint8_t nibble = (pos & 1u) ? (byte & 0x0F) : ((byte >> 4) & 0x0F);
            uint8_t s8     = (uint8_t)(nibble * 17);  // 0..255
            v = ((float)s8 - 127.5f) * (0.6f / 127.5f);
            if(++sub >= 8)
            {
                sub = 0;
                if(++pos >= n) { data = nullptr; }
            }
        }
        out[0][i] = v;
        out[1][i] = v;
    }
    g_play_pos  = pos;
    g_play_sub  = sub;
    g_play_data = data;
}

static void say(const uint8_t* data, uint32_t samples)
{
    g_play_pos     = 0;
    g_play_sub     = 0;
    g_play_samples = samples;
    g_play_data    = data;
    while(g_play_data != nullptr) System::Delay(5);
    System::Delay(120);  // gap between words
}

static void speak_number(uint16_t v)
{
    uint8_t digits[5];
    int     n = 0;
    if(v == 0) { digits[n++] = 0; }
    else { while(v > 0 && n < 5) { digits[n++] = v % 10; v /= 10; } }
    for(int i = n - 1; i >= 0; i--)
        say(SPEECH_DIGITS[digits[i]], SPEECH_DIGITS_SAMPLES[digits[i]]);
}

static constexpr uint32_t CAL_DURATION_MS  = 15000;
static constexpr uint32_t SAMPLE_PERIOD_MS = 20;     // 50 Hz
static constexpr int      BAR_W            = 30;
// Touch detected when the live reading deviates from idle by more than this.
static constexpr int32_t  TOUCH_DELTA      = 2000;   // ADC counts (out of 65535)

static void make_bar(char* out, uint32_t val, uint32_t max_val)
{
    int n = (max_val > 0) ? (int)((val * BAR_W) / max_val) : 0;
    if(n < 0) n = 0; else if(n > BAR_W) n = BAR_W;
    out[0] = '[';
    for(int i = 0; i < BAR_W; i++) out[1 + i] = (i < n) ? '#' : ' ';
    out[1 + BAR_W] = ']';
    out[2 + BAR_W] = '\0';
}

int main()
{
    hw.Init();
    hw.SetAudioBlockSize(4);
    hw.StartLog(false);

    AdcChannelConfig adc_cfg;
    adc_cfg.InitSingle(seed::A1);
    hw.adc.Init(&adc_cfg, 1);
    hw.adc.Start();
    System::Delay(50);

    hw.StartAudio(AudioCallback);

    hw.PrintLine("FSR Min/Max Calibrator  (A1 / D16 / pin 23)");
    hw.PrintLine("===========================================");

    // ── Idle baseline — keep finger OFF ───────────────────────────────────────
    hw.PrintLine("Capturing idle baseline (1s) — finger OFF...");
    System::Delay(500);
    uint32_t base_acc = 0;
    constexpr int BASE_SAMPLES = 50;
    for(int i = 0; i < BASE_SAMPLES; i++){
        base_acc += hw.adc.Get(0);
        System::Delay(20);
    }
    uint16_t idle = (uint16_t)(base_acc / BASE_SAMPLES);
    hw.PrintLine("idle = %u", idle);
    hw.PrintLine("");
    hw.PrintLine("Touch the FSR to start 15s calibration...");

    // ── Wait for first touch ──────────────────────────────────────────────────
    while(true){
        uint16_t r = hw.adc.Get(0);
        int32_t d = (int32_t)r - (int32_t)idle;
        if(d < 0) d = -d;
        if(d >= TOUCH_DELTA) break;
        System::Delay(10);
    }
    hw.PrintLine("GO — vary pressure across the full range for 15s.");
    hw.PrintLine("");

    uint16_t lo = 0xFFFF, hi = 0;
    char     bar[BAR_W + 3];

    uint32_t t_start = System::GetNow();
    uint32_t t_last  = t_start;

    while(true)
    {
        uint32_t now = System::GetNow();
        uint32_t elapsed = now - t_start;
        if(elapsed >= CAL_DURATION_MS) break;

        uint16_t raw = hw.adc.Get(0);
        if(raw < lo) lo = raw;
        if(raw > hi) hi = raw;

        if(now - t_last >= 100u)  // print at 10 Hz
        {
            t_last = now;
            uint32_t remaining_s = (CAL_DURATION_MS - elapsed + 999u) / 1000u;
            make_bar(bar, raw, 65535u);
            hw.PrintLine("t-%2lus  raw:%5u  min:%5u  max:%5u  %s",
                         (unsigned long)remaining_s,
                         raw, lo, hi, bar);
        }

        System::Delay(SAMPLE_PERIOD_MS);
    }

    uint16_t range = (uint16_t)(hi - lo);
    uint32_t lo_mv = ((uint32_t)lo * 3300u) / 65535u;
    uint32_t hi_mv = ((uint32_t)hi * 3300u) / 65535u;

    hw.PrintLine("");
    hw.PrintLine("===========================================");
    hw.PrintLine("RESULT");
    hw.PrintLine("===========================================");
    hw.PrintLine("min raw : %5u  (%lu.%03lu V)",
                 lo,
                 (unsigned long)(lo_mv / 1000), (unsigned long)(lo_mv % 1000));
    hw.PrintLine("max raw : %5u  (%lu.%03lu V)",
                 hi,
                 (unsigned long)(hi_mv / 1000), (unsigned long)(hi_mv % 1000));
    hw.PrintLine("range   : %5u counts", range);
    hw.PrintLine("");
    hw.PrintLine("With pull-up wiring: max = hardest press, min = released.");
    hw.PrintLine("With pull-down wiring: invert (min = hardest press).");
    hw.PrintLine("");

    // ── Speak the results ────────────────────────────────────────────────────
    hw.PrintLine("(speaking results through audio output...)");
    System::Delay(300);
    say(SPEECH_MIN, SPEECH_MIN_SAMPLES);
    speak_number(lo);
    System::Delay(400);
    say(SPEECH_MAX, SPEECH_MAX_SAMPLES);
    speak_number(hi);

    hw.PrintLine("Done. Reset to run again.");
    while(true){}
}
