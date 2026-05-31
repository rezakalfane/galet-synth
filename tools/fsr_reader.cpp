/**
 * fsr_reader.cpp — FSR pressure sensor reader on ADC1
 * Daisy Seed
 *
 * Reads an FSR connected to A1 (ADC1 / D16 / pin 23) and prints raw ADC,
 * voltage, percent, and a bar to the serial monitor at ~10 Hz.
 *
 * Typical wiring (voltage divider, higher pressure → higher reading):
 *   3.3V ── FSR ── A1 ── 10kΩ ── GND
 *
 * Inverted (higher pressure → lower reading):
 *   3.3V ── 10kΩ ── A1 ── FSR ── GND
 *
 * Open `screen /dev/tty.usbmodem* 115200` to view.
 */

#include "daisy_seed.h"

using namespace daisy;

DaisySeed hw;

static constexpr int BAR_W = 30;

int main()
{
    hw.Init();
    hw.StartLog(false);  // false = don't wait for serial monitor

    AdcChannelConfig adc_cfg;
    adc_cfg.InitSingle(seed::A1);
    hw.adc.Init(&adc_cfg, 1);
    hw.adc.Start();

    hw.PrintLine("FSR Pressure Reader  (A1 / D16 / pin 23)");
    hw.PrintLine("========================================");
    hw.PrintLine("");

    uint16_t lo = 0xFFFF, hi = 0;
    char     bar[BAR_W + 3];

    while (true)
    {
        uint16_t raw   = hw.adc.Get(0);
        if (raw < lo) lo = raw;
        if (raw > hi) hi = raw;

        // 0.000 V .. 3.300 V — manual 3-decimal format (no float in PrintLine)
        uint32_t mv  = ((uint32_t)raw * 3300u) / 65535u; // millivolts
        int      pct = (int)(((uint32_t)raw * 100u) / 65535u);

        int n = (pct * BAR_W) / 100;
        if (n < 0) n = 0; else if (n > BAR_W) n = BAR_W;
        bar[0] = '[';
        for (int i = 0; i < BAR_W; i++) bar[1 + i] = (i < n) ? '#' : ' ';
        bar[1 + BAR_W] = ']';
        bar[2 + BAR_W] = '\0';

        hw.PrintLine("raw:%5u  %d.%03dV  %3d%%  %s   (min:%5u max:%5u)",
                     raw,
                     (int)(mv / 1000), (int)(mv % 1000),
                     pct, bar, lo, hi);

        System::Delay(100);
    }
}
