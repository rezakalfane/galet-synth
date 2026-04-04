/**
 * mpr121_glass_tuner.cpp — MPR121 Sensitivity Tuner for Glass-Covered Electrodes
 * Daisy Seed — Bit-Bang I2C  (SDA=D12, SCL=D11)
 *
 * Sweeps CDC (charge current) and CDT (charge time) combinations,
 * measures SNR through 3mm glass, and recommends the best config.
 *
 * Usage:
 *   1. Flash and open serial monitor
 *   2. For each config: keep finger OFF during baseline, then press firmly
 *   3. Read recommended registers at the end
 */

#include "daisy_seed.h"

using namespace daisy;
using namespace daisy::seed;

static constexpr Pin     SDA_PIN  = D12;
static constexpr Pin     SCL_PIN  = D11;
static constexpr uint8_t MPR_ADDR = 0x5A;
static constexpr uint32_t HP      = 5;   // I2C half-period µs

// ── MPR121 registers ──────────────────────────────────────────────────────────
static constexpr uint8_t REG_ELE0_LSB  = 0x04;
static constexpr uint8_t REG_BASE_ELE0 = 0x1E;
static constexpr uint8_t REG_MHDR      = 0x2B;
static constexpr uint8_t REG_NHDR      = 0x2C;
static constexpr uint8_t REG_NCLR      = 0x2D;
static constexpr uint8_t REG_FDLR      = 0x2E;
static constexpr uint8_t REG_MHDF      = 0x2F;
static constexpr uint8_t REG_NHDF      = 0x30;
static constexpr uint8_t REG_NCLF      = 0x31;
static constexpr uint8_t REG_FDLF      = 0x32;
static constexpr uint8_t REG_NHDT      = 0x33;
static constexpr uint8_t REG_NCLT      = 0x34;
static constexpr uint8_t REG_FDLT      = 0x35;
static constexpr uint8_t REG_DEBOUNCE  = 0x5B;
static constexpr uint8_t REG_CONFIG1   = 0x5C;
static constexpr uint8_t REG_CONFIG2   = 0x5D;
static constexpr uint8_t REG_ECR       = 0x5E;
static constexpr uint8_t REG_SOFTRESET = 0x80;

// ── Globals ───────────────────────────────────────────────────────────────────
DaisySeed hw;
GPIO      sda, scl;

// ── Bit-bang I2C ──────────────────────────────────────────────────────────────
inline void sda_high() { sda.Init(SDA_PIN, GPIO::Mode::INPUT,  GPIO::Pull::PULLUP); }
inline void sda_low()  { sda.Init(SDA_PIN, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL); sda.Write(false); }
inline void scl_high() { scl.Init(SCL_PIN, GPIO::Mode::INPUT,  GPIO::Pull::PULLUP); }
inline void scl_low()  { scl.Init(SCL_PIN, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL); scl.Write(false); }
inline void d()        { System::DelayUs(HP); }

void i2c_start() { sda_high();d(); scl_high();d(); sda_low();d(); scl_low();d(); }
void i2c_stop()  { sda_low();d();  scl_high();d(); sda_high();d(); }

bool i2c_write_byte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--)
    {
        if (byte & (1<<i)) sda_high(); else sda_low();
        d(); scl_high(); d(); scl_low(); d();
    }
    sda_high(); d(); scl_high(); d();
    bool ack = !sda.Read();
    scl_low(); d();
    return ack;
}

uint8_t i2c_read_byte(bool send_ack)
{
    uint8_t byte = 0;
    sda_high();
    for (int i = 7; i >= 0; i--)
    {
        d(); scl_high(); d();
        if (sda.Read()) byte |= (1<<i);
        scl_low();
    }
    if (send_ack) sda_low(); else sda_high();
    d(); scl_high(); d(); scl_low(); d(); sda_high();
    return byte;
}

bool mpr_write(uint8_t reg, uint8_t val)
{
    i2c_start();
    if (!i2c_write_byte((MPR_ADDR<<1)|0)) { i2c_stop(); return false; }
    if (!i2c_write_byte(reg))             { i2c_stop(); return false; }
    if (!i2c_write_byte(val))             { i2c_stop(); return false; }
    i2c_stop(); return true;
}

bool mpr_read(uint8_t reg, uint8_t* buf, uint8_t len)
{
    i2c_start();
    if (!i2c_write_byte((MPR_ADDR<<1)|0)) { i2c_stop(); return false; }
    if (!i2c_write_byte(reg))             { i2c_stop(); return false; }
    i2c_start();
    if (!i2c_write_byte((MPR_ADDR<<1)|1)) { i2c_stop(); return false; }
    for (uint8_t i = 0; i < len; i++)
        buf[i] = i2c_read_byte(i < len-1);
    i2c_stop(); return true;
}

// ── MPR121 init with given CDC and CDT ────────────────────────────────────────
// CONFIG1 bits[7:6] = FFI (first filter iterations): 11 = 34 iters (max)
// CONFIG1 bits[5:0] = CDC (charge current 1-63 µA)
// CONFIG2 bits[6:4] = CDT (charge time: 4=4µs 5=8µs 6=16µs 7=32µs)
// CONFIG2 bits[3:2] = SFI (second filter iterations): 11 = 18 iters (max)
// CONFIG2 bits[1:0] = ESI (sample interval): 00 = 1ms

bool mpr_init_custom(uint8_t cdc, uint8_t cdt)
{
    mpr_write(REG_SOFTRESET, 0x63);
    System::Delay(10);
    mpr_write(REG_ECR, 0x00);

    // Slow baseline filter — glass reduces signal so we don't want
    // baseline tracking to eat the already-small touch deltas
    mpr_write(REG_MHDR, 0x01);
    mpr_write(REG_NHDR, 0x01);
    mpr_write(REG_NCLR, 0xFF);
    mpr_write(REG_FDLR, 0x02);
    mpr_write(REG_MHDF, 0x01);
    mpr_write(REG_NHDF, 0x01);
    mpr_write(REG_NCLF, 0xFF);
    mpr_write(REG_FDLF, 0x02);
    mpr_write(REG_NHDT, 0x00);
    mpr_write(REG_NCLT, 0x00);
    mpr_write(REG_FDLT, 0x00);

    // Low thresholds for glass
    for (uint8_t ch = 0; ch < 12; ch++)
    {
        mpr_write(0x41 + ch*2, 4);  // touch threshold
        mpr_write(0x42 + ch*2, 2);  // release threshold
    }

    mpr_write(REG_DEBOUNCE, 0x00);
    mpr_write(REG_CONFIG1, (uint8_t)(0xC0 | (cdc & 0x3F)));        // FFI=34 | CDC
    mpr_write(REG_CONFIG2, (uint8_t)(((cdt & 0x07) << 4) | 0x0C)); // CDT | SFI=18 | ESI=1ms
    mpr_write(REG_ECR, 0x8C);
    System::Delay(500);

    uint8_t ecr = 0;
    mpr_read(REG_ECR, &ecr, 1);
    return (ecr == 0x8C);
}

// ── Read electrodes ───────────────────────────────────────────────────────────
void read_electrodes(uint16_t* out)
{
    uint8_t buf[24];
    mpr_read(REG_ELE0_LSB, buf, 24);
    for (uint8_t i = 0; i < 12; i++)
        out[i] = (uint16_t)(buf[i*2] | ((buf[i*2+1] & 0x03) << 8));
}

// ── Measure peak delta across all electrodes over n samples ──────────────────
// Returns peak delta as integer (counts)
int32_t measure_peak(uint16_t* baseline, int n_samples)
{
    uint16_t vals[12];
    int32_t  peak = 0;
    for (int s = 0; s < n_samples; s++)
    {
        read_electrodes(vals);
        for (uint8_t i = 0; i < 12; i++)
        {
            int32_t delta = (int32_t)baseline[i] - (int32_t)vals[i];
            if (delta > peak) peak = delta;
        }
        System::Delay(16);
    }
    return peak;
}

// ── Result storage ────────────────────────────────────────────────────────────
struct Result {
    uint8_t cdc;
    uint8_t cdt;
    int32_t peak_off;   // noise peak (no touch)
    int32_t peak_on;    // signal peak (touch)
    int32_t snr_x10;    // SNR * 10 (integer, 1 decimal place)
};

static constexpr int MAX_RESULTS = 16;
Result results[MAX_RESULTS];
int    n_results = 0;

// ── CDT index to string ───────────────────────────────────────────────────────
const char* cdt_str(uint8_t cdt)
{
    switch(cdt) {
        case 4: return "4us ";
        case 5: return "8us ";
        case 6: return "16us";
        case 7: return "32us";
        default: return "??  ";
    }
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main()
{
    hw.Init();
    hw.StartLog(true);

    sda_high();
    scl_high();

    hw.PrintLine("MPR121 Glass Sensitivity Tuner");
    hw.PrintLine("==============================");
    hw.PrintLine("3mm glass  SDA=D12 SCL=D11");
    hw.PrintLine("");

    // CDC: charge current in µA
    static const uint8_t cdc_vals[] = { 16, 32, 48, 63 };
    // CDT: charge time index (4=4us, 5=8us, 6=16us, 7=32us)
    static const uint8_t cdt_vals[] = { 4, 5, 6, 7 };
    static const int n_cdc = 4, n_cdt = 4;
    int total = n_cdc * n_cdt, sweep = 0;

    for (int ci = 0; ci < n_cdc; ci++)
    {
        for (int ti = 0; ti < n_cdt; ti++)
        {
            uint8_t cdc = cdc_vals[ci];
            uint8_t cdt = cdt_vals[ti];
            sweep++;

            hw.PrintLine("--- %d/%d: CDC=%2duA  CDT=%s ---",
                         sweep, total, cdc, cdt_str(cdt));

            if (!mpr_init_custom(cdc, cdt))
            {
                hw.PrintLine("  INIT FAILED - skip");
                continue;
            }

            // ── Baseline snapshot (no touch) ──────────────────────────────────
            hw.PrintLine("  Finger OFF... (2s)");
            System::Delay(2000);

            uint32_t acc[12] = {0};
            uint16_t tmp[12];
            for (int s = 0; s < 16; s++)
            {
                read_electrodes(tmp);
                for (int i = 0; i < 12; i++) acc[i] += tmp[i];
                System::Delay(16);
            }
            uint16_t baseline[12];
            for (int i = 0; i < 12; i++) baseline[i] = (uint16_t)(acc[i] / 16);

            // Noise measurement
            int32_t peak_off = measure_peak(baseline, 20);
            hw.PrintLine("  Noise peak: %d counts", (int)peak_off);

            // ── Touch measurement ─────────────────────────────────────────────
            hw.PrintLine("  >>> TOUCH glass firmly (3s) <<<");
            System::Delay(500);

            int32_t peak_on = measure_peak(baseline, 60);
            hw.PrintLine("  Touch peak: %d counts", (int)peak_on);

            // SNR = peak_on / peak_off, stored as *10 for 1 decimal without floats
            int32_t snr_x10 = 0;
            if (peak_off > 0)
                snr_x10 = (peak_on * 10) / peak_off;

            hw.PrintLine("  SNR: %d.%d  %s",
                         (int)(snr_x10/10), (int)(snr_x10%10),
                         snr_x10 > 50 ? "GOOD" :
                         snr_x10 > 20 ? "marginal" : "poor");
            hw.PrintLine("");

            if (n_results < MAX_RESULTS)
                results[n_results++] = {cdc, cdt, peak_off, peak_on, snr_x10};

            if (sweep < total)
            {
                hw.PrintLine("  Lift finger. Next in 2s...");
                System::Delay(2000);
            }
        }
    }

    // ── Results table ─────────────────────────────────────────────────────────
    hw.PrintLine("==============================");
    hw.PrintLine("RESULTS");
    hw.PrintLine("==============================");
    hw.PrintLine(" #  | CDC  | CDT  | NO-TOUCH | TOUCH | SNR  | VERDICT");
    hw.PrintLine("----+------+------+----------+-------+------+--------");

    int best = 0;
    for (int i = 0; i < n_results; i++)
    {
        Result& r = results[i];
        hw.PrintLine(" %2d | %2duA | %s |  %6d  | %5d | %2d.%d | %s",
                     i+1, r.cdc, cdt_str(r.cdt),
                     (int)r.peak_off, (int)r.peak_on,
                     (int)(r.snr_x10/10), (int)(r.snr_x10%10),
                     r.snr_x10 > 50 ? "GOOD    " :
                     r.snr_x10 > 20 ? "marginal" : "poor    ");
        if (results[i].snr_x10 > results[best].snr_x10) best = i;
    }

    hw.PrintLine("");
    hw.PrintLine("BEST: CDC=%duA  CDT=%s  SNR=%d.%d",
                 results[best].cdc, cdt_str(results[best].cdt),
                 (int)(results[best].snr_x10/10),
                 (int)(results[best].snr_x10%10));
    hw.PrintLine("");
    hw.PrintLine("Paste into your synth:");
    hw.PrintLine("  CONFIG1 = 0x%02X   // FFI=34 | CDC=%duA",
                 0xC0 | (results[best].cdc & 0x3F), results[best].cdc);
    hw.PrintLine("  CONFIG2 = 0x%02X   // CDT=%s | SFI=18 | ESI=1ms",
                 ((results[best].cdt & 0x07) << 4) | 0x0C,
                 cdt_str(results[best].cdt));
    hw.PrintLine("  Touch threshold   = 4");
    hw.PrintLine("  Release threshold = 2");
    hw.PrintLine("");
    hw.PrintLine("Done. Reset to run again.");
    while (true) {}
}
