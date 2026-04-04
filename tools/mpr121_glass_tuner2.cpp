/**
 * mpr121_glass_tuner2.cpp — MPR121 Fine Tuner (Phase 2)
 * Daisy Seed — Bit-Bang I2C  (SDA=D12, SCL=D11)
 *
 * Phase 1 found CDC=16uA CDT=16us as best.
 * This sweep fixes those and varies:
 *   FFI (first filter iterations): 6, 10, 18, 34
 *   ESI (electrode sample interval): 1ms, 2ms, 4ms, 8ms
 *
 * More FFI = more hardware averaging per sample = less noise
 * Slower ESI = more time per sample = more stable readings
 * Both should increase the effective touch delta through glass.
 */

#include "daisy_seed.h"

using namespace daisy;
using namespace daisy::seed;

static constexpr Pin     SDA_PIN  = D12;
static constexpr Pin     SCL_PIN  = D11;
static constexpr uint8_t MPR_ADDR = 0x5A;
static constexpr uint32_t HP      = 5;

static constexpr uint8_t REG_ELE0_LSB  = 0x04;
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

// Fixed from phase 1
static constexpr uint8_t BEST_CDC = 16;  // µA
static constexpr uint8_t BEST_CDT = 6;  // index 6 = 16µs

DaisySeed hw;
GPIO      sda, scl;

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

// FFI encoding: 00=6, 01=10, 10=18, 11=34 iterations
// SFI encoding: 00=4, 01=6,  10=10, 11=18 iterations
// ESI encoding: 000=1ms,001=2ms,010=4ms,011=8ms,100=16ms,...

bool mpr_init(uint8_t ffi, uint8_t esi)
{
    mpr_write(REG_SOFTRESET, 0x63);
    System::Delay(10);
    mpr_write(REG_ECR, 0x00);

    // Slow baseline — same as phase 1
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

    for (uint8_t ch = 0; ch < 12; ch++)
    {
        mpr_write(0x41 + ch*2, 4);
        mpr_write(0x42 + ch*2, 2);
    }

    mpr_write(REG_DEBOUNCE, 0x00);

    // CONFIG1: FFI | CDC=16
    uint8_t config1 = (uint8_t)((ffi & 0x03) << 6) | (BEST_CDC & 0x3F);
    mpr_write(REG_CONFIG1, config1);

    // CONFIG2: CDT=16us(6) | SFI=18(11) | ESI
    uint8_t config2 = (uint8_t)((BEST_CDT << 4) | 0x0C | (esi & 0x07));
    mpr_write(REG_CONFIG2, config2);

    mpr_write(REG_ECR, 0x8C);

    // Settle time scales with ESI — need ~60 samples minimum
    // 60 samples × ESI period
    uint32_t esi_ms = (uint32_t)(1 << esi); // 1,2,4,8 ms
    System::Delay(60 * esi_ms + 200);

    uint8_t ecr = 0;
    mpr_read(REG_ECR, &ecr, 1);
    return (ecr == 0x8C);
}

void read_electrodes(uint16_t* out)
{
    uint8_t buf[24];
    mpr_read(REG_ELE0_LSB, buf, 24);
    for (uint8_t i = 0; i < 12; i++)
        out[i] = (uint16_t)(buf[i*2] | ((buf[i*2+1] & 0x03) << 8));
}

int32_t measure_peak(uint16_t* baseline, int n_samples, uint32_t esi_ms)
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
        System::Delay(esi_ms + 1);
    }
    return peak;
}

struct Result {
    uint8_t ffi;
    uint8_t esi;
    int32_t peak_off;
    int32_t peak_on;
    int32_t snr_x10;
};

static constexpr int MAX_RESULTS = 16;
Result results[MAX_RESULTS];
int    n_results = 0;

const char* ffi_str(uint8_t ffi)
{
    switch(ffi) { case 0: return "FFI=6 "; case 1: return "FFI=10";
                  case 2: return "FFI=18"; default: return "FFI=34"; }
}

const char* esi_str(uint8_t esi)
{
    switch(esi) { case 0: return "ESI=1ms"; case 1: return "ESI=2ms";
                  case 2: return "ESI=4ms"; default: return "ESI=8ms"; }
}

int main()
{
    hw.Init();
    hw.StartLog(true);
    sda_high();
    scl_high();

    hw.PrintLine("MPR121 Glass Tuner — Phase 2");
    hw.PrintLine("============================");
    hw.PrintLine("Fixed: CDC=16uA  CDT=16us");
    hw.PrintLine("Sweep: FFI x ESI (4x4 = 16 configs)");
    hw.PrintLine("");

    // FFI: 0=6iters, 1=10iters, 2=18iters, 3=34iters
    static const uint8_t ffi_vals[] = { 0, 1, 2, 3 };
    // ESI: 0=1ms, 1=2ms, 2=4ms, 3=8ms
    static const uint8_t esi_vals[] = { 0, 1, 2, 3 };
    static const int n_ffi = 4, n_esi = 4;
    int total = n_ffi * n_esi, sweep = 0;

    for (int fi = 0; fi < n_ffi; fi++)
    {
        for (int ei = 0; ei < n_esi; ei++)
        {
            uint8_t ffi = ffi_vals[fi];
            uint8_t esi = esi_vals[ei];
            uint32_t esi_ms = (uint32_t)(1 << esi);
            sweep++;

            hw.PrintLine("--- %d/%d: %s  %s ---",
                         sweep, total, ffi_str(ffi), esi_str(esi));

            if (!mpr_init(ffi, esi))
            {
                hw.PrintLine("  INIT FAILED - skip");
                continue;
            }

            // Baseline snapshot
            hw.PrintLine("  Finger OFF... (2s)");
            System::Delay(2000);

            uint32_t acc[12] = {0};
            uint16_t tmp[12];
            for (int s = 0; s < 16; s++)
            {
                read_electrodes(tmp);
                for (int i = 0; i < 12; i++) acc[i] += tmp[i];
                System::Delay(esi_ms + 1);
            }
            uint16_t baseline[12];
            for (int i = 0; i < 12; i++) baseline[i] = (uint16_t)(acc[i] / 16);

            // Noise (finger off)
            int32_t peak_off = measure_peak(baseline, 30, esi_ms);
            // Clamp to 1 so we don't divide by zero
            if (peak_off < 1) peak_off = 1;
            hw.PrintLine("  Noise peak: %d counts", (int)peak_off);

            // Touch signal
            hw.PrintLine("  >>> TOUCH glass firmly (3s) <<<");
            System::Delay(500);
            int32_t peak_on = measure_peak(baseline, 60, esi_ms);
            hw.PrintLine("  Touch peak: %d counts", (int)peak_on);

            int32_t snr_x10 = (peak_on * 10) / peak_off;
            hw.PrintLine("  SNR: %d.%d  %s",
                         (int)(snr_x10/10), (int)(snr_x10%10),
                         snr_x10 > 50 ? "GOOD" :
                         snr_x10 > 20 ? "marginal" : "poor");
            hw.PrintLine("");

            if (n_results < MAX_RESULTS)
                results[n_results++] = {ffi, esi, peak_off, peak_on, snr_x10};

            if (sweep < total)
            {
                hw.PrintLine("  Lift finger. Next in 2s...");
                System::Delay(2000);
            }
        }
    }

    // Results table
    hw.PrintLine("============================");
    hw.PrintLine("PHASE 2 RESULTS");
    hw.PrintLine("============================");
    hw.PrintLine(" #  | FFI   | ESI    | NO-TOUCH | TOUCH | SNR  | VERDICT");
    hw.PrintLine("----+-------+--------+----------+-------+------+--------");

    int best = 0;
    for (int i = 0; i < n_results; i++)
    {
        Result& r = results[i];
        hw.PrintLine(" %2d | %s | %s |  %6d  | %5d | %2d.%d | %s",
                     i+1, ffi_str(r.ffi), esi_str(r.esi),
                     (int)r.peak_off, (int)r.peak_on,
                     (int)(r.snr_x10/10), (int)(r.snr_x10%10),
                     r.snr_x10 > 50 ? "GOOD    " :
                     r.snr_x10 > 20 ? "marginal" : "poor    ");
        if (results[i].snr_x10 > results[best].snr_x10) best = i;
    }

    Result& b = results[best];
    uint8_t config1 = (uint8_t)((b.ffi & 0x03) << 6) | (BEST_CDC & 0x3F);
    uint8_t config2 = (uint8_t)((BEST_CDT << 4) | 0x0C | (b.esi & 0x07));

    hw.PrintLine("");
    hw.PrintLine("BEST: %s  %s  SNR=%d.%d  Touch=%d",
                 ffi_str(b.ffi), esi_str(b.esi),
                 (int)(b.snr_x10/10), (int)(b.snr_x10%10),
                 (int)b.peak_on);
    hw.PrintLine("");
    hw.PrintLine("Final recommended registers:");
    hw.PrintLine("  CONFIG1 = 0x%02X   // %s | CDC=16uA", config1, ffi_str(b.ffi));
    hw.PrintLine("  CONFIG2 = 0x%02X   // CDT=16us | SFI=18 | %s", config2, esi_str(b.esi));
    hw.PrintLine("  Touch threshold   = 4");
    hw.PrintLine("  Release threshold = 2");
    hw.PrintLine("  NCLR/NCLF = 0xFF  // slow baseline filter");
    hw.PrintLine("");
    hw.PrintLine("Done. Reset to run again.");
    while (true) {}
}
