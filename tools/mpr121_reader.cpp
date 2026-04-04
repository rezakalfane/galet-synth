/**
 * mpr121_reader.cpp — MPR121 12-Electrode Reader (Glass-Optimised)
 * Daisy Seed — Bit-Bang I2C  SDA=D12  SCL=D11
 *
 * Uses a manual software baseline snapshot at boot instead of relying
 * on the MPR121's internal baseline tracker, which is too slow/unreliable
 * through glass. The software baseline is simply the average resting value
 * captured during the 2-second settle window after init.
 */

#include "daisy_seed.h"

using namespace daisy;
using namespace daisy::seed;

static constexpr Pin      SDA_PIN  = D12;
static constexpr Pin      SCL_PIN  = D11;
static constexpr uint8_t  MPR_ADDR = 0x5A;
static constexpr uint32_t HP       = 5;
static constexpr uint32_t PRINT_MS = 80;

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

bool mpr_init()
{
    mpr_write(REG_SOFTRESET, 0x63);
    System::Delay(10);
    mpr_write(REG_ECR, 0x00);

    // Baseline filter: moderate speed — not too slow (we use software baseline)
    // just slow enough that ambient drift doesn't corrupt our snapshot
    mpr_write(REG_MHDR, 0x01);
    mpr_write(REG_NHDR, 0x01);
    mpr_write(REG_NCLR, 0x10);
    mpr_write(REG_FDLR, 0x20);
    mpr_write(REG_MHDF, 0x01);
    mpr_write(REG_NHDF, 0x01);
    mpr_write(REG_NCLF, 0x10);
    mpr_write(REG_FDLF, 0x20);
    mpr_write(REG_NHDT, 0x00);
    mpr_write(REG_NCLT, 0x00);
    mpr_write(REG_FDLT, 0x00);

    // Low thresholds for glass
    for (uint8_t ch = 0; ch < 12; ch++)
    {
        mpr_write(0x41 + ch*2, 4);
        mpr_write(0x42 + ch*2, 2);
    }

    mpr_write(REG_DEBOUNCE, 0x11);
    mpr_write(REG_CONFIG1,  0x50);  // FFI=10 | CDC=16uA
    mpr_write(REG_CONFIG2,  0x6C);  // CDT=16us | SFI=18 | ESI=1ms
    mpr_write(REG_ECR,      0x8C);  // enable 12 electrodes

    System::Delay(100);
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

void make_bar(char* out, int32_t val, int32_t max_val)
{
    const int W = 24;
    int n = (max_val > 0 && val > 0)
            ? (int)((val * W + max_val/2) / max_val)
            : 0;
    if (n > W) n = W;
    out[0] = '[';
    for (int i = 0; i < W; i++) out[1+i] = (i < n) ? '#' : ' ';
    out[1+W] = ']';
    out[2+W] = '\0';
}

int main()
{
    hw.Init();
    hw.StartLog(true);
    sda_high();
    scl_high();

    hw.PrintLine("MPR121 Reader — Glass Optimised");
    hw.PrintLine("================================");

    if (!mpr_init())
    {
        hw.PrintLine("ERROR: MPR121 not found!");
        while (true) {}
    }

    // ── Software baseline capture ─────────────────────────────────────────────
    // Wait for electrodes to stabilise, then average 64 samples with no touch.
    // This gives a per-channel resting value regardless of hardware baseline state.
    hw.PrintLine("Keep finger OFF — capturing baseline...");
    System::Delay(500);

    uint32_t acc[12] = {0};
    uint16_t tmp[12];
    static const int BL_SAMPLES = 64;
    for (int s = 0; s < BL_SAMPLES; s++)
    {
        read_electrodes(tmp);
        for (int i = 0; i < 12; i++) acc[i] += tmp[i];
        System::Delay(5);
    }
    uint16_t sw_baseline[12];
    for (int i = 0; i < 12; i++)
        sw_baseline[i] = (uint16_t)(acc[i] / BL_SAMPLES);

    hw.PrintLine("Baseline: %u %u %u %u %u %u %u %u %u %u %u %u",
                 sw_baseline[0],  sw_baseline[1],  sw_baseline[2],
                 sw_baseline[3],  sw_baseline[4],  sw_baseline[5],
                 sw_baseline[6],  sw_baseline[7],  sw_baseline[8],
                 sw_baseline[9],  sw_baseline[10], sw_baseline[11]);
    hw.PrintLine("Ready.\n");

    uint16_t data[12];
    char     bar[28];

    while (true)
    {
        read_electrodes(data);

        int32_t max_delta = 1;
        for (uint8_t i = 0; i < 12; i++)
        {
            int32_t d = (int32_t)sw_baseline[i] - (int32_t)data[i];
            if (d > max_delta) max_delta = d;
        }

        hw.PrintLine(" CH | DATA | BASE | DELTA | BAR");
        hw.PrintLine("----+------+------+-------+--------------------------");
        for (uint8_t i = 0; i < 12; i++)
        {
            int32_t delta = (int32_t)sw_baseline[i] - (int32_t)data[i];
            if (delta < 0) delta = 0;
            make_bar(bar, delta, max_delta);
            hw.PrintLine(" %2d | %4u | %4u | %5d | %s",
                         i, data[i], sw_baseline[i], (int)delta, bar);
        }
        hw.PrintLine("");
        System::Delay(PRINT_MS);
    }
}
