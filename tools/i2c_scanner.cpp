/**
 * i2c_scanner.cpp — Bit-Bang I2C Address Scanner for Daisy Seed
 *
 * Scans the full 7-bit I2C address space (0x00–0x7F) using software
 * bit-banging and reports found devices over USB serial (daisy::UsbHandle).
 *
 * MPR121 I2C addresses (set by ADDR pin):
 *   ADDR → GND  : 0x5A  (default)
 *   ADDR → VCC  : 0x5B
 *   ADDR → SDA  : 0x5C
 *   ADDR → SCL  : 0x5D
 *
 * Wiring (change SDA_PIN / SCL_PIN to your actual Daisy pins):
 *   Daisy D12 (PB8)  → MPR121 SDA  (+ 4.7kΩ pull-up to 3.3V)
 *   Daisy D11 (PB9)  → MPR121 SCL  (+ 4.7kΩ pull-up to 3.3V)
 *   Daisy 3V3        → MPR121 VCC / ADDR (for address 0x5A tie ADDR to GND)
 *   Daisy GND        → MPR121 GND
 *
 * Build with libDaisy. Flash via DFU or ST-Link.
 */

#include "daisy_seed.h"

using namespace daisy;

// ── Pin configuration ─────────────────────────────────────────────────────────
// Change these to whichever Daisy Seed pins you have wired.
// Any GPIO-capable pin works; both need external pull-ups (4.7 kΩ to 3.3 V).
static constexpr Pin SDA_PIN = seed::D12;   // e.g. PB8
static constexpr Pin SCL_PIN = seed::D11;   // e.g. PB9

// Bit-bang timing — 100 kHz standard mode (~5 µs half-period).
// Increase HALF_PERIOD_US for slower / more reliable scans on long wires.
static constexpr uint32_t HALF_PERIOD_US = 5;

// ── Global objects ────────────────────────────────────────────────────────────
DaisySeed hw;
GPIO      sda, scl;

// ── Low-level helpers ─────────────────────────────────────────────────────────

inline void delay_us(uint32_t us)
{
    System::DelayUs(us);
}

// daisy::GPIO has no GetPin(); we re-init using the original Pin constants.
// Release a line HIGH (open-drain: switch to input + pull-up)
inline void line_high(GPIO &pin, Pin p)
{
    pin.Init(p, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
}
// Pull a line LOW (switch to output-low)
inline void line_low(GPIO &pin, Pin p)
{
    pin.Init(p, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
    pin.Write(false);
}

// Convenience wrappers that know which pin they operate on
inline void sda_high() { line_high(sda, SDA_PIN); }
inline void sda_low()  { line_low (sda, SDA_PIN); }
inline void scl_high() { line_high(scl, SCL_PIN); }
inline void scl_low()  { line_low (scl, SCL_PIN); }

// ── Bit-bang I2C primitives ───────────────────────────────────────────────────

void i2c_start()
{
    sda_high();
    scl_high();
    delay_us(HALF_PERIOD_US);
    // SDA falls while SCL is high → START condition
    sda_low();
    delay_us(HALF_PERIOD_US);
    scl_low();
    delay_us(HALF_PERIOD_US);
}

void i2c_stop()
{
    sda_low();
    delay_us(HALF_PERIOD_US);
    scl_high();
    delay_us(HALF_PERIOD_US);
    // SDA rises while SCL is high → STOP condition
    sda_high();
    delay_us(HALF_PERIOD_US);
}

/**
 * Clock out one byte MSB-first.
 * Returns true if the device ACKed (SDA pulled low during ACK clock).
 */
bool i2c_write_byte(uint8_t byte)
{
    for (int bit = 7; bit >= 0; bit--)
    {
        if (byte & (1u << bit))
            sda_high();
        else
            sda_low();

        delay_us(HALF_PERIOD_US);
        scl_high();
        delay_us(HALF_PERIOD_US);
        scl_low();
        delay_us(HALF_PERIOD_US);
    }

    // Release SDA for ACK — device pulls it LOW to acknowledge
    sda_high();
    delay_us(HALF_PERIOD_US);
    scl_high();
    delay_us(HALF_PERIOD_US);

    bool acked = !sda.Read();   // ACK = SDA pulled LOW by device

    scl_low();
    delay_us(HALF_PERIOD_US);
    return acked;
}

/**
 * Probe a 7-bit address.
 * Sends START + (addr<<1 | 0) and checks for ACK, then sends STOP.
 * Returns true if a device responded.
 */
bool i2c_probe(uint8_t addr)
{
    i2c_start();
    bool acked = i2c_write_byte(static_cast<uint8_t>((addr << 1) | 0x00));
    i2c_stop();
    return acked;
}

// ── USB serial helper ─────────────────────────────────────────────────────────

void usb_print(const char *msg)
{
    hw.PrintLine(msg);
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main()
{
    hw.Init();
    hw.StartLog(true);  // wait for USB serial connection before proceeding

    // Initial pin state: both lines released high (open-drain idle)
    sda_high();
    scl_high();

    usb_print("===================================");
    usb_print(" Daisy Seed — Bit-Bang I2C Scanner");
    usb_print("===================================");
    usb_print("Scanning addresses 0x00 – 0x7F ...");
    usb_print("");

    int found = 0;
    char buf[64];

    for (uint8_t addr = 0x00; addr <= 0x7F; addr++)
    {
        // Skip reserved addresses: 0x00–0x07 and 0x78–0x7F
        if (addr < 0x08 || addr > 0x77)
            continue;

        if (i2c_probe(addr))
        {
            found++;

            // Check if this is an MPR121 address
            const char *note = "";
            switch (addr)
            {
                case 0x5A: note = "  ← MPR121 (ADDR=GND, default)"; break;
                case 0x5B: note = "  ← MPR121 (ADDR=VCC)";          break;
                case 0x5C: note = "  ← MPR121 (ADDR=SDA)";          break;
                case 0x5D: note = "  ← MPR121 (ADDR=SCL)";          break;
                default:   break;
            }

            sprintf(buf, "  FOUND: 0x%02X%s", addr, note);
            usb_print(buf);
        }

        // Small inter-probe delay for bus recovery
        System::DelayUs(200);
    }

    usb_print("");
    usb_print("-----------------------------------");
    sprintf(buf, "Scan complete. %d device(s) found.", found);
    usb_print(buf);

    if (found == 0)
    {
        usb_print("");
        usb_print("No devices found. Check:");
        usb_print("  1. Pull-up resistors on SDA & SCL (4.7k to 3.3V)");
        usb_print("  2. Correct SDA_PIN / SCL_PIN in code");
        usb_print("  3. Device power (VCC/GND connected)");
        usb_print("  4. MPR121 ADDR pin connection");
    }

    usb_print("===================================");

    // Loop forever — re-trigger a scan by pressing reset
    while (true)
    {
        System::DelayUs(1000000); // ~1 s
    }
}
