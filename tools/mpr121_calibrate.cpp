/**
 * mpr121_calibrate.cpp — Full sensor calibration for GaletSynth
 * Daisy Seed — Bit-Bang I2C  SDA=D12  SCL=D11
 *
 * Phase 1a  CDC × CDT sweep (9 configs)
 *           Find the best charge current / charge time for the sensor.
 * Phase 1b  FFI × ESI sweep (12 configs, best CDC/CDT locked)
 *           Fine-tune filter iterations and sample interval.
 * Phase 2   Two guided swipe rounds: LIGHT (→ PRESSURE_MIN_REF) and
 *           HARD (→ PRESSURE_MAX_REF), 8 s each. TOUCH_THRESHOLD = 10.
 *
 * At the end, prints the exact lines to paste into src/main.cpp.
 *
 * Build:
 *   make TARGET=tools/mpr121_calibrate libdaisy_dir=<path> \
 *        SYSTEM_FILES_DIR=<path>/core LIBDAISY_DIR=<path>
 * Flash:
 *   (same flags) program-dfu
 */

#include "daisy_seed.h"

using namespace daisy;
using namespace daisy::seed;

// ── Hardware config ───────────────────────────────────────────────────────────
static constexpr Pin      SDA_PIN  = D12;
static constexpr Pin      SCL_PIN  = D11;
static constexpr uint8_t  MPR_ADDR = 0x5A;
static constexpr uint32_t HP       = 5;   // I2C half-period µs
static constexpr int      N_CH     = 12;

// ── Calibration constants ─────────────────────────────────────────────────────
static constexpr int32_t CAL_TOUCH_THRESHOLD = 10;  // min delta to count as touch
static constexpr int32_t MAX_REF_MARGIN      = 3;   // safety headroom added to measured max
static constexpr int32_t DEFAULT_MAX_REF     = 35;  // fallback for electrodes never visited

// ── MPR121 registers ──────────────────────────────────────────────────────────
static constexpr uint8_t REG_ELE0_LSB  = 0x04;
static constexpr uint8_t REG_MHDR=0x2B, REG_NHDR=0x2C, REG_NCLR=0x2D, REG_FDLR=0x2E;
static constexpr uint8_t REG_MHDF=0x2F, REG_NHDF=0x30, REG_NCLF=0x31, REG_FDLF=0x32;
static constexpr uint8_t REG_NHDT=0x33, REG_NCLT=0x34, REG_FDLT=0x35;
static constexpr uint8_t REG_DEBOUNCE=0x5B, REG_CONFIG1=0x5C, REG_CONFIG2=0x5D;
static constexpr uint8_t REG_ECR=0x5E, REG_SOFTRESET=0x80;

// ── Globals ───────────────────────────────────────────────────────────────────
DaisySeed hw;
GPIO      sda, scl;

// ── Bit-bang I2C ──────────────────────────────────────────────────────────────
inline void sda_high(){sda.Init(SDA_PIN,GPIO::Mode::INPUT, GPIO::Pull::PULLUP);}
inline void sda_low() {sda.Init(SDA_PIN,GPIO::Mode::OUTPUT,GPIO::Pull::NOPULL);sda.Write(false);}
inline void scl_high(){scl.Init(SCL_PIN,GPIO::Mode::INPUT, GPIO::Pull::PULLUP);}
inline void scl_low() {scl.Init(SCL_PIN,GPIO::Mode::OUTPUT,GPIO::Pull::NOPULL);scl.Write(false);}
inline void d(){System::DelayUs(HP);}

void i2c_start(){sda_high();d();scl_high();d();sda_low();d();scl_low();d();}
void i2c_stop() {sda_low();d();scl_high();d();sda_high();d();}

bool i2c_write_byte(uint8_t byte)
{
    for(int i=7;i>=0;i--){if(byte&(1<<i))sda_high();else sda_low();d();scl_high();d();scl_low();d();}
    sda_high();d();scl_high();d();bool ack=!sda.Read();scl_low();d();return ack;
}

uint8_t i2c_read_byte(bool ack)
{
    uint8_t b=0;sda_high();
    for(int i=7;i>=0;i--){d();scl_high();d();if(sda.Read())b|=(1<<i);scl_low();}
    if(ack)sda_low();else sda_high();d();scl_high();d();scl_low();d();sda_high();return b;
}

bool mpr_write(uint8_t reg,uint8_t val)
{
    i2c_start();
    if(!i2c_write_byte((MPR_ADDR<<1)|0)){i2c_stop();return false;}
    if(!i2c_write_byte(reg))            {i2c_stop();return false;}
    if(!i2c_write_byte(val))            {i2c_stop();return false;}
    i2c_stop();return true;
}

bool mpr_read(uint8_t reg,uint8_t* buf,uint8_t len)
{
    i2c_start();
    if(!i2c_write_byte((MPR_ADDR<<1)|0)){i2c_stop();return false;}
    if(!i2c_write_byte(reg))            {i2c_stop();return false;}
    i2c_start();
    if(!i2c_write_byte((MPR_ADDR<<1)|1)){i2c_stop();return false;}
    for(uint8_t i=0;i<len;i++)buf[i]=i2c_read_byte(i<len-1);
    i2c_stop();return true;
}

// ── MPR121 init with explicit AFE parameters ──────────────────────────────────
// ffi: 0=6iters 1=10iters 2=18iters 3=34iters
// esi: 0=1ms    1=2ms     2=4ms     3=8ms
bool mpr_init_afe(uint8_t cdc,uint8_t cdt,uint8_t ffi,uint8_t esi)
{
    mpr_write(REG_SOFTRESET,0x63);System::Delay(10);
    mpr_write(REG_ECR,0x00);
    // Slow baseline filter — prevents baseline tracking from eating touch signal
    mpr_write(REG_MHDR,0x01);mpr_write(REG_NHDR,0x01);
    mpr_write(REG_NCLR,0xFF);mpr_write(REG_FDLR,0x02);
    mpr_write(REG_MHDF,0x01);mpr_write(REG_NHDF,0x01);
    mpr_write(REG_NCLF,0xFF);mpr_write(REG_FDLF,0x02);
    mpr_write(REG_NHDT,0x00);mpr_write(REG_NCLT,0x00);mpr_write(REG_FDLT,0x00);
    for(uint8_t ch=0;ch<12;ch++){mpr_write(0x41+ch*2,4);mpr_write(0x42+ch*2,2);}
    mpr_write(REG_DEBOUNCE,0x00);
    mpr_write(REG_CONFIG1,(uint8_t)(((ffi&0x03)<<6)|(cdc&0x3F)));
    mpr_write(REG_CONFIG2,(uint8_t)(((cdt&0x07)<<4)|0x0C|(esi&0x07)));
    mpr_write(REG_ECR,0x8C);
    uint32_t esi_ms=(uint32_t)(1u<<esi);
    System::Delay(60*esi_ms+200);
    uint8_t ecr=0;mpr_read(REG_ECR,&ecr,1);
    return(ecr==0x8C);
}

// ── Read raw electrode data ───────────────────────────────────────────────────
void read_electrodes(uint16_t* out)
{
    uint8_t buf[24];
    mpr_read(REG_ELE0_LSB,buf,24);
    for(int i=0;i<N_CH;i++)
        out[i]=(uint16_t)(buf[i*2]|((buf[i*2+1]&0x03)<<8));
}

// ── Capture software baseline (32 samples) ────────────────────────────────────
void capture_baseline(uint16_t* bl,uint32_t esi_ms)
{
    uint32_t acc[N_CH]={0};uint16_t tmp[N_CH];
    for(int s=0;s<32;s++){
        read_electrodes(tmp);
        for(int i=0;i<N_CH;i++)acc[i]+=tmp[i];
        System::Delay(esi_ms+1);
    }
    for(int i=0;i<N_CH;i++)bl[i]=(uint16_t)(acc[i]/32);
}

// ── Measure global peak delta over n samples ──────────────────────────────────
int32_t measure_peak_global(uint16_t* bl,int n,uint32_t esi_ms)
{
    uint16_t v[N_CH];int32_t pk=0;
    for(int s=0;s<n;s++){
        read_electrodes(v);
        for(int i=0;i<N_CH;i++){
            int32_t d=(int32_t)bl[i]-(int32_t)v[i];
            if(d>pk)pk=d;
        }
        System::Delay(esi_ms+1);
    }
    return pk;
}

// ── AFE result storage ────────────────────────────────────────────────────────
struct AfeResult { uint8_t cdc,cdt,ffi,esi; int32_t snr_x10,peak_on; };
static constexpr int MAX_AFE = 24;
static AfeResult afe_results[MAX_AFE];
static int       n_afe = 0;

// ── String helpers ────────────────────────────────────────────────────────────
static const char* cdt_us(uint8_t cdt)
{
    switch(cdt){case 4:return "4us ";case 5:return "8us ";case 6:return "16us";default:return "32us";}
}
static const char* ffi_str(uint8_t ffi)
{
    switch(ffi){case 0:return " 6";case 1:return "10";case 2:return "18";default:return "34";}
}

// ── Block until any electrode exceeds CAL_TOUCH_THRESHOLD ────────────────────
void wait_for_touch(uint16_t* bl)
{
    hw.PrintLine("  Waiting for touch...");
    uint16_t v[N_CH];
    bool hit = false;
    while(!hit){
        read_electrodes(v);
        for(int i=0;i<N_CH;i++)
            if((int32_t)bl[i]-(int32_t)v[i] >= CAL_TOUCH_THRESHOLD){hit=true;break;}
        if(!hit) System::Delay(10);
    }
    hw.PrintLine("  Touch detected — go!");
}

// ── Run one AFE config, measure noise + touch SNR ─────────────────────────────
void sweep_one(int idx,int total,uint8_t cdc,uint8_t cdt,uint8_t ffi,uint8_t esi)
{
    uint32_t esi_ms=(uint32_t)(1u<<esi);
    hw.PrintLine("--- %d/%d  CDC=%2duA CDT=%s FFI=%siters ESI=%dms ---",
                 idx,total,(int)cdc,cdt_us(cdt),ffi_str(ffi),(int)esi_ms);

    if(!mpr_init_afe(cdc,cdt,ffi,esi)){hw.PrintLine("  INIT FAILED — skip");return;}

    // Baseline
    hw.PrintLine("  Keep fingers OFF... (2s)");
    System::Delay(2000);
    uint16_t bl[N_CH];
    capture_baseline(bl,esi_ms);
    int32_t noise=measure_peak_global(bl,20,esi_ms);
    if(noise<1)noise=1;
    hw.PrintLine("  Noise: %d counts",(int)noise);

    // Touch — wait for actual contact before measuring
    hw.PrintLine("  >>> Press glass FIRMLY when ready <<<");
    wait_for_touch(bl);
    int32_t sig=measure_peak_global(bl,60,esi_ms);
    int32_t snr=(sig*10)/noise;
    hw.PrintLine("  Touch: %d  SNR: %d.%d  %s",
                 (int)sig,(int)(snr/10),(int)(snr%10),
                 snr>50?"GOOD":snr>20?"ok":"poor");

    if(n_afe<MAX_AFE)
        afe_results[n_afe++]={cdc,cdt,ffi,esi,snr,sig};

    if(idx<total){hw.PrintLine("  Lift finger. Next in 2s...");System::Delay(2000);}
    hw.PrintLine("");
}

// ── Find index of best SNR result in afe_results[] ───────────────────────────
int find_best()
{
    int b=0;
    for(int i=1;i<n_afe;i++)
        if(afe_results[i].snr_x10>afe_results[b].snr_x10)b=i;
    return b;
}

// ── Phase 1a: sweep CDC × CDT (FFI=10iters, ESI=1ms fixed) ───────────────────
void sweep_cdc_cdt()
{
    static const uint8_t cdc_v[]={8,16,32};       // µA
    static const uint8_t cdt_v[]={5,6,7};         // 8µs, 16µs, 32µs
    int total=9,sw=0;

    hw.PrintLine("=== Phase 1a: CDC x CDT (9 configs) ===");
    hw.PrintLine("FFI=10iters ESI=1ms fixed.");
    hw.PrintLine("For each config: keep finger off, then press firmly.");
    hw.PrintLine("");

    for(int ci=0;ci<3;ci++)
        for(int ti=0;ti<3;ti++)
            sweep_one(++sw,total,cdc_v[ci],cdt_v[ti],1,0);
}

// ── Phase 1b: sweep FFI × ESI (best CDC/CDT locked) ──────────────────────────
void sweep_ffi_esi(uint8_t cdc,uint8_t cdt)
{
    static const uint8_t ffi_v[]={0,1,2,3};       // 6,10,18,34 iters
    static const uint8_t esi_v[]={0,1,2};         // 1ms,2ms,4ms (8ms too slow for play)
    int total=12,sw=0;

    hw.PrintLine("=== Phase 1b: FFI x ESI (12 configs) ===");
    hw.PrintLine("CDC=%duA CDT=%s locked from phase 1a best.",(int)cdc,cdt_us(cdt));
    hw.PrintLine("");

    for(int fi=0;fi<4;fi++)
        for(int ei=0;ei<3;ei++)
            sweep_one(++sw,total,cdc,cdt,ffi_v[fi],esi_v[ei]);
}

// ── Shared: one 8-second swipe round, records max delta per electrode ─────────
// Waits for first touch before starting the timer.
void swipe_round(uint16_t* bl,uint32_t esi_ms,int32_t* out)
{
    wait_for_touch(bl);
    hw.PrintLine("  Recording 8s — swipe across all electrodes now!");

    uint16_t v[N_CH];
    uint32_t t_start=System::GetNow();
    int frame=0;
    while(System::GetNow()-t_start < 8000u)
    {
        read_electrodes(v);
        for(int i=0;i<N_CH;i++){
            int32_t delta=(int32_t)bl[i]-(int32_t)v[i];
            if(delta<0)delta=0;
            if(delta>out[i])out[i]=delta;
        }
        if((frame%15)==0)
        {
            uint32_t rem=(8000u-(System::GetNow()-t_start))/1000u;
            int32_t d[N_CH];
            for(int i=0;i<N_CH;i++){d[i]=(int32_t)bl[i]-(int32_t)v[i];if(d[i]<0)d[i]=0;}
            hw.PrintLine("[%ds] %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d",
                (int)rem,
                (int)d[0],(int)d[1],(int)d[2],(int)d[3],(int)d[4],(int)d[5],
                (int)d[6],(int)d[7],(int)d[8],(int)d[9],(int)d[10],(int)d[11]);
        }
        frame++;
        System::Delay(esi_ms+18);
    }
}

// ── Fill any electrode that was never touched via neighbour interpolation ─────
void fill_unvisited(int32_t* ref,int32_t threshold,int32_t fallback)
{
    for(int i=0;i<N_CH;i++)
    {
        if(ref[i]>=threshold) continue;
        int32_t nb=0; int nn=0;
        if(i>0      && ref[i-1]>=threshold){nb+=ref[i-1];nn++;}
        if(i<N_CH-1 && ref[i+1]>=threshold){nb+=ref[i+1];nn++;}
        ref[i]=(nn>0)?(nb/nn):fallback;
        hw.PrintLine("  ch%02d not visited — filled with %d",i,(int)ref[i]);
    }
}

// ── Phase 2: per-electrode pressure calibration (LIGHT + HARD) ───────────────
void calibrate_pressure(uint16_t* bl,uint32_t esi_ms,int32_t* min_ref,int32_t* max_ref)
{
    for(int i=0;i<N_CH;i++){min_ref[i]=0;max_ref[i]=0;}

    // ── Round 1: LIGHT → min_ref ─────────────────────────────────────────────
    hw.PrintLine("--- Round 1/2: LIGHT touch ---");
    hw.PrintLine("  Barely graze the glass — lightest contact you'd ever play.");
    hw.PrintLine("  Touch when ready, then swipe slowly across ALL electrodes.");
    swipe_round(bl,esi_ms,min_ref);
    hw.PrintLine("  Done. Lift finger.");
    hw.PrintLine("  ch: 00  01  02  03  04  05  06  07  08  09  10  11");
    hw.PrintLine("  min %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d",
        (int)min_ref[0],(int)min_ref[1],(int)min_ref[2],(int)min_ref[3],
        (int)min_ref[4],(int)min_ref[5],(int)min_ref[6],(int)min_ref[7],
        (int)min_ref[8],(int)min_ref[9],(int)min_ref[10],(int)min_ref[11]);
    System::Delay(1500);
    hw.PrintLine("");

    // ── Round 2: HARD → max_ref ──────────────────────────────────────────────
    hw.PrintLine("--- Round 2/2: HARD press ---");
    hw.PrintLine("  Maximum pressure — hardest you would ever push while playing.");
    hw.PrintLine("  Touch when ready, then swipe slowly across ALL electrodes.");
    swipe_round(bl,esi_ms,max_ref);
    hw.PrintLine("  Done. Lift finger.");
    hw.PrintLine("  ch: 00  01  02  03  04  05  06  07  08  09  10  11");
    hw.PrintLine("  max %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d",
        (int)max_ref[0],(int)max_ref[1],(int)max_ref[2],(int)max_ref[3],
        (int)max_ref[4],(int)max_ref[5],(int)max_ref[6],(int)max_ref[7],
        (int)max_ref[8],(int)max_ref[9],(int)max_ref[10],(int)max_ref[11]);
    System::Delay(1500);
    hw.PrintLine("");

    // ── Post-process ──────────────────────────────────────────────────────────
    // Compute fallback averages from visited electrodes
    int32_t min_sum=0,max_sum=0; int nv=0;
    for(int i=0;i<N_CH;i++)
        if(min_ref[i]>=CAL_TOUCH_THRESHOLD){min_sum+=min_ref[i];max_sum+=max_ref[i];nv++;}
    int32_t min_fb=(nv>0)?(min_sum/nv):(CAL_TOUCH_THRESHOLD+2);
    int32_t max_fb=(nv>0)?(max_sum/nv):DEFAULT_MAX_REF;

    fill_unvisited(min_ref,CAL_TOUCH_THRESHOLD,min_fb);
    fill_unvisited(max_ref,CAL_TOUCH_THRESHOLD,max_fb);

    // Ensure min < max with at least a minimum useful range per electrode
    for(int i=0;i<N_CH;i++)
    {
        if(min_ref[i]<CAL_TOUCH_THRESHOLD) min_ref[i]=CAL_TOUCH_THRESHOLD;
        if(max_ref[i]<=min_ref[i])         max_ref[i]=min_ref[i]+15;
    }
}

// ── Print the final patch block to paste into main.cpp ───────────────────────
void print_patch(AfeResult& b,int32_t* min_ref,int32_t* max_ref)
{
    uint8_t cfg1=(uint8_t)(((b.ffi&0x03)<<6)|(b.cdc&0x3F));
    uint8_t cfg2=(uint8_t)(((b.cdt&0x07)<<4)|0x0C|(b.esi&0x07));
    uint32_t esi_ms=(uint32_t)(1u<<b.esi);

    hw.PrintLine("========================================");
    hw.PrintLine("CALIBRATION DONE — paste into src/main.cpp");
    hw.PrintLine("========================================");
    hw.PrintLine("");
    hw.PrintLine("// ── Touch tuning (replace lines ~38-50) ────────────────");
    hw.PrintLine("static constexpr int32_t  TOUCH_THRESHOLD    = %d;",(int)CAL_TOUCH_THRESHOLD);
    hw.PrintLine("static constexpr int32_t  PRESSURE_MIN_REF[12] = {");
    hw.PrintLine("    %d,%d,%d,%d,%d,%d,",
        (int)min_ref[0],(int)min_ref[1],(int)min_ref[2],
        (int)min_ref[3],(int)min_ref[4],(int)min_ref[5]);
    hw.PrintLine("    %d,%d,%d,%d,%d,%d",
        (int)min_ref[6],(int)min_ref[7],(int)min_ref[8],
        (int)min_ref[9],(int)min_ref[10],(int)min_ref[11]);
    hw.PrintLine("};");
    hw.PrintLine("static constexpr int32_t  PRESSURE_MAX_REF[12] = {");
    hw.PrintLine("    %d,%d,%d,%d,%d,%d,",
        (int)max_ref[0],(int)max_ref[1],(int)max_ref[2],
        (int)max_ref[3],(int)max_ref[4],(int)max_ref[5]);
    hw.PrintLine("    %d,%d,%d,%d,%d,%d",
        (int)max_ref[6],(int)max_ref[7],(int)max_ref[8],
        (int)max_ref[9],(int)max_ref[10],(int)max_ref[11]);
    hw.PrintLine("};");
    hw.PrintLine("");
    hw.PrintLine("// ── mpr_init() CONFIG lines ──────────────────────────────");
    hw.PrintLine("mpr_write(REG_CONFIG1, 0x%02X);  // FFI=%siters CDC=%duA",
                 cfg1,ffi_str(b.ffi),(int)b.cdc);
    hw.PrintLine("mpr_write(REG_CONFIG2, 0x%02X);  // CDT=%s SFI=18 ESI=%dms",
                 cfg2,cdt_us(b.cdt),(int)esi_ms);
    hw.PrintLine("========================================");
    hw.PrintLine("SNR: %d.%d   Done. Reset to run again.",
                 (int)(b.snr_x10/10),(int)(b.snr_x10%10));
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main()
{
    hw.Init();
    hw.StartLog(true);
    sda_high();scl_high();

    hw.PrintLine("GaletSynth — Full Sensor Calibration");
    hw.PrintLine("=====================================");
    hw.PrintLine("Phase 1a  CDC x CDT   (9 configs, ~2 min)");
    hw.PrintLine("Phase 1b  FFI x ESI  (12 configs, ~3 min)");
    hw.PrintLine("Phase 2   Pressure per electrode (3 x 8s swipes)");
    hw.PrintLine("TOUCH_THRESHOLD fixed at %d",(int)CAL_TOUCH_THRESHOLD);
    hw.PrintLine("");

    // ── Phase 1a: CDC x CDT ───────────────────────────────────────────────────
    sweep_cdc_cdt();

    int b1=find_best();
    uint8_t best_cdc=afe_results[b1].cdc;
    uint8_t best_cdt=afe_results[b1].cdt;
    hw.PrintLine("Phase 1a best: CDC=%duA CDT=%s SNR=%d.%d",
                 (int)best_cdc,cdt_us(best_cdt),
                 (int)(afe_results[b1].snr_x10/10),(int)(afe_results[b1].snr_x10%10));
    hw.PrintLine("");

    // ── Phase 1b: FFI x ESI ───────────────────────────────────────────────────
    sweep_ffi_esi(best_cdc,best_cdt);

    int b2=find_best();
    AfeResult& best=afe_results[b2];
    uint32_t best_esi_ms=(uint32_t)(1u<<best.esi);

    hw.PrintLine("Overall best: CDC=%duA CDT=%s FFI=%siters ESI=%dms SNR=%d.%d",
                 (int)best.cdc,cdt_us(best.cdt),ffi_str(best.ffi),(int)best_esi_ms,
                 (int)(best.snr_x10/10),(int)(best.snr_x10%10));
    hw.PrintLine("");

    // ── Phase 2: per-electrode pressure calibration ───────────────────────────
    hw.PrintLine("=====================================");
    hw.PrintLine("Phase 2 — Per-electrode pressure calibration");
    hw.PrintLine("Using best AFE config. Capturing baseline...");

    if(!mpr_init_afe(best.cdc,best.cdt,best.ffi,best.esi))
    {
        hw.PrintLine("MPR121 init failed — halting.");
        while(true){}
    }
    System::Delay(300);

    hw.PrintLine("Keep fingers OFF (2s)...");
    System::Delay(2000);
    uint16_t baseline[N_CH];
    capture_baseline(baseline,best_esi_ms);
    hw.PrintLine("Baseline ready.");
    hw.PrintLine("");
    hw.PrintLine("  ch: 00  01  02  03  04  05  06  07  08  09  10  11");
    hw.PrintLine("  (columns match electrode positions left to right)");
    hw.PrintLine("");

    int32_t min_ref[N_CH], max_ref[N_CH];
    calibrate_pressure(baseline,best_esi_ms,min_ref,max_ref);

    print_patch(best,min_ref,max_ref);
    while(true){}
}
