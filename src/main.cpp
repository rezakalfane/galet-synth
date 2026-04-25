/**
 * mpr121_synth.cpp — Glass Touch Moog Synth
 * Daisy Seed — Bit-Bang I2C  SDA=D12  SCL=D11
 *
 * FINGER 1:
 *   Position  → Pitch (quantized on touch-down, continuous slide after)
 *   Pressure  → Filter cutoff (light=dark/closed, hard=bright/open)
 *               + Vibrato depth (press harder for more wobble)
 *
 * FINGER 2:
 *   Position  → Oscillator detune (spread between osc1 and osc2)
 *   Pressure  → Wavefold distortion (gentle saturation to complex fold)
 *               + Ring mod amount (extreme pressure adds metallic edge)
 *
 * EXPRESSION:
 *   - Note quantize on initial touch, then free pitch bend on slide
 *   - Pressure-controlled vibrato (LFO depth from finger 1 pressure)
 *   - Portamento / glide between quantized notes
 *   - Sub oscillator one octave below (adds weight)
 *   - Soft-clip waveshaper before output
 *   - Amplitude envelope: fast attack, pressure-controlled release
 */

#include "daisy_seed.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <string.h>

using namespace daisy;
using namespace daisy::seed;

// ── Pin / I2C config ──────────────────────────────────────────────────────────
static constexpr Pin      SDA_PIN  = D12;
static constexpr Pin      SCL_PIN  = D11;
static constexpr uint8_t  MPR_ADDR = 0x5A;
static constexpr uint32_t HP       = 2;   // I2C half-period µs (~250kHz)
static constexpr int      N_CH     = 12;

// ── Touch tuning ──────────────────────────────────────────────────────────────
static constexpr int32_t  TOUCH_THRESHOLD    = 7;
static constexpr int32_t  PRESSURE_MAX_REF   = 24;
static constexpr int32_t  MIN_FINGER_SEP     = 4;
static constexpr uint32_t REBASELINE_IDLE_MS = 2000;
static constexpr int      REBASELINE_SAMPLES = 32;
static constexpr int32_t  MAX_POS_JUMP       = 200;

// ── Musical mapping ───────────────────────────────────────────────────────────
// Finger 1 position 0–1000 maps across MIDI notes LOW_NOTE to HIGH_NOTE
// Frequency range: 50Hz (electrode 0) to 300Hz (electrode 11), exponential
static constexpr float FREQ_LOW  = 50.0f;
static constexpr float FREQ_HIGH = 300.0f;
// Set to false to disable all quantization (fully continuous pitch)
static constexpr bool QUANTIZE_ENABLED = false;
// Quantize to chromatic scale (all 12 semitones)
// Change to {0,2,4,5,7,9,11} for major scale, {0,2,3,5,7,8,10} for minor, etc.
static const int SCALE[] = {0,1,2,3,4,5,6,7,8,9,10,11}; // chromatic
static constexpr int SCALE_LEN = 12;

// ── MPR121 registers ──────────────────────────────────────────────────────────
static constexpr uint8_t REG_ELE0_LSB  = 0x04;
static constexpr uint8_t REG_MHDR=0x2B,REG_NHDR=0x2C,REG_NCLR=0x2D,REG_FDLR=0x2E;
static constexpr uint8_t REG_MHDF=0x2F,REG_NHDF=0x30,REG_NCLF=0x31,REG_FDLF=0x32;
static constexpr uint8_t REG_NHDT=0x33,REG_NCLT=0x34,REG_FDLT=0x35;
static constexpr uint8_t REG_DEBOUNCE=0x5B,REG_CONFIG1=0x5C,REG_CONFIG2=0x5D;
static constexpr uint8_t REG_ECR=0x5E,REG_SOFTRESET=0x80;

// ── Globals ───────────────────────────────────────────────────────────────────
DaisySeed hw;
GPIO      sda, scl;

// ── MPR121 IRQ — D10 = PB5, active low ───────────────────────────────────────
static volatile bool g_mpr_irq = false;

static void mpr_irq_init()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {};
    g.Pin  = GPIO_PIN_5;
    g.Mode = GPIO_MODE_IT_FALLING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

extern "C" void EXTI9_5_IRQHandler(void)
{
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5))
    {
        g_mpr_irq = true;
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    }
}


// ── Bit-bang I2C ──────────────────────────────────────────────────────────────
// SDA: OUTPUT_OD + PULLUP — Write(1) releases (high-Z, pulled up), Write(0) drives low.
//      Read() works in OD mode (STM32 IDR always reflects actual pin state).
// SCL: OUTPUT push-pull — master always drives it, never reads it.
// Both pins inited once in main(); no mode switching in the hot path.
inline void sda_high(){sda.Write(true);}
inline void sda_low() {sda.Write(false);}
inline void scl_high(){scl.Write(true);}
inline void scl_low() {scl.Write(false);}
inline void d(){System::DelayUs(HP);}

void i2c_start(){sda_high();d();scl_high();d();sda_low();d();scl_low();d();}
void i2c_stop() {sda_low();d();scl_high();d();sda_high();d();}

bool i2c_write_byte(uint8_t byte)
{
    for(int i=7;i>=0;i--){
        if(byte&(1<<i))sda_high();else sda_low();
        d();scl_high();d();scl_low();d();
    }
    sda_high();d();scl_high();d();
    bool ack=!sda.Read();
    scl_low();d();
    return ack;
}

uint8_t i2c_read_byte(bool ack)
{
    uint8_t b=0;sda_high();
    for(int i=7;i>=0;i--){d();scl_high();d();if(sda.Read())b|=(1<<i);scl_low();}
    if(ack)sda_low();else sda_high();
    d();scl_high();d();scl_low();d();sda_high();
    return b;
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
    for(uint8_t i=0;i<len;i++) buf[i]=i2c_read_byte(i<len-1);
    i2c_stop();return true;
}

bool mpr_init()
{
    mpr_write(REG_SOFTRESET,0x63);System::Delay(10);
    mpr_write(REG_ECR,0x00);
    mpr_write(REG_MHDR,0x01);mpr_write(REG_NHDR,0x01);
    mpr_write(REG_NCLR,0x10);mpr_write(REG_FDLR,0x20);
    mpr_write(REG_MHDF,0x01);mpr_write(REG_NHDF,0x01);
    mpr_write(REG_NCLF,0x10);mpr_write(REG_FDLF,0x20);
    mpr_write(REG_NHDT,0x00);mpr_write(REG_NCLT,0x00);mpr_write(REG_FDLT,0x00);
    for(uint8_t ch=0;ch<12;ch++){mpr_write(0x41+ch*2,4);mpr_write(0x42+ch*2,2);}
    mpr_write(REG_DEBOUNCE,0x11);
    mpr_write(REG_CONFIG1,0x50);
    mpr_write(REG_CONFIG2,0x7C);  // CDT=32us SFI=18 ESI=1ms
    mpr_write(REG_ECR,0x8C);
    System::Delay(100);
    uint8_t ecr=0;mpr_read(REG_ECR,&ecr,1);
    return(ecr==0x8C);
}

void read_electrodes(uint16_t* out)
{
    uint8_t buf[24];
    mpr_read(REG_ELE0_LSB,buf,24);
    for(int i=0;i<N_CH;i++)
        out[i]=(uint16_t)(buf[i*2]|((buf[i*2+1]&0x03)<<8));
}

void capture_baseline(uint16_t* bl)
{
    uint32_t acc[N_CH]={0};uint16_t tmp[N_CH];
    for(int s=0;s<REBASELINE_SAMPLES;s++){
        read_electrodes(tmp);
        for(int i=0;i<N_CH;i++)acc[i]+=tmp[i];
        System::Delay(5);
    }
    for(int i=0;i<N_CH;i++)bl[i]=(uint16_t)(acc[i]/REBASELINE_SAMPLES);
}

// ── Touch structs ─────────────────────────────────────────────────────────────
struct Finger { bool active; int32_t pos,pressure,peak_ch,peak_delta; };
struct TrackedFinger { bool alive; int32_t pos,pressure; int peak_ch; int32_t peak_delta; };
static TrackedFinger tracked[2]={};

int32_t centroid_window(int32_t* d,int s,int e,int32_t* pk_out,int* pkch_out)
{
    int32_t sw=0,swx=0,pk=0;int pkc=s;
    for(int i=s;i<=e;i++){
        int32_t w=d[i]>0?d[i]:0;
        sw+=w;swx+=w*i;
        if(w>pk){pk=w;pkc=i;}
    }
    if(pk_out)*pk_out=pk;if(pkch_out)*pkch_out=pkc;
    if(sw==0)return -1;
    int32_t p=(swx*1000)/(sw*(N_CH-1));
    if(p<0)p=0;if(p>1000)p=1000;
    return p;
}

int32_t pressure_pct(int32_t pk){
    int32_t range = PRESSURE_MAX_REF - TOUCH_THRESHOLD;
    int32_t raw   = pk - TOUCH_THRESHOLD;
    if(raw <= 0)     return 0;
    if(raw >= range) return 100;
    // Clamp raw strictly before sqrt to prevent Newton overshoot
    if(raw > range) raw = range;
    int32_t scaled = raw * 10000 / range; // 0..10000
    if(scaled > 10000) scaled = 10000;
    // Integer sqrt via Newton — start from 101 (max output) for guaranteed convergence
    int32_t s = 101;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    // Hard clamp output — Newton can land at 101 due to rounding
    if(s < 0)   s = 0;
    if(s > 100) s = 100;
    return s;
}

int detect_raw(int32_t* delta,Finger out[2])
{
    out[0].active=out[1].active=false;
    int pka=0;int32_t da=0;
    for(int i=0;i<N_CH;i++)if(delta[i]>da){da=delta[i];pka=i;}
    if(da<TOUCH_THRESHOLD)return 0;
    int ws=pka-2;if(ws<0)ws=0;int we=pka+2;if(we>=N_CH)we=N_CH-1;
    int32_t pa;int pca;
    int32_t posa=centroid_window(delta,ws,we,&pa,&pca);
    out[0]={true,posa,pressure_pct(pa),pca,pa};

    int32_t d2[N_CH];
    int ss=pka-2;if(ss<0)ss=0;int se=pka+2;if(se>=N_CH)se=N_CH-1;
    for(int i=0;i<N_CH;i++)d2[i]=(i>=ss&&i<=se)?0:delta[i];
    int pkb=0;int32_t db=0;
    for(int i=0;i<N_CH;i++)if(d2[i]>db){db=d2[i];pkb=i;}
    int sep=pkb-pka;if(sep<0)sep=-sep;
    if(db<TOUCH_THRESHOLD||sep<MIN_FINGER_SEP)return 1;
    int ws2=pkb-2;if(ws2<0)ws2=0;int we2=pkb+2;if(we2>=N_CH)we2=N_CH-1;
    int32_t pb;int pcb;
    int32_t posb=centroid_window(d2,ws2,we2,&pb,&pcb);
    out[1]={true,posb,pressure_pct(pb),pcb,pb};
    return 2;
}

void update_tracked(Finger* raw,int n)
{
    bool ra[2]={false,false};
    for(int slot=0;slot<2;slot++){
        if(!tracked[slot].alive)continue;
        int32_t best=MAX_POS_JUMP+1;int br=-1;
        for(int r=0;r<n;r++){
            if(ra[r])continue;
            int32_t c=raw[r].pos-tracked[slot].pos;if(c<0)c=-c;
            if(c<best){best=c;br=r;}
        }
        if(br>=0){
            tracked[slot].pos=raw[br].pos;tracked[slot].pressure=raw[br].pressure;
            tracked[slot].peak_ch=raw[br].peak_ch;tracked[slot].peak_delta=raw[br].peak_delta;
            ra[br]=true;
        } else tracked[slot].alive=false;
    }
    for(int r=0;r<n;r++){
        if(ra[r])continue;
        for(int slot=0;slot<2;slot++){
            if(!tracked[slot].alive){
                tracked[slot]={true,raw[r].pos,raw[r].pressure,raw[r].peak_ch,raw[r].peak_delta};
                ra[r]=true;break;
            }
        }
    }
}

// ── Maths helpers ─────────────────────────────────────────────────────────────
static inline float clampf(float v,float lo,float hi){
    return v<lo?lo:v>hi?hi:v;
}

// Fast tanh approximation (Padé)
static inline float fast_tanh(float x)
{
    if(x> 3.0f)return  1.0f;
    if(x<-3.0f)return -1.0f;
    float x2=x*x;
    return x*(27.0f+x2)/(27.0f+9.0f*x2);
}

// ── Note quantizer ────────────────────────────────────────────────────────────
// Returns the nearest MIDI note in SCALE[] for a raw float MIDI pitch.
float quantize_midi(float midi)
{
    int root = (int)midi;
    int pc   = root % 12;
    int oct  = root / 12;
    // find nearest scale degree
    int best_deg=0, best_dist=127;
    for(int i=0;i<SCALE_LEN;i++){
        int d=SCALE[i]-pc; if(d<0)d=-d;
        int d2=12-d; if(d2<d)d=d2;
        if(d<best_dist){best_dist=d;best_deg=SCALE[i];}
    }
    float q=(float)(oct*12+best_deg);
    // fractional semitone from the quantized note (for slide expression)
    float frac=midi-(float)root;
    return q+frac;
}

// ── Audio state (written by touch loop, read by audio callback) ───────────────
// All volatile — touched by both cores / interrupt context
volatile float g_target_freq    = 65.41f;
volatile float g_target_cutoff  = 100.0f;  // starts dark, pressure opens it
volatile float g_target_detune  = 0.0f;     // semitones osc2 offset
volatile float g_target_drive   = 1.0f;     // waveshaper input gain
volatile float g_vibrato_depth  = 0.0f;     // 0–1, LFO depth
volatile float g_bitcrush       = 0.0f;     // 0–1, amount of crush
volatile float g_ringmod        = 0.0f;     // 0–1, ring mod wet
volatile bool  g_finger_on      = false;
volatile float g_amp_target     = 0.0f;
volatile float g_volume         = 0.9f;  // 0–0.9, set by pot on A0

// ── Audio DSP state (audio callback only) ─────────────────────────────────────
static float s_freq      = 65.41f;
static float s_cutoff    = 100.0f;
static float s_detune    = 0.0f;
static float s_drive     = 1.0f;
static float s_vibdepth  = 0.0f;
static float s_bitcrush  = 0.0f;
static float s_ringmod   = 0.0f;
static float s_amp       = 0.0f;

static float s_phase1    = 0.0f;
static float s_phase2    = 0.0f;
static float s_phase_sub = 0.0f;
static float s_phase_oct = 0.0f; // octave-up oscillator (2x freq)
static float s_phase_rm  = 0.0f; // ring mod carrier phase
static float s_lfo_phase = 0.0f;

// Moog ladder filter state
static float s_flt[4]    = {0,0,0,0};

// Slew rates (per sample at 48 kHz)
// Lower value = faster response
static constexpr float SLEW_FREQ   = 0.9985f; // pitch glide
static constexpr float SLEW_CUT    = 0.9800f; // filter cutoff — fast response
static constexpr float SLEW_MISC   = 0.9900f; // drive/vib
// Finger 2 — asymmetric attack/release
static constexpr float SLEW_F2_A   = 0.9800f; // attack  (~10ms)
static constexpr float SLEW_F2_R   = 0.9990f; // release (~200ms smooth fade)

// Separate attack and release for amplitude envelope
// Attack: very fast (~0.5ms) — snappy pluck feel
// Release: moderate (~80ms) — clean tail when finger lifts
static constexpr float SLEW_AMP_A  = 0.9700f; // attack  (~3ms — snappy but click-free)
static constexpr float SLEW_AMP_R  = 0.999971f; // release (~5 seconds)

// Triangle wave
static inline float tri(float ph){
    return ph<0.5f?(4.0f*ph-1.0f):(3.0f-4.0f*ph);
}

// Moog 4-pole ladder
static inline float moog(float in, float cutoff, float res, float sr)
{
    float f = 2.0f*cutoff/sr;
    if(f>0.98f)f=0.98f;
    float k  = 3.8f*res;
    float fb = k*s_flt[3];
    float x  = in - fb;
    s_flt[0]+=f*(fast_tanh(x)      -fast_tanh(s_flt[0]));
    s_flt[1]+=f*(fast_tanh(s_flt[0])-fast_tanh(s_flt[1]));
    s_flt[2]+=f*(fast_tanh(s_flt[1])-fast_tanh(s_flt[2]));
    s_flt[3]+=f*(fast_tanh(s_flt[2])-fast_tanh(s_flt[3]));
    return s_flt[3];
}

// Wavefolder distortion
// Folds signal back when it exceeds threshold — creates rich harmonics.
// amount 0 = clean, 1 = heavy folding (4 folds)
static inline float wavefold(float in, float amount)
{
    if(amount < 0.01f) return in;
    // Drive the signal harder as amount increases (1x to 5x)
    float driven = in * (1.0f + amount * 4.0f);
    // Fold: reflect signal at ±1 boundary repeatedly
    // fold(x) = 1 - |((x+1) mod 2) - 1|  (triangle fold)
    float x = driven;
    // 2 passes of folding for richness
    for(int fold = 0; fold < 2; fold++) {
        x = x * 0.5f + 0.5f;           // shift to 0..1
        x = x - floorf(x);              // wrap to 0..1
        x = 1.0f - fabsf(2.0f*x - 1.0f); // triangle fold
        x = x * 2.0f - 1.0f;           // back to -1..1
    }
    // Blend dry/wet so at low amounts it stays subtle
    return in*(1.0f-amount) + x*amount;
}

void AudioCallback(AudioHandle::InputBuffer,
                   AudioHandle::OutputBuffer out, size_t size)
{
    // Snapshot targets
    float tfreq  = g_target_freq;
    float tcut   = g_target_cutoff;
    float tdet   = g_target_detune;
    float tdrv   = g_target_drive;
    float tvib   = g_vibrato_depth;
    float tbc    = g_bitcrush;
    float trm    = g_ringmod;
    float tamp   = g_amp_target * g_volume;
    float sr     = hw.AudioSampleRate();

    for(size_t i=0;i<size;i++)
    {
        // Slew all parameters
        s_freq    = s_freq   *SLEW_FREQ + tfreq*(1.0f-SLEW_FREQ);
        // Cutoff: fast when opening (attack), slow when closing (release)
        { float sc = (tcut > s_cutoff) ? SLEW_CUT : SLEW_AMP_R;
          s_cutoff = s_cutoff * sc + tcut * (1.0f - sc); }
        s_drive   = s_drive   * SLEW_MISC + tdrv * (1.0f - SLEW_MISC);
        s_vibdepth= s_vibdepth* SLEW_MISC + tvib * (1.0f - SLEW_MISC);
        // Finger 2 params: fast attack, slow release
        { float sa = tdet     > s_detune   ? SLEW_F2_A : SLEW_F2_R;
          s_detune   = s_detune   * sa + tdet * (1.0f - sa); }
        { float sa = tbc      > s_bitcrush ? SLEW_F2_A : SLEW_F2_R;
          s_bitcrush = s_bitcrush * sa + tbc  * (1.0f - sa); }
        { float sa = trm      > s_ringmod  ? SLEW_F2_A : SLEW_F2_R;
          s_ringmod  = s_ringmod  * sa + trm  * (1.0f - sa); }
        // Asymmetric envelope: fast attack, slower release
        float slew_amp = (tamp > s_amp) ? SLEW_AMP_A : SLEW_AMP_R;
        s_amp = s_amp * slew_amp + tamp * (1.0f - slew_amp);

        // Bleed filter state toward zero when silent
        // Prevents stale filter energy from clicking on next note onset
        if(s_amp < 0.001f) {
            s_flt[0] *= 0.999f;
            s_flt[1] *= 0.999f;
            s_flt[2] *= 0.999f;
            s_flt[3] *= 0.999f;
        }

        // During release, close cutoff in sync with amplitude
        // This prevents clicks from abrupt filter state changes
        if(tamp < 0.001f && s_amp > 0.0001f) {
            // Pull cutoff toward silence-floor proportional to amp decay
            float close_target = 30.0f + s_amp * tcut;
            tcut = close_target;
        }

        // LFO for vibrato (6 Hz sine approximation via triangle)
        s_lfo_phase += 6.0f/sr;
        if(s_lfo_phase>=1.0f)s_lfo_phase-=1.0f;
        float lfo = tri(s_lfo_phase); // -1..1

        // Vibrato: modulate frequency by ±1 semitone * depth
        // powf(2, semitones/12) — approx for small values: exp(x*0.0578)
        float vib_amt = lfo * s_vibdepth * 0.05f; // max ±~0.05 semitones*depth
        float freq_vib = s_freq * (1.0f + vib_amt);

        // Osc 2 frequency: detuned by s_detune semitones
        // pow2(s_detune/12) approximated
        float det_ratio = 1.0f + s_detune * 0.05776f; // linear approx, good for ±1 oct
        float freq2 = freq_vib * det_ratio;

        // Sub osc: one octave down
        float freq_sub = freq_vib * 0.5f;
        // Octave-up osc: one octave above fundamental — adds air and sparkle
        float freq_oct = freq_vib * 2.0f;

        // Ring mod carrier: slightly above fundamental for metallic beating
        float freq_rm = freq_vib * 1.0f; // unison ring mod (can offset for effect)

        // Oscillators
        float osc1    = tri(s_phase1)   * 0.50f;
        float osc2    = tri(s_phase2)   * 0.22f; // detuned copy
        float sub     = tri(s_phase_sub)* 0.18f; // one octave below
        float osc_oct = tri(s_phase_oct)* 0.15f; // one octave above — bright shimmer

        // Mix — reduce osc1 slightly to make room for the new octave osc
        float mix = osc1 + osc2 + sub + osc_oct;

        // Ring modulation (finger 2 pressure → metallic/bell edge)
        float rm_carrier = tri(s_phase_rm);
        mix = mix*(1.0f-s_ringmod) + (mix*rm_carrier)*s_ringmod;

        // Waveshaper drive before filter (adds harmonics, warms tone)
        mix = fast_tanh(mix * s_drive);

        // Moog ladder filter — no attack boost (was causing onset clicks)
        float eff_cutoff = clampf(s_cutoff, 20.0f, 20000.0f);
        float filtered = moog(mix, eff_cutoff, 0.75f, sr);

        // Wavefold distortion (finger 2 pressure)
        float crushed = wavefold(filtered, s_bitcrush);

        // Soft clip output
        float sample = fast_tanh(crushed * 1.3f) * s_amp;

        out[0][i] = sample;
        out[1][i] = sample;

        // Advance phases
        s_phase1   += freq_vib/sr;  if(s_phase1  >=1.0f)s_phase1  -=1.0f;
        s_phase2   += freq2   /sr;  if(s_phase2  >=1.0f)s_phase2  -=1.0f;
        s_phase_sub+= freq_sub/sr;  if(s_phase_sub>=1.0f)s_phase_sub-=1.0f;
        s_phase_oct+= freq_oct/sr;  if(s_phase_oct>=1.0f)s_phase_oct-=1.0f;
        s_phase_rm += freq_rm /sr;  if(s_phase_rm >=1.0f)s_phase_rm -=1.0f;
    }
}


// ── Main ──────────────────────────────────────────────────────────────────────
int main()
{
    hw.Init();
    hw.SetAudioBlockSize(4);

    // ── ADC — potentiometer on A0 (D15) ──────────────────────────────────────
    AdcChannelConfig adcConfig;
    adcConfig.InitSingle(A0);
    hw.adc.Init(&adcConfig, 1);
    hw.adc.Start();

    // ── I2C pins + MPR121 init BEFORE audio starts ────────────────────────────
    // The audio ISR firing during bit-bang I2C corrupts I2C timing.
    // Do all sensor work first, then start the audio engine.
    // SDA: open-drain + internal pull-up — no mode switching needed during I2C
    // SCL: push-pull — master always drives it
    // VERY_HIGH speed for maximum slew rate at ~250kHz
    sda.Init(SDA_PIN, GPIO::Mode::OUTPUT_OD, GPIO::Pull::PULLUP,  GPIO::Speed::VERY_HIGH);
    scl.Init(SCL_PIN, GPIO::Mode::OUTPUT,    GPIO::Pull::NOPULL,  GPIO::Speed::VERY_HIGH);
    sda.Write(true);
    scl.Write(true);

    uint16_t baseline[N_CH];
    if(mpr_init())
    {
        System::Delay(300);
        capture_baseline(baseline);
    }
    mpr_irq_init();
    // If MPR121 fails we still start audio so the problem is audible (silence)

    // ── Start audio ───────────────────────────────────────────────────────────
    // Set initial targets before callback fires
    g_target_freq   = 130.81f;  // C3
    g_target_cutoff = 100.0f;
    g_amp_target    = 0.0f;
    hw.StartAudio(AudioCallback);


    uint16_t data[N_CH];
    int32_t  delta[N_CH];
    Finger   raw[2];

    uint32_t idle_since      = System::GetNow();
    bool     f1_was_on       = false;
    uint32_t f1_last_seen_ms = 0;
    static constexpr uint32_t F1_REQUANTIZE_MS = 200;
    float    f1_midi_base    = 60.0f;

    bool currently_touching = false;

    while(true)
    {
        // ── Volume potentiometer (A0) — 0..1 ADC → 0..0.9 gain ───────────────
        g_volume = hw.adc.GetFloat(0) * 0.9f;

        // ── IRQ gate: skip I2C read when idle and sensor has nothing to report ─
        // Use both edge flag (ISR wake) and pin level (MPR121 OUTA stays LOW
        // while any ELE bit is set — no mpr_clear_irq needed, level drops
        // naturally when the finger lifts and ELE bits clear).
        const bool irq_asserted = !(GPIOB->IDR & GPIO_PIN_5); // active-low
        if(!g_mpr_irq && !irq_asserted && !currently_touching)
        {
            // Still service the rebaseline timer while idle
            if(System::GetNow() - idle_since >= REBASELINE_IDLE_MS)
            {
                tracked[0].alive = tracked[1].alive = false;
                capture_baseline(baseline);
                idle_since = System::GetNow();
            }
            System::Delay(1);
            continue;
        }
        g_mpr_irq = false; // consume edge flag; level check keeps us reading

        read_electrodes(data);

        for(int i=0;i<N_CH;i++){
            delta[i]=(int32_t)baseline[i]-(int32_t)data[i];
            if(delta[i]<0)delta[i]=0;
        }

        int n_raw=detect_raw(delta,raw);
        bool touching=(n_raw>0);
        currently_touching = touching;

        // ── Auto-rebaseline ───────────────────────────────────────────────────
        if(touching){
            idle_since=System::GetNow();
        } else {
            if(System::GetNow()-idle_since>=REBASELINE_IDLE_MS){
                tracked[0].alive=tracked[1].alive=false;
                capture_baseline(baseline);
                idle_since=System::GetNow();
            }
        }

        update_tracked(raw,n_raw);

        // ── Finger 1 → Pitch + Filter cutoff + Vibrato ───────────────────────
        bool f1_on = tracked[0].alive;
        if(f1_on) f1_last_seen_ms = System::GetNow();
        bool f1_fresh = !f1_was_on && (System::GetNow() - f1_last_seen_ms >= F1_REQUANTIZE_MS);

        if(f1_on)
        {
            float pos01 = clampf((float)tracked[0].pos / 1000.0f, 0.0f, 1.0f);
            float prs01 = clampf((float)tracked[0].pressure / 100.0f, 0.0f, 1.0f); // hard clamp

            // ── Direct exponential frequency mapping: 50Hz to 300Hz ─────────
            // freq = 50 * (300/50)^pos01 = 50 * 6^pos01
            // Using e^x approx: 6^x = e^(x*ln6), ln6=1.7918
            float earg = pos01 * 1.7918f;
            float ex   = 1.0f + earg*(1.0f + earg*(0.5f + earg*0.1667f));
            float freq_continuous = clampf(50.0f * ex, 50.0f, 300.0f);

            // Convert to MIDI for quantizer: midi = 69 + 12*log2(freq/440)
            // log2(x) = ln(x)/ln(2); ln approx via y=(x-1)/(x+1) series
            float freq_ratio = freq_continuous / 440.0f;
            float y = (freq_ratio - 1.0f) / (freq_ratio + 1.0f);
            float y2 = y*y;
            float ln_ratio = 2.0f*y*(1.0f + y2*(0.3333f + y2*0.2f));
            float midi_raw = 69.0f + 17.3123f * ln_ratio; // 17.3123 = 12/ln(2)

            float freq; // final frequency to use

            if(QUANTIZE_ENABLED && f1_fresh)
            {
                // Touch-down: snap to nearest chromatic note
                f1_midi_base = quantize_midi(midi_raw);
                // Convert quantized MIDI back to Hz
                float qe = (f1_midi_base - 69.0f) * 0.05776f;
                freq = clampf(440.0f*(1.0f+qe*(1.0f+qe*(0.5f+qe*0.1667f))), 50.0f, 300.0f);
            }
            else
            {
                // Sliding or quantize disabled: follow continuous frequency directly
                f1_midi_base = f1_midi_base*0.88f + midi_raw*0.12f;
                freq = freq_continuous;
            }
            freq = clampf(freq, 50.0f, 300.0f);

            // Cutoff: base tracks 2× pitch, pressure opens it up
            // Light press = 1× pitch (dark, Moog-like tracking)
            // Hard press = up to 12000 Hz (wide open)
            // ── Moog-style exponential cutoff ─────────────────────────────
            // At zero pressure: cutoff = 0.5x pitch (below fundamental, very dark)
            // At full pressure: cutoff = 0.5x * 2^(6*prs) — exponential opening
            // 6 octaves of sweep = factor of 64x, so fully open = 32x pitch
            // This gives the classic Moog "closed → wah → bright" feel.
            // Power-3 on pressure so the bottom half stays dark and controlled.
            // ── Smootherstep pressure curves (6x⁵ - 15x⁴ + 10x³) ──────────
            // Starts slow, steep in the middle, eases gently at the top.
            // Much more playable than power curves — feels like a real knob.
            auto smoothstep = [](float x) -> float {
                x = x<0?0:x>1?1:x;
                return x*x*(3.0f - 2.0f*x);
            };
            auto smootherstep = [](float x) -> float {
                x = x<0?0:x>1?1:x;
                return x*x*x*(x*(x*6.0f - 15.0f) + 10.0f);
            };

            // Cutoff: smootherstep — very dark at low, dramatic in mid, eases at top
            float prs_cut  = smootherstep(prs01);
            float cutoff_oct = 12.0f * prs_cut;              // 12 octaves of sweep
            float oct_arg  = cutoff_oct * 0.6931f;
            float oct_mult = 1.0f + oct_arg*(1.0f + oct_arg*(0.5f + oct_arg*(0.1667f + oct_arg*0.0417f)));
            float cutoff   = clampf(freq * 0.3f * oct_mult, 20.0f, 18000.0f); // screams

            // Vibrato: smoothstep, only activates above 60% pressure
            float vib_in = clampf((prs01 - 0.6f) / 0.4f, 0.0f, 1.0f);
            float vib    = smoothstep(vib_in) * 0.8f;

            // Drive: smoothstep — clean at low, progressively warmer
            float drive = 1.0f + smoothstep(prs01) * 2.5f;

            g_target_freq    = freq;
            g_target_cutoff  = cutoff;
            g_vibrato_depth  = vib;
            g_target_drive   = drive;
            g_finger_on      = true;
            g_amp_target     = 0.72f;
        }
        else
        {
            // Finger lifted: gate off
            g_finger_on  = false;
            g_amp_target = 0.0f;
            // Do NOT reset cutoff here — let it close with the amplitude
            // in the audio callback to avoid discontinuity clicks
        }

        f1_was_on = f1_on;

        // ── Finger 2 → Detune + Bitcrush + Ring mod ──────────────────────────
        if(tracked[1].alive)
        {
            float pos01 = clampf((float)tracked[1].pos / 1000.0f, 0.0f, 1.0f);
            float prs01 = clampf((float)tracked[1].pressure / 100.0f, 0.0f, 1.0f); // hard clamp

            // Detune: finger 2 position maps to ±7 semitones (±1 fifth)
            // Centre (pos=0.5) = no detune, left = flat, right = sharp
            float detune_pos  = (pos01 - 0.5f) * 3.0f;  // ±1.5 semitones raw

            // Wavefold: smootherstep — barely there until deliberate pressure
            auto smootherstep2 = [](float x) -> float {
                x = x<0?0:x>1?1:x;
                return x*x*x*(x*(x*6.0f - 15.0f) + 10.0f);
            };
            // Apply smootherstep twice and cap at 0.20 — effect only arrives
            // with real deliberate pressure, stays subtle even at max
            float ss2 = smootherstep2(prs01);
            float ss3 = smootherstep2(ss2);   // triple smootherstep — very lazy
            float bc  = smootherstep2(ss3) * 0.15f; // needs real deliberate pressure

            // Detune: smoothstep on position offset from centre
            auto smoothstep2 = [](float x) -> float {
                x = x<0?0:x>1?1:x;
                return x*x*(3.0f - 2.0f*x);
            };
            // Detune amount scales with pressure via smoothstep — no detune at low pressure
            float det_prs = smoothstep2(prs01);

            // Ring mod: smoothstep, only above 85% pressure
            float rm_in = clampf((prs01 - 0.85f) / 0.15f, 0.0f, 1.0f);
            float rm    = smoothstep2(rm_in) * 0.15f;

            g_target_detune = detune_pos * det_prs; // pressure gates detune amount
            g_bitcrush      = bc;
            g_ringmod       = rm;
        }
        else
        {
            // No second finger: return detune to zero, clear effects
            g_target_detune = 0.0f;
            g_bitcrush      = 0.0f;
            g_ringmod       = 0.0f;
        }

    }
}
