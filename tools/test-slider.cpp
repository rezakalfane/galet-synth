/**
 * test-slider.cpp — MPR121 sensor display tool
 * Daisy Seed — Bit-Bang I2C  SDA=D12  SCL=D11
 *
 * Shows live electrode deltas, finger position and pressure bars.
 * No audio — use this to tune touch parameters in src/main.cpp.
 *
 * Build & flash:
 *   make TARGET=tools/test-slider
 *   dfu-util -a 0 -s 0x08000000:leave -D build/tools/test-slider.bin
 * Monitor:
 *   screen /dev/tty.usbmodem* 115200
 */

#include "daisy_seed.h"

using namespace daisy;
using namespace daisy::seed;

// ── Pin / I2C config ──────────────────────────────────────────────────────────
static constexpr Pin      SDA_PIN  = D12;
static constexpr Pin      SCL_PIN  = D11;
static constexpr uint8_t  MPR_ADDR = 0x5A;
static constexpr uint32_t HP       = 5;
static constexpr int      N_CH     = 12;

// ── Touch tuning — mirror values from src/main.cpp ───────────────────────────
static constexpr int32_t  TOUCH_THRESHOLD    = 10;
static constexpr int32_t  PRESSURE_MAX_REF[12] = {
    100,60,60,60,60,60,
    60,60,60,60,60,100
};
static constexpr int32_t  PRESSURE_DEAD_ZONE[12] = {
    30,23,13,13,13,13,
    13,13,13,13,23,35
};
static constexpr int32_t  MIN_FINGER_SEP     = 4;
static constexpr uint32_t REBASELINE_IDLE_MS = 2000;
static constexpr int      REBASELINE_SAMPLES = 32;
static constexpr int32_t  MAX_POS_JUMP       = 200;

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
    mpr_write(REG_CONFIG2,0x6C);
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

int32_t pressure_pct(int32_t pk, int ch)
{
    int32_t range = PRESSURE_MAX_REF[ch] - PRESSURE_DEAD_ZONE[ch];
    int32_t raw   = pk - PRESSURE_DEAD_ZONE[ch];
    if(raw <= 0)     return 0;
    if(raw >= range) return 100;
    int32_t scaled = raw * 10000 / range;
    if(scaled > 10000) scaled = 10000;
    int32_t s = scaled / 2 + 1;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    s = (s + scaled/s) / 2;
    if(s < 0)   s = 0;
    if(s > 100) s = 100;
    return s;
}

int detect_raw(int32_t* delta, Finger out[2])
{
    out[0].active=out[1].active=false;
    int pka=0;int32_t da=0;
    for(int i=0;i<N_CH;i++)if(delta[i]>da){da=delta[i];pka=i;}
    if(da<TOUCH_THRESHOLD)return 0;
    int ws=pka-2;if(ws<0)ws=0;int we=pka+2;if(we>=N_CH)we=N_CH-1;
    int32_t pa;int pca;
    int32_t posa=centroid_window(delta,ws,we,&pa,&pca);
    out[0]={true,posa,pressure_pct(pa,pca),pca,pa};

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
    out[1]={true,posb,pressure_pct(pb,pcb),pcb,pb};
    return 2;
}

void update_tracked(Finger* raw, int n)
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

// ── Display helpers ───────────────────────────────────────────────────────────
void make_bar(char* out, int32_t val, int32_t max_val, int width)
{
    if(max_val <= 0) max_val = 1;
    int filled = (int)(val * width / max_val);
    if(filled < 0)     filled = 0;
    if(filled > width) filled = width;
    out[0] = '[';
    for(int i=0;i<width;i++) out[1+i] = (i < filled) ? '#' : ' ';
    out[1+width] = ']';
    out[2+width] = '\0';
}

void make_pos_bar(char* out, int width)
{
    for(int i=0;i<width+2;i++) out[i]=' ';
    out[width+2]='\0';
    out[0]='['; out[width+1]=']';
    for(int slot=0;slot<2;slot++){
        if(!tracked[slot].alive) continue;
        int p=(int)(((int64_t)tracked[slot].pos*(width-1)+500)/1000);
        if(p<0)p=0;if(p>=width)p=width-1;
        out[1+p]=(char)('1'+slot);
    }
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main()
{
    hw.Init();
    hw.StartLog(false);
    sda_high(); scl_high();

    hw.PrintLine("MPR121 Slider Test");
    hw.PrintLine("==================");

    if(!mpr_init())
        hw.PrintLine("MPR121 INIT FAILED");
    else
        hw.PrintLine("MPR121 OK");

    uint16_t baseline[N_CH];
    capture_baseline(baseline);
    hw.PrintLine("Baseline captured. Slide away.\n");

    uint16_t data[N_CH];
    int32_t  delta[N_CH];
    Finger   raw[2];
    char     bar[32], pos_bar[52];

    uint32_t idle_since    = System::GetNow();
    uint32_t last_print_ms = 0;

    while(true)
    {
        read_electrodes(data);

        int32_t max_delta = 1;
        for(int i=0;i<N_CH;i++){
            delta[i]=(int32_t)baseline[i]-(int32_t)data[i];
            if(delta[i]<0)delta[i]=0;
            if(delta[i]>max_delta)max_delta=delta[i];
        }

        int n_raw = detect_raw(delta, raw);
        bool touching = (n_raw > 0);

        if(touching){
            idle_since = System::GetNow();
        } else {
            if(System::GetNow()-idle_since >= REBASELINE_IDLE_MS){
                capture_baseline(baseline);
                hw.PrintLine("[rebaseline]");
                idle_since = System::GetNow();
            }
        }

        update_tracked(raw, n_raw);

        // ── Display at ~10 Hz ─────────────────────────────────────────────────
        uint32_t now = System::GetNow();
        if(now - last_print_ms < 100) continue;
        last_print_ms = now;

        hw.PrintLine(" CH | DELTA | BAR");
        hw.PrintLine("----+-------+--------------------");
        for(int i=0;i<N_CH;i++){
            make_bar(bar, delta[i], max_delta, 20);
            char mk=' ';
            for(int s=0;s<2;s++)
                if(tracked[s].alive && i==tracked[s].peak_ch) mk=(char)('1'+s);
            hw.PrintLine(" %2d | %5d | %s %c", i, (int)delta[i], bar, mk);
        }

        make_pos_bar(pos_bar, 40);
        hw.PrintLine("\nPOS  %s", pos_bar);

        for(int slot=0;slot<2;slot++){
            if(tracked[slot].alive){
                make_bar(bar, tracked[slot].pressure, 100, 16);
                hw.PrintLine("  %d  pos:%4d  prs:%3d%%  %s",
                    slot+1, (int)tracked[slot].pos, (int)tracked[slot].pressure, bar);
            } else {
                hw.PrintLine("  %d  pos: ---  prs: --%%  [                ]", slot+1);
            }
        }
        hw.PrintLine("--------------------------------------------");
    }
}
