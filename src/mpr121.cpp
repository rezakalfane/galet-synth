#include "mpr121.h"
#include "config.h"     // N_CH, REBASELINE_SAMPLES

using namespace daisy;
using namespace daisy::seed;

// GPIO lines owned by the driver.
static GPIO sda, scl;

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
    mpr_write(REG_CONFIG1,0xF0);  // FFI=34 | CDC=48uA
    mpr_write(REG_CONFIG2,0x4C);  // CDT=4us | SFI=18 | ESI=1ms
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

// Set both lines to idle (high) — call once before mpr_init().
void mpr_idle_pins(){ sda_high(); scl_high(); }
