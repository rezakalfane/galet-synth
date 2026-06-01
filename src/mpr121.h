#pragma once
// MPR121 capacitive touch sensor — bit-bang I2C driver (SDA=D12, SCL=D11).
#include "daisy_seed.h"

static constexpr daisy::Pin SDA_PIN  = daisy::seed::D12;
static constexpr daisy::Pin SCL_PIN  = daisy::seed::D11;
static constexpr uint8_t    MPR_ADDR = 0x5A;
static constexpr uint32_t   HP       = 5;   // I2C half-period (us)

static constexpr uint8_t REG_ELE0_LSB  = 0x04;
static constexpr uint8_t REG_MHDR=0x2B,REG_NHDR=0x2C,REG_NCLR=0x2D,REG_FDLR=0x2E;
static constexpr uint8_t REG_MHDF=0x2F,REG_NHDF=0x30,REG_NCLF=0x31,REG_FDLF=0x32;
static constexpr uint8_t REG_NHDT=0x33,REG_NCLT=0x34,REG_FDLT=0x35;
static constexpr uint8_t REG_DEBOUNCE=0x5B,REG_CONFIG1=0x5C,REG_CONFIG2=0x5D;
static constexpr uint8_t REG_ECR=0x5E,REG_SOFTRESET=0x80;

// Public driver API (impl in mpr121.cpp; all blocking, call outside the audio ISR):
void mpr_idle_pins();              // set SDA/SCL to their idle (high) state
bool mpr_init();                   // configure the sensor; true if it acked OK
void read_electrodes(uint16_t* out);   // N_CH filtered electrode values
void capture_baseline(uint16_t* bl);   // averaged N_CH baseline (REBASELINE_SAMPLES)
