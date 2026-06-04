# Glass Moog Synth — Daisy Seed Makefile
# Requires libDaisy in the parent directory: ../libDaisy
# Usage:
#   make              — build the synth (runs from internal flash, APP_TYPE=BOOT_NONE)
#   make program-dfu  — flash via USB DFU bootloader
#   make clean        — remove build artifacts
#
# QSPI build (runs the app from QSPI flash via the Daisy bootloader — needed once
# the firmware outgrows the 128 KB internal flash, e.g. for the USB-audio work):
#   make program-boot                 — install the Daisy bootloader (once, in DFU mode)
#   make APP_TYPE=BOOT_QSPI ...        — build the app for QSPI (lands at 0x90040000)
#   make APP_TYPE=BOOT_QSPI program-dfu — load the QSPI app via the bootloader
# Pass the same libdaisy flags as a normal build. See docs/usb-audio-plan.md (Phase 0).
#
# To build a tool instead:
#   make TARGET=tools/mpr121_reader
#   make TARGET=tools/i2c_scanner
#   make TARGET=tools/osc_test
#   make TARGET=tools/mpr121_glass_tuner
#   make TARGET=tools/mpr121_glass_tuner2
#   make TARGET=tools/mpr121_calibrate
#   make TARGET=tools/test-slider

TARGET ?= src/main

C_SOURCES   =
CPP_SOURCES = $(TARGET).cpp

# The synth (src/main) is split across modules; the tools stay standalone.
ifeq ($(TARGET),src/main)
CPP_SOURCES += src/mpr121.cpp src/touch.cpp src/engine.cpp src/serialtune.cpp src/persist.cpp
CPP_SOURCES += src/usb_glue.cpp
# TinyUSB (vendored subset, lib/tinyusb) + our descriptors — the CDC device that
# replaces libDaisy's USB logger (Phase 1 of the USB-audio work). CFG_TUSB_MCU is
# set in src/tusb_config.h, found via -Isrc.
TINYUSB = lib/tinyusb/src
C_SOURCES   += src/usb_descriptors.c src/usb_audio.c \
  $(TINYUSB)/tusb.c $(TINYUSB)/common/tusb_fifo.c \
  $(TINYUSB)/device/usbd.c $(TINYUSB)/device/usbd_control.c \
  $(TINYUSB)/class/cdc/cdc_device.c \
  $(TINYUSB)/class/audio/audio_device.c \
  $(TINYUSB)/portable/synopsys/dwc2/dcd_dwc2.c
C_INCLUDES  += -I$(TINYUSB) -Isrc
# Run from SRAM via the Daisy bootloader: big enough for the USB-audio work AND
# keeps QSPI writable so voice persistence (storage.Save) works. Big audio buffers
# live in SDRAM (DSY_SDRAM_BSS) so data fits the 128 KB DTCM. Override on the CLI.
APP_TYPE ?= BOOT_SRAM
endif

libdaisy_dir = ../libDaisy

include $(libdaisy_dir)/core/Makefile
