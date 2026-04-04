# Glass Moog Synth — Daisy Seed Makefile
# Requires libDaisy in the parent directory: ../libDaisy
# Usage:
#   make              — build the synth
#   make program-dfu  — flash via USB DFU bootloader
#   make clean        — remove build artifacts
#
# To build a tool instead:
#   make TARGET=tools/mpr121_reader
#   make TARGET=tools/i2c_scanner
#   make TARGET=tools/osc_test
#   make TARGET=tools/mpr121_glass_tuner
#   make TARGET=tools/mpr121_glass_tuner2

TARGET ?= src/main

C_SOURCES   =
CPP_SOURCES = $(TARGET).cpp

LDFLAGS = -specs=nano.specs -specs=nosys.specs

libdaisy_dir = ../libDaisy

include $(libdaisy_dir)/core/Makefile
