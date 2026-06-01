# Host unit tests for the pure (hardware-free) modules — no Daisy, no gtest.
# Run from the repo root with:  make -C test     (or: cd test && make)
CXX      ?= c++
# -Wno-missing-field-initializers: voice presets intentionally omit trailing
# fields (they zero-default — see voice.h).
CXXFLAGS  = -std=c++17 -I../src -Wall -Wextra -Wno-missing-field-initializers -O0 -g
SRCS      = test_main.cpp ../src/touch.cpp
BIN       = /tmp/galetsynth_tests

test: $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(BIN) $(SRCS)
	$(BIN)

.PHONY: test
