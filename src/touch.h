#pragma once
// Finger detection & tracking — pure logic over electrode deltas (testable).
#include <cstdint>

// Up to 4 fingers tracked (for the polyphonic Drums MultiVoice). The mono
// melodic path still only uses tracked[0] (pitch) and tracked[1] (effects).
static constexpr int MAX_FINGERS = 4;
struct Finger { bool active; int32_t pos,pressure,peak_ch,peak_delta; };
struct TrackedFinger { bool alive; int32_t pos,pressure; int peak_ch; int32_t peak_delta; };
extern TrackedFinger tracked[MAX_FINGERS];

int32_t centroid_window(int32_t* d, int s, int e, int32_t* pk_out, int* pkch_out);
int32_t pressure_pct(int32_t pk, int ch);
int     detect_raw(int32_t* delta, Finger* out, int maxf);
void    update_tracked(Finger* raw, int n);
