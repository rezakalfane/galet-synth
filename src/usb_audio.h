// UAC2 capture glue (Phase 2): the engine pushes its stereo output here; the USB
// audio class drains it to the isochronous IN endpoint. Plain C linkage so the
// C++ audio callback and the C TinyUSB callbacks share it.
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// Push one stereo frame (16-bit) from the audio callback into the capture ring.
// No-op unless a host has the audio interface streaming. Lock-free SPSC.
void usb_audio_push(int16_t left, int16_t right);

// 1 while the host is streaming the audio IN endpoint (alt setting 1).
int usb_audio_is_streaming(void);

// Init the UAC2 control state (sample-rate range etc.). Call from usb_fs_init().
void usb_audio_init(void);

#ifdef __cplusplus
}
#endif
