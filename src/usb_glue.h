// TinyUSB glue for GaletSynth (Phase 1): board init, the USB IRQ, the device
// task pump, and a printf-style logger + CDC read that replace libDaisy's USB
// logger (hw.PrintLine / hw.usb_handle). Plain C linkage so the C IRQ vector and
// the C++ firmware can both call in.
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// MSP glue (GPIO PA11/PA12 AF10 + OTG_FS clock) then tud_init(). The system USB
// clock (HSI48) + VDDUSB are already set up by hw.Init(). Call after hw.Init().
void usb_fs_init(void);

// Service the TinyUSB device stack. Call frequently from the control loop.
void usb_task(void);

// printf + CRLF → the CDC port (drop-in for hw.PrintLine). Integer/%s/%c/%0Nd
// formats only (no %f) — same as the old logger.
void usb_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Read up to maxlen bytes from the CDC RX FIFO (non-blocking). Returns count.
int usb_cdc_read_avail(char *buf, int maxlen);

// 1 once a host has opened the CDC port (DTR), else 0.
int usb_cdc_is_connected(void);

#ifdef __cplusplus
}
#endif
