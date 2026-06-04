// TinyUSB glue — see usb_glue.h. STM32H750 USB Full-Speed device (USB2_OTG_FS
// on PA11/PA12 = TinyUSB rhport 0).
#include "tusb.h"
#include "usb_glue.h"
#include <stdarg.h>
#include <stdio.h>
extern "C" {
#include "stm32h7xx_hal.h"
}

// ── Board glue + device init ──────────────────────────────────────────────────
extern "C" void usb_fs_init(void)
{
    // hw.Init() already configured the USB clock source (HSI48) and VDDUSB; here we
    // do only the MSP-level setup libDaisy would otherwise do in its USBD MspInit.
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {};
    g.Pin       = GPIO_PIN_11 | GPIO_PIN_12;     // OTG_FS DM / DP
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF10_OTG1_FS;
    HAL_GPIO_Init(GPIOA, &g);

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    // Below the audio DMA IRQ so streaming is never starved by USB servicing.
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);

    tud_init(BOARD_TUD_RHPORT);                  // enables the NVIC IRQ internally
}

// USB global interrupt → TinyUSB. This is the strong override of the startup weak
// OTG_FS_IRQHandler; works only because we no longer link libDaisy's USB (usb.cpp),
// which would otherwise define this same symbol.
extern "C" void OTG_FS_IRQHandler(void) { tud_int_handler(0); }

extern "C" void usb_task(void) { tud_task(); }

// ── CDC TX (printf-style logger) ──────────────────────────────────────────────
static void cdc_write_all(const char *data, uint32_t len)
{
    if(!tud_cdc_connected()) return;             // no host listening → drop
    uint32_t off = 0, guard = 0;
    while(off < len) {
        uint32_t avail = tud_cdc_write_available();
        if(avail) {
            uint32_t chunk = (len - off < avail) ? (len - off) : avail;
            off += tud_cdc_write(data + off, chunk);
        } else {
            tud_cdc_write_flush();
            tud_task();                          // pump so the IN endpoint drains
            if(++guard > 2000) break;            // host stalled → give up, don't hang
        }
    }
    tud_cdc_write_flush();
}

extern "C" void usb_log(const char *fmt, ...)
{
    char buf[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);
    if(n < 0) return;
    if(n > (int)sizeof(buf) - 2) n = sizeof(buf) - 2;
    buf[n++] = '\r';
    buf[n++] = '\n';
    cdc_write_all(buf, (uint32_t)n);
}

// ── CDC RX ──────────────────────────────────────────────────────────────────
extern "C" int usb_cdc_read_avail(char *buf, int maxlen)
{
    if(maxlen <= 0 || !tud_cdc_available()) return 0;
    return (int)tud_cdc_read(buf, (uint32_t)maxlen);
}

extern "C" int usb_cdc_is_connected(void) { return tud_cdc_connected() ? 1 : 0; }
