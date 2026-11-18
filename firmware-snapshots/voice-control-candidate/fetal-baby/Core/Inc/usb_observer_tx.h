#ifndef USB_OBSERVER_TX_H
#define USB_OBSERVER_TX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define USB_OBSERVER_FRAME_SIZE 1088U
#define USB_OBSERVER_STALL_MS 500U
#if defined(BABY_USB_VISION_UPLINK)
#define USB_OBSERVER_LOG_MUX 1
#endif

typedef struct {
    uint32_t submitted, completed, dropped, busy, errors, stalls, discarded_rx_bytes;
    uint32_t queued, coalesced, queue_drops;
} UsbObserver_Stats;

/* Boot initialization; writes and Task run only in main context. */
void UsbObserver_Init(void);
bool UsbObserver_Write(const void *data, size_t length);
/* Main context only. In mux builds, success means copied/queued, not delivered. */
bool UsbObserver_WritePriority(const void *data, size_t length, bool alert);
void UsbObserver_Task(void);
void UsbObserver_GetStats(UsbObserver_Stats *stats);
/* USB callbacks publish state only; OUT payloads never reach the application. */
void UsbObserver_OnSessionReset(void);
void UsbObserver_OnControlLines(uint16_t lines);
void UsbObserver_DiscardRx(uint32_t length);

#endif
