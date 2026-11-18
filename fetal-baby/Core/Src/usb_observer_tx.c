#include "usb_observer_tx.h"
#include "baby_care_hal.h"
#include "usbd_cdc.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t s_buffer[USB_OBSERVER_FRAME_SIZE];
static volatile uint8_t s_dtr;
static volatile uint32_t s_session;
static uint8_t s_active, s_stalled;
static uint32_t s_active_session, s_started;
static volatile UsbObserver_Stats s_stats;

static USBD_CDC_HandleTypeDef *configured_class(void)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { return NULL; }
    return (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
}

void UsbObserver_Init(void)
{
    s_dtr = s_active = s_stalled = 0U;
    s_session = s_active_session = s_started = 0U;
    s_stats = (UsbObserver_Stats){0};
}

void UsbObserver_OnSessionReset(void)
{
    s_dtr = 0U;
    s_session++;
}

void UsbObserver_OnControlLines(uint16_t lines)
{
    s_dtr = (uint8_t)((lines & 1U) != 0U);
}

void UsbObserver_DiscardRx(uint32_t length)
{
    s_stats.discarded_rx_bytes += length;
}

void UsbObserver_Task(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_active != 0U) {
        USBD_CDC_HandleTypeDef *cdc = configured_class();
        if (s_active_session != s_session) {
            /* The USB class deinit/reset has aborted the old endpoint transfer. */
            s_stats.dropped++;
            s_active = 0U;
        } else if (cdc != NULL && cdc->TxState == 0U) {
            s_stats.completed++;
            s_active = 0U;
        } else if (s_stalled == 0U && (HAL_GetTick() - s_started) >= USB_OBSERVER_STALL_MS) {
            s_stats.stalls++;
            s_stalled = 1U;
            /* A timeout does not release a buffer still owned by the USB stack. */
        }
    }
    __set_PRIMASK(primask);
}

bool UsbObserver_Write(const void *data, size_t length)
{
    USBD_CDC_HandleTypeDef *cdc;
    uint32_t primask, session;
    uint8_t status;
    if (data == NULL || length == 0U || length > sizeof(s_buffer)) {
        s_stats.dropped++;
        return false;
    }
    UsbObserver_Task();
    primask = __get_PRIMASK();
    __disable_irq();
    cdc = configured_class();
    if (s_dtr == 0U || cdc == NULL) {
        s_stats.dropped++;
        __set_PRIMASK(primask);
        return false;
    }
    if (cdc->TxState != 0U) {
        s_stats.busy++;
        s_stats.dropped++;
        __set_PRIMASK(primask);
        return false;
    }
    session = s_session;
    __set_PRIMASK(primask);
    /* Only this main-context producer submits data; IRQ callbacks never write it. */
    memcpy(s_buffer, data, length);
    primask = __get_PRIMASK();
    __disable_irq();
    cdc = configured_class();
    if (session != s_session || s_dtr == 0U || cdc == NULL || cdc->TxState != 0U) {
        s_stats.dropped++;
        __set_PRIMASK(primask);
        return false;
    }
    status = USBD_CDC_SetTxBuffer(&hUsbDeviceFS, s_buffer, (uint32_t)length);
    if (status == USBD_OK) { status = USBD_CDC_TransmitPacket(&hUsbDeviceFS); }
    if (status == USBD_OK) {
        s_active = 1U;
        s_stalled = 0U;
        s_active_session = session;
        s_started = HAL_GetTick();
        s_stats.submitted++;
    } else {
        s_stats.errors++;
        s_stats.dropped++;
    }
    __set_PRIMASK(primask);
    return status == USBD_OK;
}

void UsbObserver_GetStats(UsbObserver_Stats *stats)
{
    if (stats != NULL) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        *stats = s_stats;
        __set_PRIMASK(primask);
    }
}
