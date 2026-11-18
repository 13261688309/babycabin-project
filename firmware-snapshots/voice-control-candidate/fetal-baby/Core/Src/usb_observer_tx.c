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
static bool submit_now(const void *data, size_t length, uint32_t expected_epoch);
#if defined(USB_OBSERVER_LOG_MUX)
#define PENDING_SLOTS 9U
#define ALERT_BEGIN 7U
#define USB_RETRY_MS 10U
typedef struct {
    uint8_t bytes[USB_OBSERVER_FRAME_SIZE];
    uint16_t length;
    uint8_t occupied;
    uint32_t order;
} Pending;
static Pending s_pending[PENDING_SLOTS];
static volatile uint32_t s_queue_epoch;
static uint32_t s_pending_epoch, s_order, s_retry_ms;
static uint8_t s_alert_streak, s_backoff;
static void drive_queue(void);

static void clear_pending(void)
{
    for (unsigned i = 0U; i < PENDING_SLOTS; ++i) {
        if (s_pending[i].occupied) { s_stats.dropped++; }
        s_pending[i].occupied = 0U;
    }
    s_stats.queued = 0U;
    s_alert_streak = s_backoff = 0U;
    s_pending_epoch = s_queue_epoch;
}
#endif

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
#if defined(USB_OBSERVER_LOG_MUX)
    memset(s_pending, 0, sizeof(s_pending));
    s_queue_epoch = s_pending_epoch = s_order = s_retry_ms = 0U;
    s_alert_streak = s_backoff = 0U;
#endif
}

void UsbObserver_OnSessionReset(void)
{
    s_dtr = 0U;
    s_session++;
#if defined(USB_OBSERVER_LOG_MUX)
    s_queue_epoch++;
#endif
}

void UsbObserver_OnControlLines(uint16_t lines)
{
    const uint8_t dtr = (uint8_t)((lines & 1U) != 0U);
#if defined(USB_OBSERVER_LOG_MUX)
    if (s_dtr != dtr) { s_queue_epoch++; }
#endif
    s_dtr = dtr;
}

void UsbObserver_DiscardRx(uint32_t length)
{
    s_stats.discarded_rx_bytes += length;
}

void UsbObserver_Task(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
#if defined(USB_OBSERVER_LOG_MUX)
    if (s_pending_epoch != s_queue_epoch || s_dtr == 0U || configured_class() == NULL) {
        clear_pending();
    }
#endif
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
#if defined(USB_OBSERVER_LOG_MUX)
    drive_queue();
#endif
}

static bool submit_now(const void *data, size_t length, uint32_t expected_epoch)
{
    USBD_CDC_HandleTypeDef *cdc;
    uint32_t primask, session;
    uint8_t status;
    if (data == NULL || length == 0U || length > sizeof(s_buffer)) {
        s_stats.dropped++;
        return false;
    }
#if !defined(USB_OBSERVER_LOG_MUX)
    (void)expected_epoch;
#endif
    primask = __get_PRIMASK();
    __disable_irq();
    cdc = configured_class();
#if defined(USB_OBSERVER_LOG_MUX)
    if (expected_epoch != s_queue_epoch) {
        s_stats.dropped++;
        __set_PRIMASK(primask);
        return false;
    }
#endif
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
    if (session != s_session || s_dtr == 0U || cdc == NULL || cdc->TxState != 0U
#if defined(USB_OBSERVER_LOG_MUX)
        || expected_epoch != s_queue_epoch
#endif
        ) {
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

#if defined(USB_OBSERVER_LOG_MUX)
static int oldest(unsigned begin, unsigned end)
{
    int chosen = -1;
    for (unsigned i = begin; i < end; ++i) {
        if (s_pending[i].occupied &&
            (chosen < 0 || (int32_t)(s_pending[i].order - s_pending[chosen].order) < 0)) {
            chosen = (int)i;
        }
    }
    return chosen;
}

static void drive_queue(void)
{
    int chosen, normal, alert;
    USBD_CDC_HandleTypeDef *cdc = configured_class();
    if (s_pending_epoch != s_queue_epoch || s_dtr == 0U || cdc == NULL ||
        s_active || cdc->TxState != 0U) { return; }
    if (s_backoff && HAL_GetTick() - s_retry_ms < USB_RETRY_MS) { return; }
    s_backoff = 0U;
    normal = oldest(0U, ALERT_BEGIN);
    alert = oldest(ALERT_BEGIN, PENDING_SLOTS);
    chosen = alert >= 0 && (normal < 0 || s_alert_streak < 2U) ? alert : normal;
    if (chosen < 0) { return; }
    if (submit_now(s_pending[chosen].bytes, s_pending[chosen].length, s_pending_epoch)) {
        s_pending[chosen].occupied = 0U;
        s_stats.queued--;
        if ((unsigned)chosen >= ALERT_BEGIN) {
            if (s_alert_streak < 2U) { s_alert_streak++; }
        } else { s_alert_streak = 0U; }
    } else {
        s_backoff = 1U;
        s_retry_ms = HAL_GetTick();
    }
}

static int topic_slot(const void *data, size_t length)
{
    static const char *prefixes[] = {"status ", "POST /api/device/report ", "voice_rx ", "vision_link ", "gimbal_link "};
    for (unsigned i = 0U; i < 5U; ++i) {
        const size_t size = strlen(prefixes[i]);
        if (length >= size && memcmp(data, prefixes[i], size) == 0) { return (int)i; }
    }
    return -1;
}
#endif

bool UsbObserver_WritePriority(const void *data, size_t length, bool alert)
{
#if defined(USB_OBSERVER_LOG_MUX)
    uint32_t primask, epoch;
    int chosen;
    if (data == NULL || length == 0U || length > USB_OBSERVER_FRAME_SIZE) {
        s_stats.dropped++;
        return false;
    }
    UsbObserver_Task();
    primask = __get_PRIMASK();
    __disable_irq();
    epoch = s_queue_epoch;
    if (s_dtr == 0U || configured_class() == NULL || s_pending_epoch != epoch) {
        s_stats.dropped++;
        __set_PRIMASK(primask);
        return false;
    }
    __set_PRIMASK(primask);
    chosen = alert ? -1 : topic_slot(data, length);
    if (chosen < 0) {
        const unsigned begin = alert ? ALERT_BEGIN : 5U;
        const unsigned end = alert ? PENDING_SLOTS : ALERT_BEGIN;
        for (unsigned i = begin; i < end; ++i) {
            if (!s_pending[i].occupied) { chosen = (int)i; break; }
        }
    }
    if (chosen < 0) { s_stats.queue_drops++; s_stats.dropped++; return false; }
    /* USB only owns s_buffer, never a pending slot; queued topics can be replaced. */
    memcpy(s_pending[chosen].bytes, data, length);
    primask = __get_PRIMASK();
    __disable_irq();
    if (epoch != s_queue_epoch || s_dtr == 0U) {
        s_stats.dropped++;
        __set_PRIMASK(primask);
        return false;
    }
    s_pending[chosen].length = (uint16_t)length;
    if (s_pending[chosen].occupied) { s_stats.coalesced++; }
    else {
        s_pending[chosen].occupied = 1U;
        s_pending[chosen].order = s_order++;
        s_stats.queued++;
    }
    __set_PRIMASK(primask);
    drive_queue();
    return true;
#else
    (void)alert;
    UsbObserver_Task();
    return submit_now(data, length, 0U);
#endif
}

bool UsbObserver_Write(const void *data, size_t length)
{
    return UsbObserver_WritePriority(data, length, false);
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
