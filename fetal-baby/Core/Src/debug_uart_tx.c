#include "debug_uart_tx.h"
#if defined(BABY_USB_OBSERVER)
#include "usb_observer_tx.h"
#if USB_OBSERVER_FRAME_SIZE < DEBUG_TX_FRAME_SIZE
#error "USB log frame capacity must cover UART log frames"
#endif
#endif
#include <string.h>
#define SLOT_COUNT (DEBUG_TX_NORMAL_SLOTS + DEBUG_TX_ALERT_SLOTS)
#define ACTIVE_NONE (-1)
#define ACTIVE_RESYNC (-2)
typedef struct {
    uint8_t bytes[DEBUG_TX_FRAME_SIZE];
    uint16_t length;
    uint8_t occupied, retries;
    uint32_t order;
} tx_slot_t;
static tx_slot_t s_slots[SLOT_COUNT];
static UART_HandleTypeDef *s_uart;
static volatile int s_active = ACTIVE_NONE;
static volatile uint8_t s_complete, s_error;
static uint8_t s_cancel, s_resync, s_backoff;
static uint32_t s_order, s_started_ms, s_retry_ms;
static DebugTx_Stats s_stats;
static uint8_t s_newline[] = "\r\n";

bool DebugTx_Init(UART_HandleTypeDef *uart)
{
    uint32_t primask;
    if (uart == NULL) { return false; }
    if (s_uart != NULL && s_active != ACTIVE_NONE && HAL_UART_AbortTransmit(s_uart) != HAL_OK) {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    s_uart = uart;
    s_active = ACTIVE_NONE;
    s_complete = s_error = 0U;
    __set_PRIMASK(primask);
    memset(s_slots, 0, sizeof(s_slots));
    memset(&s_stats, 0, sizeof(s_stats));
    s_cancel = s_resync = s_backoff = 0U;
    s_order = 0U;
    return true;
}

bool DebugTx_Write(const void *data, size_t length, bool alert)
{
    const unsigned begin = alert ? DEBUG_TX_NORMAL_SLOTS : 0U;
    const unsigned end = alert ? SLOT_COUNT : DEBUG_TX_NORMAL_SLOTS;
    unsigned index;
    if (s_uart == NULL || data == NULL || length == 0U || length > DEBUG_TX_FRAME_SIZE) {
        s_stats.dropped++;
        return false;
    }
#if defined(BABY_USB_OBSERVER)
    (void)UsbObserver_Write(data, length);
#endif
    for (index = begin; index < end; index++) {
        tx_slot_t *slot = &s_slots[index];
        if (slot->occupied == 0U) {
            memcpy(slot->bytes, data, length);
            slot->length = (uint16_t)length;
            slot->order = s_order++;
            slot->retries = 0U;
            slot->occupied = 1U;
            s_stats.queued++;
            return true;
        }
    }
    s_stats.dropped++;
    return false;
}

static void release_active(uint8_t delivered)
{
    if (s_active >= 0) {
        s_slots[s_active].occupied = 0U;
        s_stats.queued--;
        if (delivered != 0U) { s_stats.completed++; }
        else { s_stats.dropped++; }
    } else if (delivered != 0U) { s_resync = 0U; }
    s_active = ACTIVE_NONE;
    s_cancel = 0U;
}

static int oldest_slot(unsigned begin, unsigned end)
{
    int chosen = ACTIVE_NONE;
    unsigned index;
    for (index = begin; index < end; index++) {
        if (s_slots[index].occupied != 0U &&
            (chosen < 0 || (int32_t)(s_slots[index].order - s_slots[chosen].order) < 0)) {
            chosen = (int)index;
        }
    }
    return chosen;
}

void DebugTx_Task(void)
{
    uint32_t primask;
    uint8_t complete, error;
    uint8_t *bytes;
    uint16_t length;
    HAL_StatusTypeDef result;
    const uint32_t now = HAL_GetTick();
    if (s_uart == NULL) { return; }
    primask = __get_PRIMASK();
    __disable_irq();
    complete = s_complete;
    error = s_error;
    s_complete = s_error = 0U;
    __set_PRIMASK(primask);
    if (complete != 0U && s_active != ACTIVE_NONE) {
        release_active(1U);
    } else if (s_active != ACTIVE_NONE && error != 0U) { s_cancel = 1U; }
    if (s_active != ACTIVE_NONE && s_cancel == 0U && (now - s_started_ms) >= DEBUG_TX_TIMEOUT_MS) {
        s_stats.timeouts++;
        s_cancel = 1U;
    }
    if (s_backoff != 0U && (now - s_retry_ms) < DEBUG_TX_RETRY_MS) { return; }
    s_backoff = 0U;
    if (s_cancel != 0U) {
        if (HAL_UART_AbortTransmit(s_uart) != HAL_OK) {
            s_stats.abort_errors++;
            s_backoff = 1U;
            s_retry_ms = now;
            return; /* The HAL may still reference this buffer. Keep it reserved. */
        }
        primask = __get_PRIMASK();
        __disable_irq();
        s_complete = s_error = 0U;
        __set_PRIMASK(primask);
        release_active(0U);
        s_resync = 1U;
    }
    if (s_active != ACTIVE_NONE) { return; }
    if (s_resync != 0U) {
        s_active = ACTIVE_RESYNC;
        bytes = s_newline;
        length = 2U;
    } else {
        int chosen = oldest_slot(DEBUG_TX_NORMAL_SLOTS, SLOT_COUNT);
        if (chosen < 0) { chosen = oldest_slot(0U, DEBUG_TX_NORMAL_SLOTS); }
        if (chosen < 0) { return; }
        s_active = chosen;
        bytes = s_slots[chosen].bytes;
        length = s_slots[chosen].length;
    }
    s_started_ms = now;
    result = HAL_UART_Transmit_IT(s_uart, bytes, length);
    if (result != HAL_OK) {
        s_stats.start_errors++;
        if (s_active >= 0 && ++s_slots[s_active].retries >= 3U) { release_active(0U); }
        else { s_active = ACTIVE_NONE; }
        s_backoff = 1U;
        s_retry_ms = now;
    }
}

void DebugTx_OnComplete(UART_HandleTypeDef *uart)
{
    if (uart == s_uart && s_active != ACTIVE_NONE) { s_complete = 1U; }
}
void DebugTx_OnError(UART_HandleTypeDef *uart)
{
    if (uart == s_uart && s_active != ACTIVE_NONE) { s_error = 1U; }
}
void DebugTx_GetStats(DebugTx_Stats *stats)
{
    if (stats != NULL) { *stats = s_stats; }
}
