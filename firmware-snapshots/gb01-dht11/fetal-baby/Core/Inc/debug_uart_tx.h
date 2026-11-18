#ifndef DEBUG_UART_TX_H
#define DEBUG_UART_TX_H
#include "baby_care_hal.h"
#include <stdbool.h>
#include <stddef.h>
#define DEBUG_TX_FRAME_SIZE 1088U
#define DEBUG_TX_NORMAL_SLOTS 4U
#define DEBUG_TX_ALERT_SLOTS 2U
#define DEBUG_TX_TIMEOUT_MS 200U
#define DEBUG_TX_RETRY_MS 10U
typedef struct {
    uint32_t completed, dropped, start_errors, timeouts, abort_errors, queued;
} DebugTx_Stats;
/* Main-context producers; ISR callbacks only publish flags. Success means
 * queued, not delivered. USART1 must have exactly one transmission owner. */
bool DebugTx_Init(UART_HandleTypeDef *uart);
bool DebugTx_Write(const void *data, size_t length, bool alert);
void DebugTx_Task(void);
void DebugTx_OnComplete(UART_HandleTypeDef *uart);
void DebugTx_OnError(UART_HandleTypeDef *uart);
void DebugTx_GetStats(DebugTx_Stats *stats);
#endif
