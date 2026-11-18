#include "voice_interaction.h"
#include <string.h>

#ifndef VOICE_DEFAULT_TIMEOUT_MS
#define VOICE_DEFAULT_TIMEOUT_MS 100U
#endif

static uint8_t voice_i2c_hal_addr(void)
{
    return (uint8_t)(VOICE_I2C_ADDR_7BIT << 1U);
}

static uint8_t voice_is_valid_uart_frame(const uint8_t *frame)
{
    return (frame != NULL &&
            frame[0] == VOICE_UART_HEAD_0 &&
            frame[1] == VOICE_UART_HEAD_1 &&
            frame[4] == VOICE_UART_TAIL);
}

static void voice_retain_frame_prefix(Voice_HandleTypeDef *hvoice)
{
    /* A rejected candidate may already contain the start of the next frame. */
    for (uint8_t start = 1U; start < VOICE_UART_FRAME_LEN; start++) {
        if (hvoice->rx_buf[start] == VOICE_UART_HEAD_0 &&
            (start == VOICE_UART_FRAME_LEN - 1U || hvoice->rx_buf[start + 1U] == VOICE_UART_HEAD_1)) {
            hvoice->rx_index = (uint8_t)(VOICE_UART_FRAME_LEN - start);
            memmove(hvoice->rx_buf, &hvoice->rx_buf[start], hvoice->rx_index);
            return;
        }
    }
}

static void voice_store_frame(Voice_HandleTypeDef *hvoice, const uint8_t *raw)
{
    Voice_FrameTypeDef frame;
    uint8_t next = (uint8_t)((hvoice->queue_head + 1U) % VOICE_QUEUE_SIZE);
    memcpy(frame.raw, raw, VOICE_UART_FRAME_LEN);
    frame.type = raw[2];
    frame.id = raw[3];
    /* Count parsed frames even if the queue coalesces or drops their delivery. */
    hvoice->rx_frames++;
    hvoice->rx_last_frame_ms = hvoice->rx_last_byte_ms;
    if (frame.type == VOICE_UART_CMD_TYPE) {
        if (frame.id == VOICE_CMD_STOP_1 || frame.id == VOICE_CMD_STOP_2) {
            hvoice->rx_stop_frames++;
            hvoice->rx_last_stop_ms = hvoice->rx_last_byte_ms;
        } else if (frame.id == VOICE_CMD_ALARM) {
            hvoice->rx_alarm_frames++;
        }
    }
    hvoice->latest_frame = frame;
    if (frame.type == VOICE_UART_CMD_TYPE &&
        (frame.id == VOICE_CMD_STOP_1 || frame.id == VOICE_CMD_STOP_2 || frame.id == VOICE_CMD_ALARM)) {
        if (hvoice->urgent_pending == 0U || frame.id == VOICE_CMD_ALARM) {
            hvoice->urgent_frame = frame;
        }
        hvoice->urgent_pending = 1U;
    } else if (hvoice->urgent_pending != 0U || next == hvoice->queue_tail) {
        hvoice->dropped_frames++;
    } else {
        hvoice->queue[hvoice->queue_head] = frame;
        __DMB();
        hvoice->queue_head = next;
    }
}

void Voice_Init(Voice_HandleTypeDef *hvoice,
                UART_HandleTypeDef *huart,
                I2C_HandleTypeDef *hi2c)
{
    if (hvoice == NULL) {
        return;
    }

    memset(hvoice, 0, sizeof(*hvoice));
    hvoice->huart = huart;
    hvoice->hi2c = hi2c;
    hvoice->timeout_ms = VOICE_DEFAULT_TIMEOUT_MS;
}

void Voice_SetCallback(Voice_HandleTypeDef *hvoice,
                       Voice_FrameCallback callback,
                       void *user_context)
{
    if (hvoice == NULL) {
        return;
    }

    hvoice->callback = callback;
    hvoice->callback_context = user_context;
}

Voice_StatusTypeDef Voice_UART_StartReceiveIT(Voice_HandleTypeDef *hvoice)
{
    uint32_t primask;
    HAL_StatusTypeDef status;
    if (hvoice == NULL || hvoice->huart == NULL) {
        return VOICE_ERROR_PARAM;
    }

    /* Publish recovery state before HAL's newly enabled IRQ can run. */
    primask = __get_PRIMASK();
    __disable_irq();
    status = HAL_UART_Receive_IT(hvoice->huart, &hvoice->rx_byte, 1U);
    if (status != HAL_OK) {
        hvoice->restart_needed = 1U;
        Voice_UART_DiscardQueuedCommands(hvoice);
    } else {
        hvoice->restart_needed = 0U;
    }
    __set_PRIMASK(primask);
    return (status == HAL_OK) ? VOICE_OK : VOICE_ERROR_HAL;
}

Voice_StatusTypeDef Voice_UART_RxCpltCallback(Voice_HandleTypeDef *hvoice, UART_HandleTypeDef *huart)
{
    Voice_StatusTypeDef status;

    if (hvoice == NULL || hvoice->huart == NULL || huart == NULL) {
        return VOICE_ERROR_PARAM;
    }

    if (huart != hvoice->huart) {
        return VOICE_ERROR_PARAM;
    }
    if (hvoice->restart_needed != 0U) {
        return VOICE_ERROR_HAL;
    }

    status = Voice_UART_ParseByte(hvoice, hvoice->rx_byte, NULL);
    (void)Voice_UART_StartReceiveIT(hvoice);

    return status;
}

Voice_StatusTypeDef Voice_UART_ParseByte(Voice_HandleTypeDef *hvoice, uint8_t byte, Voice_FrameTypeDef *frame)
{
    uint32_t now;
    if (hvoice == NULL) {
        return VOICE_ERROR_PARAM;
    }
    now = HAL_GetTick();
    if (hvoice->rx_index != 0U && (now - hvoice->rx_last_byte_ms) >= VOICE_FRAME_GAP_MS) {
        hvoice->rx_index = 0U;
    }
    hvoice->rx_last_byte_ms = now;

    if (hvoice->rx_index == 0U && byte != VOICE_UART_HEAD_0) {
        return VOICE_ERROR_NO_FRAME;
    }

    if (hvoice->rx_index == 1U && byte != VOICE_UART_HEAD_1) {
        if (byte == VOICE_UART_HEAD_0) {
            hvoice->rx_buf[0] = VOICE_UART_HEAD_0;
            hvoice->rx_index = 1U;
        } else {
            hvoice->rx_index = 0U;
        }
        return VOICE_ERROR_NO_FRAME;
    }

    hvoice->rx_buf[hvoice->rx_index++] = byte;

    if (hvoice->rx_index < VOICE_UART_FRAME_LEN) {
        return VOICE_ERROR_NO_FRAME;
    }

    hvoice->rx_index = 0U;

    if (!voice_is_valid_uart_frame(hvoice->rx_buf)) {
        voice_retain_frame_prefix(hvoice);
        return VOICE_ERROR_FRAME;
    }

    voice_store_frame(hvoice, hvoice->rx_buf);

    if (frame != NULL) {
        *frame = hvoice->latest_frame;
    }

    return VOICE_OK;
}

Voice_StatusTypeDef Voice_UART_GetLatestFrame(Voice_HandleTypeDef *hvoice, Voice_FrameTypeDef *frame)
{
    uint32_t primask;
    if (hvoice == NULL || frame == NULL) {
        return VOICE_ERROR_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (hvoice->urgent_pending != 0U) {
        *frame = hvoice->urgent_frame;
        hvoice->urgent_pending = 0U;
        hvoice->queue_tail = hvoice->queue_head;
    } else if (hvoice->queue_tail != hvoice->queue_head) {
        *frame = hvoice->queue[hvoice->queue_tail];
        hvoice->queue_tail = (uint8_t)((hvoice->queue_tail + 1U) % VOICE_QUEUE_SIZE);
    } else {
        __set_PRIMASK(primask);
        return VOICE_ERROR_NO_FRAME;
    }
    __set_PRIMASK(primask);
    if (hvoice->callback != NULL) {
        hvoice->callback(frame, hvoice->callback_context);
    }
    return VOICE_OK;
}

void Voice_UART_DiscardQueuedCommands(Voice_HandleTypeDef *hvoice)
{
    uint32_t primask;
    if (hvoice == NULL) { return; }
    primask = __get_PRIMASK();
    __disable_irq();
    hvoice->queue_tail = hvoice->queue_head;
    hvoice->rx_index = 0U;
    hvoice->has_frame = 0U;
    /* Urgent STOP/ALARM remains pending across a software reset. */
    __set_PRIMASK(primask);
}

Voice_StatusTypeDef Voice_UART_GetRxStats(const Voice_HandleTypeDef *hvoice,
                                         Voice_RxStatsTypeDef *stats)
{
    uint32_t primask;
    if (hvoice == NULL || stats == NULL) { return VOICE_ERROR_PARAM; }
    primask = __get_PRIMASK();
    __disable_irq();
    stats->frames = hvoice->rx_frames;
    stats->stop_frames = hvoice->rx_stop_frames;
    stats->alarm_frames = hvoice->rx_alarm_frames;
    stats->last_frame_ms = hvoice->rx_last_frame_ms;
    stats->last_stop_ms = hvoice->rx_last_stop_ms;
    stats->uart_errors = hvoice->rx_uart_errors;
    stats->dropped_frames = hvoice->dropped_frames;
    __set_PRIMASK(primask);
    return VOICE_OK;
}

void Voice_UART_OnError(Voice_HandleTypeDef *hvoice, UART_HandleTypeDef *huart)
{
    if (hvoice != NULL && huart != NULL && hvoice->huart == huart) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        hvoice->rx_uart_errors++;
        hvoice->restart_needed = 1U;
        Voice_UART_DiscardQueuedCommands(hvoice);
        if (hvoice->tx_busy != 0U) { hvoice->tx_error = 1U; }
        __set_PRIMASK(primask);
    }
}

void Voice_UART_OnTxComplete(Voice_HandleTypeDef *hvoice, UART_HandleTypeDef *huart)
{
    if (hvoice != NULL && hvoice->huart == huart && hvoice->tx_busy != 0U) {
        hvoice->tx_complete = 1U;
    }
}

void Voice_UART_Task(Voice_HandleTypeDef *hvoice)
{
    uint8_t complete;
    uint8_t error;
    uint32_t primask;
    const uint32_t now = HAL_GetTick();

    if (hvoice != NULL && hvoice->huart != NULL && hvoice->restart_needed != 0U &&
        HAL_UART_AbortReceive(hvoice->huart) == HAL_OK) {
        hvoice->rx_index = 0U;
        (void)Voice_UART_StartReceiveIT(hvoice);
    }
    if (hvoice == NULL || hvoice->huart == NULL) { return; }

    primask = __get_PRIMASK();
    __disable_irq();
    complete = hvoice->tx_complete;
    error = hvoice->tx_error;
    hvoice->tx_complete = 0U;
    hvoice->tx_error = 0U;
    __set_PRIMASK(primask);

    if (complete != 0U && hvoice->tx_busy != 0U) {
        hvoice->tx_busy = 0U;
        hvoice->tx_cancel = 0U;
        return;
    }
    if (hvoice->tx_busy != 0U && error != 0U) { hvoice->tx_cancel = 1U; }
    if (hvoice->tx_busy != 0U && hvoice->tx_cancel == 0U &&
        (now - hvoice->tx_started_ms) >= VOICE_TX_TIMEOUT_MS) {
        hvoice->tx_timeouts++;
        hvoice->tx_cancel = 1U;
    }
    if (hvoice->tx_busy != 0U && hvoice->tx_cancel != 0U &&
        HAL_UART_AbortTransmit(hvoice->huart) == HAL_OK) {
        hvoice->tx_busy = 0U;
        hvoice->tx_cancel = 0U;
        hvoice->tx_failures++;
    }
}

Voice_StatusTypeDef Voice_UART_SendFrame(Voice_HandleTypeDef *hvoice, uint8_t type, uint8_t id)
{
    if (hvoice == NULL || hvoice->huart == NULL) {
        return VOICE_ERROR_PARAM;
    }
    if (hvoice->tx_busy != 0U) {
        return VOICE_ERROR_BUSY;
    }

    hvoice->tx_frame[0] = VOICE_UART_HEAD_0;
    hvoice->tx_frame[1] = VOICE_UART_HEAD_1;
    hvoice->tx_frame[2] = type;
    hvoice->tx_frame[3] = id;
    hvoice->tx_frame[4] = VOICE_UART_TAIL;
    hvoice->tx_complete = 0U;
    hvoice->tx_error = 0U;
    hvoice->tx_cancel = 0U;
    hvoice->tx_started_ms = HAL_GetTick();
    hvoice->tx_busy = 1U;
    if (HAL_UART_Transmit_IT(hvoice->huart, hvoice->tx_frame, VOICE_UART_FRAME_LEN) != HAL_OK) {
        hvoice->tx_busy = 0U;
        hvoice->tx_failures++;
        return VOICE_ERROR_HAL;
    }

    return VOICE_OK;
}

Voice_StatusTypeDef Voice_UART_PlayBroadcast(Voice_HandleTypeDef *hvoice, uint8_t broadcast_id)
{
    return Voice_UART_SendFrame(hvoice, VOICE_UART_BROADCAST_TYPE, broadcast_id);
}

Voice_StatusTypeDef Voice_I2C_PlayBroadcast(Voice_HandleTypeDef *hvoice, uint8_t broadcast_id)
{
    if (hvoice == NULL || hvoice->hi2c == NULL) {
        return VOICE_ERROR_PARAM;
    }

    if (HAL_I2C_Mem_Write(hvoice->hi2c,
                          voice_i2c_hal_addr(),
                          VOICE_I2C_WRITE_REGISTER,
                          I2C_MEMADD_SIZE_8BIT,
                          &broadcast_id,
                          1U,
                          hvoice->timeout_ms) != HAL_OK) {
        return VOICE_ERROR_HAL;
    }

    return VOICE_OK;
}

Voice_StatusTypeDef Voice_I2C_ReadCommand(Voice_HandleTypeDef *hvoice, uint8_t *command_id)
{
    if (hvoice == NULL || hvoice->hi2c == NULL || command_id == NULL) {
        return VOICE_ERROR_PARAM;
    }

    if (HAL_I2C_Mem_Read(hvoice->hi2c,
                         voice_i2c_hal_addr(),
                         VOICE_I2C_READ_REGISTER,
                         I2C_MEMADD_SIZE_8BIT,
                         command_id,
                         1U,
                         hvoice->timeout_ms) != HAL_OK) {
        return VOICE_ERROR_HAL;
    }

    return VOICE_OK;
}

Voice_EventTypeDef Voice_GetEventType(const Voice_FrameTypeDef *frame)
{
    if (frame == NULL) {
        return VOICE_EVENT_NONE;
    }

    if (frame->type == VOICE_UART_CMD_TYPE) {
        return VOICE_EVENT_COMMAND;
    }

    if (frame->type == VOICE_UART_BROADCAST_TYPE) {
        return VOICE_EVENT_BROADCAST;
    }

    if (frame->type >= VOICE_SYS_WELCOME && frame->type <= VOICE_SYS_REPORT_OFF) {
        return VOICE_EVENT_SYSTEM;
    }

    return VOICE_EVENT_NONE;
}
