#include "esp32_camera_link.h"

#include <stdio.h>
#include <string.h>

#define ESP32_CONFIG_ACK_NONE 0U
#define ESP32_CONFIG_ACK_OK 1U
#define ESP32_CONFIG_ACK_FAIL 2U
#define ESP32_CONFIG_ACK_RX_ERROR 3U
#define ESP32_REMOTE_FRAME_GAP_MS 250U

static void ESP32Cam_ConfigFail(ESP32Cam_HandleTypeDef *handle,
                               ESP32Cam_ConfigResultTypeDef result)
{
    handle->configure_ack_armed = 0U;
    handle->configure_ack = ESP32_CONFIG_ACK_NONE;
    handle->configure_waiting_ack = 0U;
    handle->configure_tx_pending = 0U;
    handle->configuring = 0U;
    handle->configure_failed = 1U;
    handle->configure_result = result;
}

static ESP32Cam_StatusTypeDef ESP32Cam_SendCommand(ESP32Cam_HandleTypeDef *handle,
                                                   const char *command)
{
    const size_t length = (command == NULL) ? 0U : strlen(command);
    if (handle == NULL || handle->huart == NULL || command == NULL) {
        return ESP32_CAM_ERROR_PARAM;
    }
    if (length == 0U || length > sizeof(handle->tx_buffer)) {
        return ESP32_CAM_ERROR_PARAM;
    }
    if (handle->tx_busy != 0U) {
        return ESP32_CAM_ERROR_BUSY;
    }

    memcpy(handle->tx_buffer, command, length);
    handle->tx_length = (uint16_t)length;
    handle->tx_complete = 0U;
    handle->tx_error = 0U;
    handle->tx_cancel = 0U;
    handle->tx_started_ms = HAL_GetTick();
    handle->tx_busy = 1U;
    if (HAL_UART_Transmit_IT(handle->huart, handle->tx_buffer, handle->tx_length) != HAL_OK) {
        handle->tx_busy = 0U;
        handle->tx_failures++;
        return ESP32_CAM_ERROR_HAL;
    }

    return ESP32_CAM_OK;
}

static void ESP32Cam_CopyIp(char *destination, const char *source)
{
    (void)strncpy(destination, source, ESP32_CAM_IP_STRING_SIZE - 1U);
    destination[ESP32_CAM_IP_STRING_SIZE - 1U] = '\0';
}

static uint8_t ESP32Cam_IsValidIp(const char *value)
{
    uint16_t octets[4];
    uint8_t index;
    for (index = 0U; index < 4U; index++) {
        uint16_t octet = 0U;
        uint8_t digits = 0U;
        const char *start = value;
        while (*value >= '0' && *value <= '9') {
            octet = (uint16_t)(octet * 10U + (uint16_t)(*value - '0'));
            if (++digits > 3U || octet > 255U) { return 0U; }
            value++;
        }
        if (digits == 0U || (digits > 1U && *start == '0')) { return 0U; }
        octets[index] = octet;
        if (index < 3U) {
            if (*value++ != '.') { return 0U; }
        } else if (*value != '\0') { return 0U; }
    }
    return octets[0] != 0U && octets[0] != 127U && octets[0] < 224U;
}

static void ESP32Cam_ParseIpLine(ESP32Cam_HandleTypeDef *handle,
                                 const char *line,
                                 const char *prefix,
                                 char *ip_buffer,
                                 uint8_t *valid_flag)
{
    const size_t prefix_len = strlen(prefix);
    const char *value;

    if (strncmp(line, prefix, prefix_len) != 0) {
        return;
    }
    value = line + prefix_len;

    if (!ESP32Cam_IsValidIp(value)) {
        ip_buffer[0] = '\0';
        *valid_flag = 0U;
    } else {
        ESP32Cam_CopyIp(ip_buffer, value);
        *valid_flag = 1U;
    }

    handle->status.updated = 1U;
    handle->status.last_update_tick = HAL_GetTick();
    if (valid_flag == &handle->status.ap_ip_valid) {
        handle->ap_last_update_ms = HAL_GetTick();
    } else {
        handle->sta_last_update_ms = HAL_GetTick();
    }
}

static void ESP32Cam_ParseLine(ESP32Cam_HandleTypeDef *handle)
{
    handle->line_buffer[handle->line_length] = '\0';

    if (handle->configure_ack_armed != 0U && handle->configure_ack == ESP32_CONFIG_ACK_NONE) {
        if (handle->line_length == 2U && memcmp(handle->line_buffer, "OK", 2U) == 0) {
            handle->configure_ack_received_ms = HAL_GetTick();
            __DMB();
            handle->configure_ack = ESP32_CONFIG_ACK_OK;
        } else if ((handle->line_length == 4U &&
                    (memcmp(handle->line_buffer, "fail", 4U) == 0 ||
                     memcmp(handle->line_buffer, "FAIL", 4U) == 0)) ||
                   (handle->line_length > 5U &&
                    (memcmp(handle->line_buffer, "fail,", 5U) == 0 ||
                     memcmp(handle->line_buffer, "FAIL,", 5U) == 0))) {
            handle->configure_ack_received_ms = HAL_GetTick();
            __DMB();
            handle->configure_ack = ESP32_CONFIG_ACK_FAIL;
        }
    }

    ESP32Cam_ParseIpLine(handle,
                         handle->line_buffer,
                         "ap_ip:",
                         handle->status.ap_ip,
                         &handle->status.ap_ip_valid);
    ESP32Cam_ParseIpLine(handle,
                         handle->line_buffer,
                         "sta_ip:",
                         handle->status.sta_ip,
                         &handle->status.sta_ip_valid);

    handle->line_length = 0U;
}

static void ESP32Cam_HandleRemoteByte(ESP32Cam_HandleTypeDef *handle, uint8_t byte)
{
    const uint32_t now = HAL_GetTick();
    /* IRQ bursts can arrive before the foreground task expires a stale prefix. */
    if (handle->remote_receiving != 0U &&
        (now - handle->remote_last_byte_ms) >= ESP32_REMOTE_FRAME_GAP_MS) {
        handle->remote_receiving = 0U;
        handle->remote_frame_length = 0U;
    }
    handle->remote_last_byte_ms = now;
    if (byte == '$') {
        handle->remote_receiving = 1U;
        handle->remote_frame_length = 0U;
        return;
    }

    if (handle->remote_receiving == 0U) {
        return;
    }

    if (byte == '#') {
        uint8_t next;
        handle->remote_receiving = 0U;
        if ((handle->remote_frame_length == 4U && memcmp(handle->remote_frame, "STOP", 4U) == 0) ||
            (handle->remote_frame_length == 7U && memcmp(handle->remote_frame, "7,0,0,0", 7U) == 0)) {
            handle->stop_pending = 1U;
            return;
        }
        next = (uint8_t)((handle->queue_head + 1U) % ESP32_CAM_QUEUE_SIZE);
        if (handle->remote_frame_length == 0U) { return; }
        if (handle->stop_pending != 0U || next == handle->queue_tail) {
            handle->dropped_frames++;
            return;
        }
        memcpy(handle->remote_queue[handle->queue_head], handle->remote_frame, handle->remote_frame_length);
        handle->remote_lengths[handle->queue_head] = handle->remote_frame_length;
        __DMB();
        handle->queue_head = next;
        return;
    }

    if (handle->remote_frame_length < ESP32_CAM_REMOTE_FRAME_SIZE) {
        handle->remote_frame[handle->remote_frame_length++] = byte;
    } else {
        handle->remote_receiving = 0U;
        handle->remote_frame_length = 0U;
    }
}

ESP32Cam_StatusTypeDef ESP32Cam_Init(ESP32Cam_HandleTypeDef *handle,
                                     UART_HandleTypeDef *huart)
{
    if (handle == NULL || huart == NULL) {
        return ESP32_CAM_ERROR_PARAM;
    }

    memset(handle, 0, sizeof(*handle));
    handle->huart = huart;
    return ESP32_CAM_OK;
}

void ESP32Cam_DiscardQueuedCommands(ESP32Cam_HandleTypeDef *handle)
{
    uint32_t primask;
    if (handle == NULL) { return; }
    primask = __get_PRIMASK();
    __disable_irq();
    handle->queue_tail = handle->queue_head;
    handle->remote_receiving = 0U;
    handle->remote_frame_ready = 0U;
    handle->remote_frame_length = 0U;
    /* Do not discard urgent STOP, AT replies or in-flight TX ownership. */
    __set_PRIMASK(primask);
}

static void ESP32Cam_ReceiveFailed(ESP32Cam_HandleTypeDef *handle)
{
    handle->restart_needed = 1U;
    ESP32Cam_DiscardQueuedCommands(handle);
    handle->line_length = 0U;
    handle->discard_line = 0U;
    if (handle->configure_ack_armed != 0U) {
        handle->configure_ack = ESP32_CONFIG_ACK_RX_ERROR;
    }
}

ESP32Cam_StatusTypeDef ESP32Cam_StartReceiveIT(ESP32Cam_HandleTypeDef *handle)
{
    uint32_t primask;
    HAL_StatusTypeDef status;
    if (handle == NULL || handle->huart == NULL) {
        return ESP32_CAM_ERROR_PARAM;
    }

    /* Publish recovery state before HAL's newly enabled IRQ can run. */
    primask = __get_PRIMASK();
    __disable_irq();
    status = HAL_UART_Receive_IT(handle->huart, &handle->rx_byte, 1U);
    if (status != HAL_OK) {
        ESP32Cam_ReceiveFailed(handle);
    } else {
        handle->restart_needed = 0U;
    }
    __set_PRIMASK(primask);
    return (status == HAL_OK) ? ESP32_CAM_OK : ESP32_CAM_ERROR_HAL;
}

ESP32Cam_StatusTypeDef ESP32Cam_ConfigureDefault(ESP32Cam_HandleTypeDef *handle)
{
    if (handle == NULL || handle->huart == NULL) { return ESP32_CAM_ERROR_PARAM; }
    if (handle->configuring != 0U || handle->tx_busy != 0U) { return ESP32_CAM_ERROR_BUSY; }
    handle->configure_step = 0U;
    handle->configuring = 1U;
    handle->configure_failed = 0U;
    handle->configure_retries = 0U;
    handle->configure_tx_pending = 0U;
    handle->configure_waiting_ack = 0U;
    handle->configure_ack_armed = 0U;
    handle->configure_ack = ESP32_CONFIG_ACK_NONE;
    handle->configure_result = ESP32_CAM_CONFIG_RUNNING;
    handle->configure_next_wait_ms = 0U;
    handle->configure_tick = HAL_GetTick();
    handle->configure_wait_ms = 2000U;
    return ESP32_CAM_OK;
}

static void ESP32Cam_ConfigureTask(ESP32Cam_HandleTypeDef *handle)
{
    char command[128];
    int written;
    uint32_t wait_ms = 300U;
    const uint32_t now = HAL_GetTick();
    uint32_t primask;
    ESP32Cam_StatusTypeDef send_status;
    if (handle->configuring == 0U || handle->configure_tx_pending != 0U ||
        handle->configure_waiting_ack != 0U ||
        (now - handle->configure_tick) < handle->configure_wait_ms) { return; }
    switch (handle->configure_step) {
    case 0: written = snprintf(command, sizeof(command), "wifi_mode:%s", ESP32_CAM_WIFI_MODE); wait_ms = 2000U; break;
    case 1: written = snprintf(command, sizeof(command), "sta_ssid:%s", ESP32_CAM_STA_SSID); break;
    case 2: written = snprintf(command, sizeof(command), "sta_pd:%s", ESP32_CAM_STA_PASSWORD); wait_ms = 2000U; break;
    case 3: written = snprintf(command, sizeof(command), "ap_ssid:%s", ESP32_CAM_AP_SSID); break;
    case 4: written = snprintf(command, sizeof(command), "ap_pd:%s", ESP32_CAM_AP_PASSWORD); wait_ms = 2000U; break;
    case 5: written = snprintf(command, sizeof(command), "ai_mode:%lu", (unsigned long)ESP32_CAM_AI_MODE); wait_ms = 2000U; break;
    default:
        handle->configuring = 0U;
        handle->configure_result = ESP32_CAM_CONFIG_ACKNOWLEDGED;
        (void)ESP32Cam_RequestApIp(handle);
        return;
    }
    if (written < 0 || (size_t)written >= sizeof(command)) {
        ESP32Cam_ConfigFail(handle, ESP32_CAM_CONFIG_INVALID);
        return;
    }
    if (handle->tx_busy != 0U) { return; }
    /* A generic OK has no command ID: never overlap a query/config transaction. */
    primask = __get_PRIMASK();
    __disable_irq();
    handle->line_length = 0U;
    handle->discard_line = 0U;
    handle->configure_ack = ESP32_CONFIG_ACK_NONE;
    handle->configure_ack_armed = 1U;
    __set_PRIMASK(primask);
    send_status = ESP32Cam_SendCommand(handle, command);
    if (send_status == ESP32_CAM_ERROR_BUSY) {
        handle->configure_ack_armed = 0U;
        handle->configure_tick = now;
        handle->configure_wait_ms = 10U;
        return;
    }
    if (send_status != ESP32_CAM_OK) {
        handle->configure_ack_armed = 0U;
        handle->configure_tick = now;
        handle->configure_wait_ms = 1000U;
        if (++handle->configure_retries >= 3U) {
            ESP32Cam_ConfigFail(handle, ESP32_CAM_CONFIG_TX_FAILED);
        }
        return;
    }
    handle->configure_tx_pending = 1U;
    handle->configure_ack_tick = handle->tx_started_ms;
    handle->configure_next_wait_ms = wait_ms;
}

ESP32Cam_StatusTypeDef ESP32Cam_RequestStaIp(ESP32Cam_HandleTypeDef *handle)
{
    if (handle != NULL && handle->configuring != 0U) { return ESP32_CAM_ERROR_BUSY; }
    return ESP32Cam_SendCommand(handle, "sta_ip");
}

ESP32Cam_StatusTypeDef ESP32Cam_RequestApIp(ESP32Cam_HandleTypeDef *handle)
{
    if (handle != NULL && handle->configuring != 0U) { return ESP32_CAM_ERROR_BUSY; }
    return ESP32Cam_SendCommand(handle, "ap_ip");
}

ESP32Cam_StatusTypeDef ESP32Cam_GetStatus(ESP32Cam_HandleTypeDef *handle,
                                          ESP32Cam_LinkStatusTypeDef *status)
{
    if (handle == NULL || status == NULL) {
        return ESP32_CAM_ERROR_PARAM;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *status = handle->status;
    handle->status.updated = 0U;
    __set_PRIMASK(primask);
    return ESP32_CAM_OK;
}

ESP32Cam_StatusTypeDef ESP32Cam_GetRemoteFrame(ESP32Cam_HandleTypeDef *handle,
                                               uint8_t *frame,
                                               uint16_t frame_size,
                                               uint16_t *len)
{
    uint16_t copy_len;
    uint32_t primask;

    if (handle == NULL || frame == NULL || len == NULL || frame_size == 0U) {
        return ESP32_CAM_ERROR_PARAM;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (handle->stop_pending != 0U && frame_size >= 4U) {
        memcpy(frame, "STOP", 4U);
        *len = 4U;
        handle->stop_pending = 0U;
        handle->queue_tail = handle->queue_head;
        __set_PRIMASK(primask);
        return ESP32_CAM_OK;
    }
    if (handle->queue_tail == handle->queue_head) {
        if (primask == 0U) {
            __enable_irq();
        }
        *len = 0U;
        return ESP32_CAM_OK;
    }

    copy_len = handle->remote_lengths[handle->queue_tail];
    if (copy_len > frame_size) {
        __set_PRIMASK(primask);
        *len = 0U;
        return ESP32_CAM_ERROR_PARAM;
    }

    (void)memcpy(frame, handle->remote_queue[handle->queue_tail], copy_len);
    handle->queue_tail = (uint8_t)((handle->queue_tail + 1U) % ESP32_CAM_QUEUE_SIZE);

    if (primask == 0U) {
        __enable_irq();
    }

    *len = copy_len;
    return ESP32_CAM_OK;
}

void ESP32Cam_OnUartRxComplete(ESP32Cam_HandleTypeDef *handle,
                               UART_HandleTypeDef *huart)
{
    uint8_t byte;

    if (handle == NULL || handle->huart == NULL || huart != handle->huart ||
        handle->restart_needed != 0U) {
        return;
    }

    byte = handle->rx_byte;

    ESP32Cam_HandleRemoteByte(handle, byte);
    if (handle->remote_receiving != 0U || byte == '$' || byte == '#') {
        (void)ESP32Cam_StartReceiveIT(handle);
        return;
    }

    if (byte == '\n') {
        if (handle->discard_line == 0U) { ESP32Cam_ParseLine(handle); }
        handle->line_length = 0U;
        handle->discard_line = 0U;
    } else if (byte == 0U) {
        handle->line_length = 0U;
        handle->discard_line = 1U;
    } else if (byte != '\r') {
        if (handle->discard_line != 0U) {
            /* Discard the entire overlong line, including any valid-looking suffix. */
        } else if (handle->line_length < (ESP32_CAM_LINE_BUFFER_SIZE - 1U)) {
            handle->line_buffer[handle->line_length++] = (char)byte;
        } else {
            handle->line_length = 0U;
            handle->discard_line = 1U;
        }
    }

    (void)ESP32Cam_StartReceiveIT(handle);
}

void ESP32Cam_OnUartError(ESP32Cam_HandleTypeDef *handle, UART_HandleTypeDef *huart)
{
    if (handle != NULL && huart != NULL && handle->huart == huart) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        ESP32Cam_ReceiveFailed(handle);
        if (handle->tx_busy != 0U) { handle->tx_error = 1U; }
        __set_PRIMASK(primask);
    }
}

void ESP32Cam_OnUartTxComplete(ESP32Cam_HandleTypeDef *handle, UART_HandleTypeDef *huart)
{
    if (handle != NULL && handle->huart == huart && handle->tx_busy != 0U) {
        handle->tx_complete = 1U;
    }
}

void ESP32Cam_Task(ESP32Cam_HandleTypeDef *handle)
{
    uint32_t primask;
    uint32_t now;
    uint8_t tx_completed;
    uint8_t tx_error;
    uint8_t tx_failed = 0U;
    uint8_t ack;
    uint32_t ack_received_ms;
    if (handle == NULL || handle->huart == NULL) { return; }
    if (handle->restart_needed != 0U && HAL_UART_AbortReceive(handle->huart) == HAL_OK) {
        handle->line_length = 0U;
        handle->discard_line = 0U;
        handle->remote_receiving = 0U;
        handle->remote_frame_length = 0U;
        (void)ESP32Cam_StartReceiveIT(handle);
    }
    now = HAL_GetTick();
    primask = __get_PRIMASK();
    __disable_irq();
    tx_completed = handle->tx_complete;
    tx_error = handle->tx_error;
    handle->tx_complete = 0U;
    handle->tx_error = 0U;
    __set_PRIMASK(primask);
    if (tx_completed != 0U && handle->tx_busy != 0U) {
        handle->tx_busy = 0U;
        handle->tx_cancel = 0U;
    } else {
        tx_completed = 0U;
        if (handle->tx_busy != 0U && tx_error != 0U) { handle->tx_cancel = 1U; }
        if (handle->tx_busy != 0U && handle->tx_cancel == 0U &&
            (now - handle->tx_started_ms) >= ESP32_CAM_TX_TIMEOUT_MS) {
            handle->tx_timeouts++;
            handle->tx_cancel = 1U;
        }
        if (handle->tx_busy != 0U && handle->tx_cancel != 0U &&
            HAL_UART_AbortTransmit(handle->huart) == HAL_OK) {
            handle->tx_busy = 0U;
            handle->tx_cancel = 0U;
            handle->tx_failures++;
            tx_failed = 1U;
        }
    }
    if (handle->configure_tx_pending != 0U && (tx_completed != 0U || tx_failed != 0U)) {
        handle->configure_tx_pending = 0U;
        handle->configure_tick = now;
        if (tx_completed != 0U) {
            handle->configure_waiting_ack = 1U;
        } else {
            /* Partial TX may already have changed settings. Do not replay it. */
            ESP32Cam_ConfigFail(handle, ESP32_CAM_CONFIG_TX_FAILED);
        }
    }

    if (handle->configure_waiting_ack != 0U) {
        primask = __get_PRIMASK();
        __disable_irq();
        ack = handle->configure_ack;
        ack_received_ms = handle->configure_ack_received_ms;
        if (ack != ESP32_CONFIG_ACK_NONE) {
            handle->configure_ack_armed = 0U;
            handle->configure_ack = ESP32_CONFIG_ACK_NONE;
        }
        __set_PRIMASK(primask);
        if ((ack == ESP32_CONFIG_ACK_OK || ack == ESP32_CONFIG_ACK_FAIL) &&
            (ack_received_ms - handle->configure_ack_tick) >= ESP32_CAM_CONFIG_ACK_TIMEOUT_MS) {
            handle->configure_ack_timeouts++;
            ESP32Cam_ConfigFail(handle, ESP32_CAM_CONFIG_ACK_TIMEOUT);
        } else if (ack == ESP32_CONFIG_ACK_OK) {
            handle->configure_waiting_ack = 0U;
            handle->configure_retries = 0U;
            handle->configure_step++;
            handle->configure_tick = now;
            handle->configure_wait_ms = handle->configure_next_wait_ms;
        } else if (ack == ESP32_CONFIG_ACK_FAIL) {
            handle->configure_rejections++;
            ESP32Cam_ConfigFail(handle, ESP32_CAM_CONFIG_REJECTED);
        } else if (ack == ESP32_CONFIG_ACK_RX_ERROR) {
            ESP32Cam_ConfigFail(handle, ESP32_CAM_CONFIG_RX_FAILED);
        } else if ((now - handle->configure_ack_tick) >= ESP32_CAM_CONFIG_ACK_TIMEOUT_MS) {
            handle->configure_ack_timeouts++;
            ESP32Cam_ConfigFail(handle, ESP32_CAM_CONFIG_ACK_TIMEOUT);
        }
    }

    primask = __get_PRIMASK();
    __disable_irq();
    /* RX IRQs may have published a newer tick since the TX/ACK snapshot. */
    now = HAL_GetTick();
    if (handle->status.ap_ip_valid != 0U && (now - handle->ap_last_update_ms) >= 15000U) {
        handle->status.ap_ip_valid = 0U;
        handle->status.ap_ip[0] = '\0';
        handle->status.updated = 1U;
    }
    if (handle->status.sta_ip_valid != 0U && (now - handle->sta_last_update_ms) >= 15000U) {
        handle->status.sta_ip_valid = 0U;
        handle->status.sta_ip[0] = '\0';
        handle->status.updated = 1U;
    }
    if (handle->remote_receiving != 0U && (now - handle->remote_last_byte_ms) >= ESP32_REMOTE_FRAME_GAP_MS) {
        handle->remote_receiving = 0U;
        handle->remote_frame_length = 0U;
    }
    __set_PRIMASK(primask);
    ESP32Cam_ConfigureTask(handle);
}
