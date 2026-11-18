#ifndef ESP32_CAMERA_LINK_H
#define ESP32_CAMERA_LINK_H

#include "baby_care_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP32_CAM_IP_STRING_SIZE      16U
#define ESP32_CAM_LINE_BUFFER_SIZE    64U
#define ESP32_CAM_REMOTE_FRAME_SIZE   16U
#define ESP32_CAM_QUEUE_SIZE          8U
#define ESP32_CAM_TX_BUFFER_SIZE      128U
#define ESP32_CAM_TX_TIMEOUT_MS       200U
#define ESP32_CAM_CONFIG_ACK_TIMEOUT_MS 3000U

#define ESP32_CAM_WIFI_MODE           "2"
#define ESP32_CAM_STA_SSID            "Yahboom2"
#define ESP32_CAM_STA_PASSWORD        "yahboom890729"
#define ESP32_CAM_AP_SSID             "ESP32_WIFI_TEST"
#define ESP32_CAM_AP_PASSWORD         ""
#define ESP32_CAM_AI_MODE             0U

typedef enum
{
    ESP32_CAM_OK = 0,
    ESP32_CAM_ERROR_PARAM,
    ESP32_CAM_ERROR_HAL,
    ESP32_CAM_ERROR_BUSY
} ESP32Cam_StatusTypeDef;

typedef enum
{
    ESP32_CAM_CONFIG_IDLE = 0,
    ESP32_CAM_CONFIG_RUNNING,
    ESP32_CAM_CONFIG_ACKNOWLEDGED,
    ESP32_CAM_CONFIG_TX_FAILED,
    ESP32_CAM_CONFIG_ACK_TIMEOUT,
    ESP32_CAM_CONFIG_REJECTED,
    ESP32_CAM_CONFIG_INVALID,
    ESP32_CAM_CONFIG_RX_FAILED
} ESP32Cam_ConfigResultTypeDef;

typedef struct
{
    char ap_ip[ESP32_CAM_IP_STRING_SIZE];
    char sta_ip[ESP32_CAM_IP_STRING_SIZE];
    uint8_t ap_ip_valid;
    uint8_t sta_ip_valid;
    uint8_t updated;
    uint32_t last_update_tick;
} ESP32Cam_LinkStatusTypeDef;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t rx_byte;
    char line_buffer[ESP32_CAM_LINE_BUFFER_SIZE];
    uint16_t line_length;
    uint8_t remote_receiving;
    uint8_t remote_frame_ready;
    uint16_t remote_frame_length;
    uint8_t remote_frame[ESP32_CAM_REMOTE_FRAME_SIZE];
    uint8_t remote_queue[ESP32_CAM_QUEUE_SIZE][ESP32_CAM_REMOTE_FRAME_SIZE];
    uint16_t remote_lengths[ESP32_CAM_QUEUE_SIZE];
    volatile uint8_t queue_head;
    volatile uint8_t queue_tail;
    volatile uint8_t stop_pending;
    volatile uint8_t restart_needed;
    volatile uint32_t dropped_frames;
    uint8_t configuring;
    uint8_t configure_step;
    uint8_t configure_retries;
    uint8_t configure_failed;
    uint8_t configure_tx_pending;
    uint8_t configure_waiting_ack;
    volatile uint8_t configure_ack_armed;
    volatile uint8_t configure_ack;
    volatile uint32_t configure_ack_received_ms;
    uint32_t configure_ack_tick;
    uint32_t configure_ack_timeouts;
    uint32_t configure_rejections;
    ESP32Cam_ConfigResultTypeDef configure_result;
    uint32_t configure_tick;
    uint32_t configure_wait_ms;
    uint32_t configure_next_wait_ms;
    uint32_t ap_last_update_ms;
    uint32_t sta_last_update_ms;
    uint32_t remote_last_byte_ms;
    uint8_t discard_line;
    uint8_t tx_buffer[ESP32_CAM_TX_BUFFER_SIZE];
    uint16_t tx_length;
    volatile uint8_t tx_busy;
    volatile uint8_t tx_complete;
    volatile uint8_t tx_error;
    uint8_t tx_cancel;
    uint32_t tx_started_ms;
    uint32_t tx_failures;
    uint32_t tx_timeouts;
    ESP32Cam_LinkStatusTypeDef status;
} ESP32Cam_HandleTypeDef;

ESP32Cam_StatusTypeDef ESP32Cam_Init(ESP32Cam_HandleTypeDef *handle,
                                     UART_HandleTypeDef *huart);
ESP32Cam_StatusTypeDef ESP32Cam_StartReceiveIT(ESP32Cam_HandleTypeDef *handle);
ESP32Cam_StatusTypeDef ESP32Cam_ConfigureDefault(ESP32Cam_HandleTypeDef *handle);
ESP32Cam_StatusTypeDef ESP32Cam_RequestStaIp(ESP32Cam_HandleTypeDef *handle);
ESP32Cam_StatusTypeDef ESP32Cam_RequestApIp(ESP32Cam_HandleTypeDef *handle);
ESP32Cam_StatusTypeDef ESP32Cam_GetStatus(ESP32Cam_HandleTypeDef *handle,
                                          ESP32Cam_LinkStatusTypeDef *status);
ESP32Cam_StatusTypeDef ESP32Cam_GetRemoteFrame(ESP32Cam_HandleTypeDef *handle,
                                               uint8_t *frame,
                                               uint16_t frame_size,
                                               uint16_t *len);
void ESP32Cam_OnUartRxComplete(ESP32Cam_HandleTypeDef *handle,
                               UART_HandleTypeDef *huart);
void ESP32Cam_OnUartError(ESP32Cam_HandleTypeDef *handle, UART_HandleTypeDef *huart);
void ESP32Cam_OnUartTxComplete(ESP32Cam_HandleTypeDef *handle, UART_HandleTypeDef *huart);
void ESP32Cam_Task(ESP32Cam_HandleTypeDef *handle);
void ESP32Cam_DiscardQueuedCommands(ESP32Cam_HandleTypeDef *handle);

#ifdef __cplusplus
}
#endif

#endif
