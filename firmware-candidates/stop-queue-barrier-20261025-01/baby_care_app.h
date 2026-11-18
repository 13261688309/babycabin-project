#ifndef BABY_CARE_APP_H
#define BABY_CARE_APP_H

#include "baby_care_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BABY_CARE_MODE_IDLE = 0,
    BABY_CARE_MODE_SLEEP,
    BABY_CARE_MODE_SOOTHE,
    BABY_CARE_MODE_ALERT,
    BABY_CARE_MODE_FAULT
} BabyCare_ModeTypeDef;

#define BABY_CARE_FAULT_MOTOR 0x01U
#define BABY_CARE_FAULT_SERVO 0x02U

typedef struct
{
    float temperature_c;
    float humidity_percent;
    uint8_t sensor_valid;
    uint8_t sensor_error;
    uint8_t sensor_pin_level;
    uint32_t sensor_last_success_ms;
    uint32_t sensor_sample_count;
    uint32_t sensor_failures;
    uint32_t fault_flags;
    uint32_t rejected_commands;
    uint8_t safety_latched;
    uint8_t interlock_stop_ok;
    uint8_t interlock_armed;
    uint8_t interlock_ready;
    uint8_t interlock_enabled;
    uint8_t interlock_trip;
    uint8_t interlock_power_good;
    uint8_t interlock_power_pending;
    uint8_t latest_voice_command;
    uint32_t voice_stop_applied;
    uint32_t voice_stop_applied_ms;
    uint8_t esp32_ap_ip_valid;
    uint8_t esp32_sta_ip_valid;
    char esp32_ap_ip[16];
    char esp32_sta_ip[16];
    BabyCare_ModeTypeDef mode;
} BabyCare_StatusTypeDef;

void BabyCare_Init(void);
void BabyCare_Task(void);
void BabyCare_Stop(void);
uint8_t BabyCare_ClearSafetyLatch(void);
uint8_t BabyCare_RequestMode(BabyCare_ModeTypeDef mode);
void BabyCare_OnUartRxComplete(UART_HandleTypeDef *huart);
void BabyCare_OnUartTxComplete(UART_HandleTypeDef *huart);
void BabyCare_OnUartError(UART_HandleTypeDef *huart);
const BabyCare_StatusTypeDef *BabyCare_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif
