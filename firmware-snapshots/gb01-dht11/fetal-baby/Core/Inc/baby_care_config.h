#ifndef BABY_CARE_CONFIG_H
#define BABY_CARE_CONFIG_H

#include "baby_care_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Integration switches.
 * Set a module to 0 if the matching peripheral is not enabled in CubeMX yet.
 */
#define BABY_CARE_USE_DHT11          0
#define BABY_CARE_USE_DC_MOTOR       0
#define BABY_CARE_USE_SERVO_GIMBAL   1
#define BABY_CARE_USE_VOICE_UART     0
#define BABY_CARE_USE_DEBUG_UART     1
#define BABY_CARE_USE_ESP32_CAMERA   0

/* Preserve the module's existing network settings during normal bring-up. */
#ifndef BABY_CARE_ESP32_CONFIGURE_ON_BOOT
#define BABY_CARE_ESP32_CONFIGURE_ON_BOOT 0
#endif
#if BABY_CARE_ESP32_CONFIGURE_ON_BOOT != 0 && BABY_CARE_ESP32_CONFIGURE_ON_BOOT != 1
#error "ESP32 boot configuration must be 0 or 1"
#endif

/*
 * Default demo hardware mapping.
 * Physical board revision and connector orientation still require validation.
 */
#define BABY_CARE_DHT11_PORT         GPIOB
#define BABY_CARE_DHT11_PIN          GPIO_PIN_0

#define BABY_CARE_MOTOR_TIM          htim2
#define BABY_CARE_MOTOR_CHANNEL      TIM_CHANNEL_3
#define BABY_CARE_MOTOR_IN1_PORT     GPIOA
#define BABY_CARE_MOTOR_IN1_PIN      GPIO_PIN_4
#define BABY_CARE_MOTOR_IN2_PORT     GPIOA
#define BABY_CARE_MOTOR_IN2_PIN      GPIO_PIN_5

#define BABY_CARE_GIMBAL_PAN_TIM     htim3
#define BABY_CARE_GIMBAL_PAN_CH      TIM_CHANNEL_1
#define BABY_CARE_GIMBAL_TILT_TIM    htim3
#define BABY_CARE_GIMBAL_TILT_CH     TIM_CHANNEL_2

#define BABY_CARE_VOICE_UART         huart2
#define BABY_CARE_DEBUG_UART         huart1
#define BABY_CARE_ESP32_UART         huart3

/* Expansion-board draft: unconnected inputs fail closed. Verify board headers. */
#if defined(STM32F407xx)
/* Keep the LXB board's TF-card pins PC8..PC11 out of the safety interface. */
#define BABY_CARE_INTERLOCK_PORT       GPIOF
#define BABY_CARE_INTERLOCK_STOP_PIN   GPIO_PIN_0
#define BABY_CARE_INTERLOCK_ARMED_PIN  GPIO_PIN_1
#define BABY_CARE_INTERLOCK_ENABLE_PIN GPIO_PIN_2
#define BABY_CARE_INTERLOCK_READY_PIN  GPIO_PIN_3
#define BABY_CARE_INTERLOCK_POWER_PIN  GPIO_PIN_4
#define BABY_CARE_HEARTBEAT_PORT       GPIOF
#define BABY_CARE_HEARTBEAT_PIN        GPIO_PIN_5
#else
/* H743ZI ENIG board also has TF pullups; FPC1 camera must remain empty. */
#define BABY_CARE_INTERLOCK_PORT       GPIOF
#define BABY_CARE_INTERLOCK_STOP_PIN   GPIO_PIN_0
#define BABY_CARE_INTERLOCK_ARMED_PIN  GPIO_PIN_1
#define BABY_CARE_INTERLOCK_ENABLE_PIN GPIO_PIN_2
#define BABY_CARE_INTERLOCK_READY_PIN  GPIO_PIN_3
#define BABY_CARE_INTERLOCK_POWER_PIN  GPIO_PIN_4
#define BABY_CARE_HEARTBEAT_PORT       GPIOF
#define BABY_CARE_HEARTBEAT_PIN        GPIO_PIN_5
#endif
#define BABY_CARE_INTERLOCK_STABLE_MS  50U
/* Provisional ramp deadline; validate against the final switched power stage. */
#define BABY_CARE_INTERLOCK_POWER_TIMEOUT_MS 200U

/* Requires the external watchdog power gate; the GPIO alone is not protection. */
#define BABY_CARE_HEARTBEAT_TOGGLE_MS 50U

#define BABY_CARE_SENSOR_PERIOD_MS   2100U
#define BABY_CARE_SENSOR_STALE_MS    6000U
/* Unmanned desktop bring-up, not a calibrated loaded mechanism. */
#define BABY_CARE_SOOTHE_MAX_MS      10000U
#define BABY_CARE_REMOTE_HOLD_MS     10000U
#define BABY_CARE_SOOTHE_SPEED_PERCENT 20
/* Bench-only command bounds; repeated commands cannot extend a motion session. */
#define BABY_CARE_MOTION_SESSION_MAX_MS 10000U
#define BABY_CARE_MANUAL_MOTOR_MAX_PERCENT 20
#define BABY_CARE_VOICE_MOTOR_MS      2000U
#define BABY_CARE_VOICE_GIMBAL_MS     3000U
#define BABY_CARE_VOICE_PAN_STEP_DEG  10U
#define BABY_CARE_ESP32_STALE_MS     15000U
#define BABY_CARE_REPORT_PERIOD_MS   2000U
#define BABY_CARE_SOOTHE_PERIOD_MS   800U
#define BABY_CARE_ESP32_QUERY_PERIOD_MS 5000U
#define BABY_CARE_PAN_SAFE_MIN_DEG       60U
#define BABY_CARE_PAN_SAFE_MAX_DEG       120U
#define BABY_CARE_TILT_SAFE_MIN_DEG      75U
#define BABY_CARE_TILT_SAFE_MAX_DEG      105U
#define BABY_CARE_SERVO_STEP_DEG         1U
#define BABY_CARE_SERVO_STEP_MS          20U
#define BABY_CARE_SERVO_MIN_PULSE_US     1000U
#define BABY_CARE_SERVO_MAX_PULSE_US     2000U

#if BABY_CARE_SOOTHE_SPEED_PERCENT < 1 || BABY_CARE_SOOTHE_SPEED_PERCENT > 100
#error "Soothe PWM percentage must be in 1..100"
#endif
#if BABY_CARE_SERVO_MIN_PULSE_US >= BABY_CARE_SERVO_MAX_PULSE_US
#error "Servo pulse bounds must be ordered"
#endif

#if BABY_CARE_USE_DC_MOTOR
extern TIM_HandleTypeDef BABY_CARE_MOTOR_TIM;
#endif

#if BABY_CARE_USE_SERVO_GIMBAL
extern TIM_HandleTypeDef BABY_CARE_GIMBAL_PAN_TIM;
extern TIM_HandleTypeDef BABY_CARE_GIMBAL_TILT_TIM;
#endif

#if BABY_CARE_USE_VOICE_UART
extern UART_HandleTypeDef BABY_CARE_VOICE_UART;
#endif

#if BABY_CARE_USE_DEBUG_UART
extern UART_HandleTypeDef BABY_CARE_DEBUG_UART;
#endif

#if BABY_CARE_USE_ESP32_CAMERA
extern UART_HandleTypeDef BABY_CARE_ESP32_UART;
#endif

#ifdef __cplusplus
}
#endif

#endif
