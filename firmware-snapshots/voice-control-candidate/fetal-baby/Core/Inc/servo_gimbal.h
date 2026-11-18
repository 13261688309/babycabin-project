#ifndef SERVO_GIMBAL_H7_H
#define SERVO_GIMBAL_H7_H

#include "baby_care_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_DEFAULT_MIN_PULSE_US 500U
#define SERVO_DEFAULT_MAX_PULSE_US 2500U
#define SERVO_DEFAULT_PERIOD_US    20000U
#define SERVO_DEFAULT_MIN_ANGLE    0U
#define SERVO_DEFAULT_MAX_ANGLE    180U
#define SERVO_DEFAULT_CENTER_ANGLE 90U
#define SERVO_DEFAULT_STEP_DEGREES  1U
#define SERVO_DEFAULT_STEP_MS       20U

typedef enum
{
    SERVO_OK = 0,
    SERVO_ERROR_PARAM,
    SERVO_ERROR_HAL
} Servo_StatusTypeDef;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    uint16_t period_us;

    uint8_t min_angle;
    uint8_t max_angle;
    uint8_t soft_min_angle;
    uint8_t soft_max_angle;
    uint8_t current_angle;
    uint8_t target_angle;
    uint8_t step_degrees;
    uint16_t step_period_ms;
    uint32_t last_step_ms;
    uint8_t running;
} Servo_HandleTypeDef;

typedef struct
{
    Servo_HandleTypeDef pan;
    Servo_HandleTypeDef tilt;
    uint8_t tilt_reverse;
} ServoGimbal_HandleTypeDef;

Servo_StatusTypeDef Servo_Init(Servo_HandleTypeDef *hservo,
                               TIM_HandleTypeDef *htim,
                               uint32_t channel);
Servo_StatusTypeDef Servo_Start(Servo_HandleTypeDef *hservo);
Servo_StatusTypeDef Servo_Stop(Servo_HandleTypeDef *hservo);
Servo_StatusTypeDef Servo_SetAngle(Servo_HandleTypeDef *hservo, uint8_t angle);
Servo_StatusTypeDef Servo_SetPulseUs(Servo_HandleTypeDef *hservo, uint16_t pulse_us);
void Servo_SetPulseRange(Servo_HandleTypeDef *hservo, uint16_t min_pulse_us, uint16_t max_pulse_us);
void Servo_SetAngleRange(Servo_HandleTypeDef *hservo, uint8_t min_angle, uint8_t max_angle);
void Servo_SetPeriodUs(Servo_HandleTypeDef *hservo, uint16_t period_us);
Servo_StatusTypeDef Servo_SetSoftLimits(Servo_HandleTypeDef *hservo, uint8_t min_angle, uint8_t max_angle);
Servo_StatusTypeDef Servo_SetMotionProfile(Servo_HandleTypeDef *hservo, uint8_t step_degrees, uint16_t step_period_ms);
Servo_StatusTypeDef Servo_SetTargetAngle(Servo_HandleTypeDef *hservo, uint8_t angle);
Servo_StatusTypeDef Servo_Task(Servo_HandleTypeDef *hservo, uint32_t now_ms);
uint8_t Servo_GetAngle(const Servo_HandleTypeDef *hservo);
uint8_t Servo_GetTargetAngle(const Servo_HandleTypeDef *hservo);

Servo_StatusTypeDef ServoGimbal_Init(ServoGimbal_HandleTypeDef *hgimbal,
                                     TIM_HandleTypeDef *pan_htim,
                                     uint32_t pan_channel,
                                     TIM_HandleTypeDef *tilt_htim,
                                     uint32_t tilt_channel);
Servo_StatusTypeDef ServoGimbal_Start(ServoGimbal_HandleTypeDef *hgimbal);
Servo_StatusTypeDef ServoGimbal_Stop(ServoGimbal_HandleTypeDef *hgimbal);
Servo_StatusTypeDef ServoGimbal_SetAngles(ServoGimbal_HandleTypeDef *hgimbal,
                                          uint8_t pan_angle,
                                          uint8_t tilt_angle);
Servo_StatusTypeDef ServoGimbal_SetByIndex(ServoGimbal_HandleTypeDef *hgimbal,
                                           uint8_t index,
                                           uint8_t angle);
Servo_StatusTypeDef ServoGimbal_ParseRemoteFrame(ServoGimbal_HandleTypeDef *hgimbal,
                                                 const uint8_t *frame,
                                                 uint16_t len);
Servo_StatusTypeDef ServoGimbal_SetSoftLimits(ServoGimbal_HandleTypeDef *hgimbal,
                                              uint8_t pan_min, uint8_t pan_max,
                                              uint8_t tilt_min, uint8_t tilt_max);
Servo_StatusTypeDef ServoGimbal_SetMotionProfile(ServoGimbal_HandleTypeDef *hgimbal,
                                                 uint8_t step_degrees,
                                                 uint16_t step_period_ms);
Servo_StatusTypeDef ServoGimbal_Task(ServoGimbal_HandleTypeDef *hgimbal, uint32_t now_ms);
void ServoGimbal_SetTiltReverse(ServoGimbal_HandleTypeDef *hgimbal, uint8_t enable);
Servo_StatusTypeDef ServoGimbal_DecodeRemoteFrame(const uint8_t *frame, uint16_t len,
                                                 uint8_t *index, uint8_t *angle);

#ifdef __cplusplus
}
#endif

#endif
