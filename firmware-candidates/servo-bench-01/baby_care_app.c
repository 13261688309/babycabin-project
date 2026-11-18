/* Unloaded desktop servo fixture. Not a baby-care or motor-control firmware. */
#include "baby_care_app.h"
#include "servo_gimbal.h"
#include "gimbal_control.h"
#include "debug_uart_tx.h"
#include <string.h>

static ServoGimbal_HandleTypeDef gimbal;
static BabyCare_StatusTypeDef status;
static uint32_t started, duration;
static uint8_t ready, running, stopped_this_cycle;

static void quiet_pins(void)
{
    GPIO_InitTypeDef gpio = {0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void BabyCare_Stop(void)
{
    GimbalControl_Invalidate();
    stopped_this_cycle = 1U;
    if (ready && ServoGimbal_Stop(&gimbal) != SERVO_OK) {
        status.fault_flags |= BABY_CARE_FAULT_SERVO;
    }
    /* Zero compare remains inactive even if a HAL stop reports an error. */
    __HAL_TIM_SET_COMPARE(&BABY_CARE_GIMBAL_PAN_TIM, BABY_CARE_GIMBAL_PAN_CH, 0U);
    __HAL_TIM_SET_COMPARE(&BABY_CARE_GIMBAL_TILT_TIM, BABY_CARE_GIMBAL_TILT_CH, 0U);
    running = 0U;
    status.motion = BABY_CARE_MOTION_NONE;
    status.motion_remaining_ms = 0U;
    status.pan_target_deg = gimbal.pan.current_angle;
    status.tilt_target_deg = (uint8_t)(180U - gimbal.tilt.current_angle);
    status.safety_latched = status.fault_flags != 0U;
    quiet_pins();
}

static void deadline(void)
{
    if (!running) { status.motion_remaining_ms = 0U; return; }
    const uint32_t elapsed = HAL_GetTick() - started;
    if (elapsed >= duration) { BabyCare_Stop(); }
    else { status.motion_remaining_ms = duration - elapsed; }
}

uint8_t BabyCare_RequestGimbal(uint16_t pan, uint16_t tilt, uint32_t hold_ms)
{
    deadline();
    if (!ready || status.fault_flags || stopped_this_cycle ||
        pan < 60U || pan > 120U || tilt < 75U || tilt > 105U ||
        hold_ms == 0U || hold_ms > 10000U) {
        status.rejected_commands++;
        return 0U;
    }
    if (!running) {
        /* Restore last commanded pulses after STOP; never assume measured position. */
        (void)Servo_SetAngle(&gimbal.pan, gimbal.pan.current_angle);
        (void)Servo_SetAngle(&gimbal.tilt, gimbal.tilt.current_angle);
        started = HAL_GetTick();
        duration = hold_ms;
        if (ServoGimbal_Start(&gimbal) != SERVO_OK) {
            status.fault_flags |= BABY_CARE_FAULT_SERVO;
            BabyCare_Stop();
            return 0U;
        }
        running = 1U;
    }
    if (ServoGimbal_SetAngles(&gimbal, (uint8_t)pan, (uint8_t)tilt) != SERVO_OK) {
        status.fault_flags |= BABY_CARE_FAULT_SERVO;
        BabyCare_Stop();
        return 0U;
    }
    status.pan_target_deg = (uint8_t)pan;
    status.tilt_target_deg = (uint8_t)tilt;
    status.motion = BABY_CARE_MOTION_GIMBAL;
    deadline();
    return running;
}

uint8_t BabyCare_RequestMotor(int16_t speed, uint32_t hold_ms)
{
    (void)speed; (void)hold_ms;
    status.rejected_commands++;
    return 0U;
}

uint8_t BabyCare_RequestMode(BabyCare_ModeTypeDef mode)
{
    if (mode == BABY_CARE_MODE_IDLE) { BabyCare_Stop(); return 1U; }
    status.rejected_commands++;
    return 0U;
}

uint8_t BabyCare_ClearSafetyLatch(void) { BabyCare_Stop(); return 0U; }

void BabyCare_Init(void)
{
    memset(&status, 0, sizeof(status));
    running = ready = stopped_this_cycle = 0U;
    GimbalControl_Init();
    (void)DebugTx_Init(&BABY_CARE_DEBUG_UART);
    quiet_pins();
    ready = ServoGimbal_Init(&gimbal, &BABY_CARE_GIMBAL_PAN_TIM,
        BABY_CARE_GIMBAL_PAN_CH, &BABY_CARE_GIMBAL_TILT_TIM,
        BABY_CARE_GIMBAL_TILT_CH) == SERVO_OK;
    if (ready) {
        Servo_SetPulseRange(&gimbal.pan, 1000U, 2000U);
        Servo_SetPulseRange(&gimbal.tilt, 1000U, 2000U);
        ready = ServoGimbal_SetSoftLimits(&gimbal, 60U, 120U, 75U, 105U) == SERVO_OK &&
            ServoGimbal_SetMotionProfile(&gimbal, 1U, 100U) == SERVO_OK;
    }
    if (!ready) { status.fault_flags |= BABY_CARE_FAULT_SERVO; }
    BabyCare_Stop();
}

void BabyCare_Task(void)
{
    stopped_this_cycle = 0U;
    deadline();
    GimbalControl_Task();
    if (running && ServoGimbal_Task(&gimbal, HAL_GetTick()) != SERVO_OK) {
        status.fault_flags |= BABY_CARE_FAULT_SERVO;
        BabyCare_Stop();
    }
    deadline();
    DebugTx_Task();
}

void BabyCare_OnUartRxComplete(UART_HandleTypeDef *uart) { (void)uart; }
void BabyCare_OnUartTxComplete(UART_HandleTypeDef *uart) { DebugTx_OnComplete(uart); }
void BabyCare_OnUartError(UART_HandleTypeDef *uart) { DebugTx_OnError(uart); }
const BabyCare_StatusTypeDef *BabyCare_GetStatus(void) { return &status; }
