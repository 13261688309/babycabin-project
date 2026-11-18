#include "servo_signal_gate.h"
#include "tim.h"

static void signal_mode(uint32_t mode)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = mode;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void ServoSignal_Idle(void)
{
    /* Preload ODR before taking ownership from the timer. No servo power cut. */
    signal_mode(GPIO_MODE_OUTPUT_PP);
}

Servo_StatusTypeDef ServoSignal_Start(ServoGimbal_HandleTypeDef *gimbal)
{
    Servo_StatusTypeDef result = SERVO_ERROR_PARAM;
    ServoSignal_Idle();
    if (gimbal == NULL || gimbal->pan.htim != &htim3 ||
        gimbal->tilt.htim != &htim3 || gimbal->pan.channel != TIM_CHANNEL_1 ||
        gimbal->tilt.channel != TIM_CHANNEL_2 ||
        __HAL_TIM_GET_AUTORELOAD(&htim3) != 19999U ||
        __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) < 1000U ||
        __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) > 2000U ||
        __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2) < 1000U ||
        __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2) > 2000U) {
        return result;
    }

    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Load both preload registers while the pins are held low. Start in the
     * inactive part of the 20 ms frame; the first exposed pulse starts at wrap. */
    if (HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE) == HAL_OK) {
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        __HAL_TIM_SET_COUNTER(&htim3, 2500U);
        result = ServoGimbal_Start(gimbal);
        if (result == SERVO_OK) { signal_mode(GPIO_MODE_AF_PP); }
    } else {
        result = SERVO_ERROR_HAL;
    }
    __set_PRIMASK(mask);
    return result;
}
