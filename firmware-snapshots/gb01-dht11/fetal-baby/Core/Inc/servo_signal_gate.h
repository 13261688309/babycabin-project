#ifndef SERVO_SIGNAL_GATE_H
#define SERVO_SIGNAL_GATE_H
#include "servo_gimbal.h"

/* GB01 H743 fixture only: PA6/PA7, TIM3 CH1/CH2 at 1 MHz / 20 ms. */
void ServoSignal_Idle(void);
Servo_StatusTypeDef ServoSignal_Start(ServoGimbal_HandleTypeDef *gimbal);
#endif
