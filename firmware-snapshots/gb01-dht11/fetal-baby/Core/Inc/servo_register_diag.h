#ifndef SERVO_REGISTER_DIAG_H
#define SERVO_REGISTER_DIAG_H
#include <stdint.h>

/* Passive main-loop snapshot; this function never starts or changes PWM. */
void ServoRegisterDiag_Task(uint32_t now, uint32_t epoch, uint8_t pwm);

#endif
