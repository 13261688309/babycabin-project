#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include <stdint.h>
/* Separate candidate control protocol, never part of BV01 observation metadata. */
void GimbalControl_Init(void);
void GimbalControl_OnBytes(const uint8_t *data, uint32_t length);
void GimbalControl_OnSessionReset(void);
void GimbalControl_Invalidate(void);
void GimbalControl_Task(void);
uint32_t GimbalControl_Epoch(void);
uint32_t GimbalControl_Lease(void);
uint32_t GimbalControl_Accepted(void);
uint32_t GimbalControl_Rejected(void);
void GimbalControl_Advertise(uint32_t now);
#endif
