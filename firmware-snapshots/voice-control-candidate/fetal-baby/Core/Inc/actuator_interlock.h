#ifndef ACTUATOR_INTERLOCK_H
#define ACTUATOR_INTERLOCK_H

#include <stdint.h>

typedef enum {
    INTERLOCK_TRIP_NONE = 0,
    INTERLOCK_TRIP_STOP = 1,
    INTERLOCK_TRIP_FEEDBACK = 2,
    INTERLOCK_TRIP_SOFTWARE = 3,
    INTERLOCK_TRIP_POWER = 4
} Interlock_TripReason;

typedef struct {
    uint8_t stop_ok;
    uint8_t armed;
    uint8_t ready;
    uint8_t enabled;
    uint8_t tripped;
    uint8_t qualified;
    uint8_t power_good;
    uint8_t power_pending;
    Interlock_TripReason reason;
    uint32_t trip_count;
} Interlock_Status;

/* GPIO clocks/modes are owned by MX_GPIO_Init. Calls run in the main context. */
void Interlock_Init(void);
void Interlock_Poll(void);
void Interlock_Disable(void);
void Interlock_Trip(void);
uint8_t Interlock_Enable(void);
/* Reset only permits a new physical ARM edge; it never drives ACT_EN high. */
uint8_t Interlock_Reset(void);
const Interlock_Status *Interlock_GetStatus(void);

#endif
