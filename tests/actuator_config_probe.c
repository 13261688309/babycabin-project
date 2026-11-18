#include <stdio.h>
#include "baby_care_config.h"

int main(void)
{
    printf("%u %u %u\n", (unsigned)BABY_CARE_SERVO_MIN_PULSE_US,
           (unsigned)BABY_CARE_SERVO_MAX_PULSE_US, (unsigned)BABY_CARE_SOOTHE_SPEED_PERCENT);
    return 0;
}
