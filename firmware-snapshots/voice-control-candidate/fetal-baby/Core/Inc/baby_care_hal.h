#ifndef BABY_CARE_HAL_H
#define BABY_CARE_HAL_H

#if defined(STM32F407xx)
#include "stm32f4xx_hal.h"
#else
/* H743 and the existing host-test HAL use this header. */
#include "stm32h7xx_hal.h"
#endif

#endif
