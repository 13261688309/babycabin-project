#ifndef TEST_DHT_HAL_H
#define TEST_DHT_HAL_H
#include "../hal/stm32h7xx_hal.h"

typedef struct { volatile uint32_t DEMCR; } TestCoreDebug;
typedef struct { volatile uint32_t CTRL, CYCCNT; } TestDwt;
extern TestCoreDebug test_core_debug;
TestDwt *test_dwt_access(void);
extern uint32_t SystemCoreClock;
#define CoreDebug (&test_core_debug)
#define DWT test_dwt_access()
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#endif
