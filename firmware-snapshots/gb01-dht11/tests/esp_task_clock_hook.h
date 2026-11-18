#ifndef TEST_ESP_TASK_CLOCK_HOOK_H
#define TEST_ESP_TASK_CLOCK_HOOK_H
#include "stm32h7xx_hal.h"
void test_clock_disable_irq(void);
#undef __disable_irq
#define __disable_irq() test_clock_disable_irq()
#endif
