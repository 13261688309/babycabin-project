#ifndef SERVO_DIAG_TEST_HAL_H
#define SERVO_DIAG_TEST_HAL_H
#include <stdint.h>
typedef struct { volatile uint32_t CR, CFGR, D1CFGR, D2CFGR, AHB4ENR, APB1LENR; } RCC_TypeDef;
typedef struct { volatile uint32_t MODER, OTYPER, AFR[2]; } GPIO_TypeDef;
typedef struct { volatile uint32_t CR1, SMCR, CCMR1, CCER, PSC, ARR, CCR1, CCR2; } TIM_TypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
extern RCC_TypeDef diag_rcc;
extern GPIO_TypeDef diag_gpio;
extern TIM_TypeDef diag_tim;
#define RCC (&diag_rcc)
#define GPIOA (&diag_gpio)
#define TIM3 (&diag_tim)
#endif
