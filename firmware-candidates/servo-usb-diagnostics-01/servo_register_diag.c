#include "servo_register_diag.h"
#include "stm32h7xx_hal.h"
#include "debug_uart_tx.h"
#include <stdio.h>

static uint32_t last_report, sample;

void ServoRegisterDiag_Task(uint32_t now, uint32_t epoch, uint8_t pwm)
{
    static const char *const names[] = {
        "RCC_CR", "RCC_CFGR", "RCC_D1CFGR", "RCC_D2CFGR", "RCC_AHB4ENR",
        "RCC_APB1LENR", "GPIOA_MODER", "GPIOA_OTYPER", "GPIOA_AFRL",
        "TIM3_CR1", "TIM3_SMCR", "TIM3_CCMR1", "TIM3_CCER", "TIM3_PSC",
        "TIM3_ARR", "TIM3_CCR1", "TIM3_CCR2"
    };
    char line[768];
    int length, added;
    if ((uint32_t)(now - last_report) < 1000U) { return; }
    last_report = now;
    /* Main-loop commands cannot interleave this sequential MMIO snapshot.
     * Register values do not prove a waveform at the connector. */
    const uint32_t words[] = {
        RCC->CR, RCC->CFGR, RCC->D1CFGR, RCC->D2CFGR, RCC->AHB4ENR,
        RCC->APB1LENR, GPIOA->MODER, GPIOA->OTYPER, GPIOA->AFR[0],
        TIM3->CR1, TIM3->SMCR, TIM3->CCMR1, TIM3->CCER, TIM3->PSC,
        TIM3->ARR, TIM3->CCR1, TIM3->CCR2
    };
    length = snprintf(line, sizeof(line),
        "servo_regs {\"v\":1,\"sample\":%lu,\"now_ms\":%lu,\"epoch\":%lu,\"pwm\":%u,\"registers\":{",
        (unsigned long)++sample, (unsigned long)now, (unsigned long)epoch, pwm != 0U);
    if (length < 0 || (size_t)length >= sizeof(line)) { return; }
    for (unsigned i = 0; i < sizeof(words)/sizeof(words[0]); ++i) {
        added = snprintf(line + length, sizeof(line) - (size_t)length,
            "%s\"%s\":\"0x%08lx\"", i ? "," : "", names[i], (unsigned long)words[i]);
        if (added < 0 || (size_t)added >= sizeof(line) - (size_t)length) { return; }
        length += added;
    }
    added = snprintf(line + length, sizeof(line) - (size_t)length, "}}\r\n");
    if (added < 0 || (size_t)added >= sizeof(line) - (size_t)length) { return; }
    length += added;
    (void)DebugTx_Write(line, (size_t)length, false);
}
