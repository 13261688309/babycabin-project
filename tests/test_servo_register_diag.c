#include "servo_register_diag.h"
#include "debug_uart_tx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

RCC_TypeDef diag_rcc = {5U, 0U, 0U, 0U, 1U, 2U};
GPIO_TypeDef diag_gpio = {0xa000U, 0U, {0x22000000U, 0U}};
TIM_TypeDef diag_tim = {0x80U, 0U, 0x6868U, 0U, 63U, 19999U, 0U, 0U};
static char saved[DEBUG_TX_FRAME_SIZE + 1U];
static unsigned writes;
static bool accept_log = true;

bool DebugTx_Write(const void *data, size_t length, bool alert)
{
    assert(!alert && length < sizeof(saved));
    memcpy(saved, data, length);
    saved[length] = '\0';
    assert(strstr(saved, "\r\n") == saved + length - 2U);
    ++writes;
    return accept_log;
}

int main(void)
{
    RCC_TypeDef rcc_before = diag_rcc;
    GPIO_TypeDef gpio_before = diag_gpio;
    TIM_TypeDef tim_before = diag_tim;
    ServoRegisterDiag_Task(999U, 1U, 0U);
    assert(writes == 0U);
    ServoRegisterDiag_Task(1000U, 1U, 0U);
    assert(writes == 1U);
    assert(strstr(saved, "\"pwm\":0") && strstr(saved, "\"TIM3_CCR2\":\"0x00000000\""));
    fputs(saved, stdout);
    assert(memcmp(&rcc_before, &diag_rcc, sizeof(diag_rcc)) == 0);
    assert(memcmp(&gpio_before, &diag_gpio, sizeof(diag_gpio)) == 0);
    assert(memcmp(&tim_before, &diag_tim, sizeof(diag_tim)) == 0);
    ServoRegisterDiag_Task(1999U, 1U, 0U);
    assert(writes == 1U);
    diag_tim.CR1 = 0x81U; diag_tim.CCER = 0x11U;
    diag_tim.CCR1 = 1500U; diag_tim.CCR2 = 1555U;
    tim_before = diag_tim;
    ServoRegisterDiag_Task(2000U, 2U, 1U);
    assert(writes == 2U);
    assert(strstr(saved, "\"pwm\":1") && strstr(saved, "\"TIM3_CCR2\":\"0x00000613\""));
    fputs(saved, stdout);
    assert(memcmp(&tim_before, &diag_tim, sizeof(diag_tim)) == 0);
    accept_log = false;
    ServoRegisterDiag_Task(UINT32_MAX - 500U, UINT32_MAX, 1U);
    assert(writes == 3U);
    ServoRegisterDiag_Task(498U, 3U, 0U);
    assert(writes == 3U);
    ServoRegisterDiag_Task(499U, 3U, 0U);
    assert(writes == 4U);
    assert(memcmp(&tim_before, &diag_tim, sizeof(diag_tim)) == 0);
    puts("PASS passive servo register diagnostics: rate, wrap, idle/active encoding, no MMIO writes, no retry on full log queue");
    return 0;
}
