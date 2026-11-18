/* Reuse the existing protocol cases with pin-aware HAL wrappers. */
#include "stm32h7xx_hal.h"
#define main original_bench_main
#define HAL_GPIO_Init original_gpio_init
#define HAL_TIM_PWM_Start original_pwm_start
#define HAL_TIM_PWM_Stop original_pwm_stop
#include "test_servo_bench.c"
#undef main
#undef HAL_GPIO_Init
#undef HAL_TIM_PWM_Start
#undef HAL_TIM_PWM_Stop
#include "servo_signal_gate.h"

static unsigned connections;
static uint32_t stop_failure = UINT32_MAX;
static uint8_t event_failure;

static uint8_t pins_low(void)
{
    for (unsigned pin=6; pin<=7; ++pin) {
        if (((GPIOA->MODER>>(2U*pin))&3U)!=1U ||
            ((GPIOA->PUPDR>>(2U*pin))&3U)!=2U ||
            (GPIOA->OTYPER&(1U<<pin)) || (GPIOA->ODR&(1U<<pin))) { return 0U; }
    }
    return 1U;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    if (port==GPIOA && (gpio->Pin&(GPIO_PIN_6|GPIO_PIN_7))) {
        CHECK(gpio->Pin==(GPIO_PIN_6|GPIO_PIN_7));
        CHECK(gpio->Pull==GPIO_PULLDOWN);
        CHECK((port->ODR&gpio->Pin)==0U);
        if (gpio->Mode==GPIO_MODE_AF_PP) {
            CHECK(test_primask==1U && pins_low());
            CHECK(htim3.enabled==((1U<<TIM_CHANNEL_1)|(1U<<TIM_CHANNEL_2)));
            CHECK(htim3.CNT==2500U);
            CHECK(htim3.loaded[TIM_CHANNEL_1]==htim3.compare[TIM_CHANNEL_1]);
            CHECK(htim3.loaded[TIM_CHANNEL_2]==htim3.compare[TIM_CHANNEL_2]);
            CHECK(gpio->Alternate==GPIO_AF2_TIM3);
            ++connections;
        } else { CHECK(gpio->Mode==GPIO_MODE_OUTPUT_PP); }
    }
    for (unsigned pin=0; pin<16; ++pin) {
        if (!(gpio->Pin&(1U<<pin))) { continue; }
        port->MODER=(port->MODER&~(3U<<(2U*pin)))|((gpio->Mode&3U)<<(2U*pin));
        port->PUPDR=(port->PUPDR&~(3U<<(2U*pin)))|(gpio->Pull<<(2U*pin));
        port->OTYPER&=~(1U<<pin);
        if (gpio->Mode==GPIO_MODE_AF_PP) {
            const unsigned bank=pin/8U, shift=(pin%8U)*4U;
            port->AFR[bank]=(port->AFR[bank]&~(15U<<shift))|(gpio->Alternate<<shift);
        }
    }
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    CHECK(pins_low() && test_primask==1U);
    CHECK(timer->CNT==2500U);
    return original_pwm_start(timer,channel);
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
    CHECK(pins_low());
    if (channel==stop_failure) { return HAL_ERROR; }
    return original_pwm_stop(timer,channel);
}

HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *timer, uint32_t event)
{
    CHECK(timer==&htim3 && event==TIM_EVENTSOURCE_UPDATE);
    CHECK(test_primask==1U && pins_low());
    if (event_failure) { return HAL_ERROR; }
    memcpy(timer->loaded,timer->compare,sizeof(timer->compare));
    timer->CNT=0U;
    timer->SR|=TIM_FLAG_UPDATE;
    return HAL_OK;
}

static void failure_and_idle_cases(void)
{
    uint8_t p[32];
    const uint32_t other_mode=0xa5a50000U, other_odr=GPIO_PIN_4|GPIO_PIN_5;
    GPIOA->MODER=other_mode; GPIOA->ODR=other_odr|GPIO_PIN_6|GPIO_PIN_7;
    ServoSignal_Idle();
    CHECK(pins_low());
    CHECK((GPIOA->MODER&~(15U<<12))==other_mode);
    CHECK(GPIOA->ODR==other_odr);

    for (unsigned channel=0; channel<2; ++channel) {
        init(0U); const unsigned prior=connections;
        fail_channel=channel?TIM_CHANNEL_2:TIM_CHANNEL_1;
        packet(p,1,95,95,1000,1); send(p);
        CHECK(stopped() && pins_low() && connections==prior);
        CHECK(BabyCare_GetStatus()->fault_flags && test_primask==0U);
    }
    fail_channel=UINT32_MAX;
    init(0U); event_failure=1U;
    packet(p,1,95,95,1000,1); send(p);
    CHECK(stopped() && pins_low() && starts==0U);
    CHECK(BabyCare_GetStatus()->fault_flags && test_primask==0U);
    event_failure=0U;

    for (unsigned channel=0; channel<2; ++channel) {
        start(0U,p); stop_failure=channel?TIM_CHANNEL_2:TIM_CHANNEL_1;
        packet(p,0,90,90,0,2); send(p);
        CHECK(pins_low() && !GimbalControl_Lease());
        CHECK(htim3.compare[TIM_CHANNEL_1]==0U && htim3.compare[TIM_CHANNEL_2]==0U);
        CHECK(BabyCare_GetStatus()->fault_flags);
        const unsigned prior=connections;
        packet(p,1,95,95,1000,3); send(p);
        CHECK(pins_low() && connections==prior);
        stop_failure=UINT32_MAX;
    }

    init(0U); test_primask=1U;
    packet(p,1,95,95,1000,1); send(p); CHECK(test_primask==1U);
    packet(p,0,90,90,0,2); send(p); CHECK(test_primask==1U && pins_low());
    test_primask=0U;
    init(0U); const unsigned prior=connections;
    for (unsigned n=0; n<200; ++n) { tick+=100; BabyCare_Task(); CHECK(pins_low()); }
    CHECK(connections==prior && starts==0U);
    CHECK(ServoSignal_Start(NULL)==SERVO_ERROR_PARAM && pins_low());
}

int main(void)
{
    CHECK(original_bench_main()==0);
    CHECK(pins_low());
    failure_and_idle_cases();
    printf("PASS idle-low gate: %lu assertions including existing protocol cases; simulated HAL only.\n",
           (unsigned long)checks);
    return 0;
}
