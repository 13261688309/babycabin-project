#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "baby_care_app.h"
#include "gimbal_control.h"
#include "debug_uart_tx.h"

GPIO_TypeDef test_gpio[8];
uint32_t test_primask;
TIM_HandleTypeDef htim2 = {99U}, htim3 = {19999U};
UART_HandleTypeDef huart1, huart2, huart3;
static uint32_t tick, starts, fail_channel, checks;
static char telemetry[300];
#define CHECK(x) do { assert(x); ++checks; } while (0)
uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s)
{ p->ODR = s ? p->ODR | pin : p->ODR & ~pin; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, uint32_t c)
{
    CHECK(t == &htim3);
    if (c == fail_channel) { return HAL_ERROR; }
    ++starts; t->enabled |= 1U << c; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t, uint32_t c)
{ t->enabled &= ~(1U << c); return HAL_OK; }
bool DebugTx_Init(UART_HandleTypeDef *u) { (void)u; return true; }
bool DebugTx_Write(const void *p, size_t n, bool a)
{ (void)a; CHECK(n < sizeof(telemetry)); memcpy(telemetry,p,n); telemetry[n]=0; return true; }
void DebugTx_Task(void) {}
void DebugTx_OnComplete(UART_HandleTypeDef *u) { (void)u; }
void DebugTx_OnError(UART_HandleTypeDef *u) { (void)u; }

static void u32(uint8_t *p, uint32_t v)
{ for (unsigned i=0; i<4; ++i) { p[i]=(uint8_t)(v>>(8U*i)); } }
static void checksum(uint8_t *p)
{
    uint32_t c=UINT32_MAX;
    for (unsigned i=0; i<28; ++i) { c^=p[i]; for(unsigned b=0;b<8;++b) { c=(c>>1)^((c&1U)?0xedb88320U:0U); } }
    u32(p+28,~c);
}
static void packet(uint8_t *p, uint8_t op, unsigned pan, unsigned tilt, unsigned hold, uint32_t seq)
{
    memset(p,0,32); memcpy(p,"GB01",4);
    u32(p+4,GimbalControl_Epoch()); u32(p+8,seq); u32(p+12,tick); u32(p+16,1234U);
    p[20]=(uint8_t)pan; p[21]=(uint8_t)(pan>>8);
    p[22]=(uint8_t)tilt; p[23]=(uint8_t)(tilt>>8); p[24]=op;
    p[26]=(uint8_t)hold; p[27]=(uint8_t)(hold>>8);
    checksum(p); GimbalControl_Advertise(tick);
}
static void send(uint8_t *p) { GimbalControl_OnBytes(p,32); BabyCare_Task(); }
static uint8_t stopped(void)
{ return htim3.enabled==0 && htim3.compare[TIM_CHANNEL_1]==0 && htim3.compare[TIM_CHANNEL_2]==0 && !GimbalControl_Lease(); }
static void init(uint32_t base)
{
    tick=base; starts=0; fail_channel=UINT32_MAX; memset(test_gpio,0,sizeof(test_gpio));
    memset(htim3.compare,0,sizeof(htim3.compare)); htim3.enabled=0;
    BabyCare_Init(); BabyCare_Task(); CHECK(stopped()); CHECK(starts==0);
    CHECK(GPIOF->ODR==0 && GPIOB->ODR==0 && GPIOA->ODR==0);
}
static void start(uint32_t base, uint8_t *p)
{ init(base); packet(p,1,110,95,1000,1); send(p); CHECK(htim3.enabled!=0 && GimbalControl_Lease()==1234U); }
static void cases(uint32_t base)
{
    uint8_t p[32],q[32]; uint32_t n;
    init(base); packet(p,1,110,95,1000,1); memcpy(p,"GC03",4); checksum(p); send(p); CHECK(stopped());
    packet(p,4,20,1000,1000,1); send(p); CHECK(stopped()); CHECK(htim2.enabled==0);
    packet(p,1,121,95,1000,1); send(p); CHECK(stopped());
    packet(p,1,90,74,1000,1); send(p); CHECK(stopped());
    packet(p,1,90,90,10001,1); send(p); CHECK(stopped());
    packet(p,1,90,90,1000,1); p[20]^=1; send(p); CHECK(stopped());
    packet(p,1,90,90,1000,1); tick+=600; send(p); CHECK(stopped());
    start(base,p); CHECK(htim3.compare[TIM_CHANNEL_1]==1500);
    tick+=99; BabyCare_Task(); CHECK(htim3.compare[TIM_CHANNEL_1]==1500);
    tick+=1; BabyCare_Task(); CHECK(htim3.compare[TIM_CHANNEL_1]==1505);
    send(p); CHECK(GimbalControl_Accepted()==1);
    tick+=500; BabyCare_Task(); CHECK(stopped()); send(p); CHECK(stopped());
    start(base,p); GimbalControl_OnSessionReset(); BabyCare_Task(); CHECK(stopped());
    start(base,p); for(unsigned i=0;i<9;++i) { GimbalControl_OnBytes(p,32); } BabyCare_Task(); CHECK(stopped());
    init(base); packet(p,1,110,95,1000,1); packet(q,0,90,90,0,2);
    GimbalControl_OnBytes(p,32); GimbalControl_OnBytes(q,32); BabyCare_Task(); CHECK(stopped()); CHECK(starts==0);
    init(base); packet(p,1,110,95,1000,1); packet(q,3,90,90,0,2);
    GimbalControl_OnBytes(p,32); GimbalControl_OnBytes(q,32); BabyCare_Task(); CHECK(stopped()); CHECK(starts==0);
    start(base,p); for(unsigned i=1;i<=10;++i) {
        tick+=100; packet(q,2,100,90,0,i+1); send(q);
    } CHECK(stopped()); CHECK(GimbalControl_Accepted()==10);
    n=starts; send(q); CHECK(stopped() && starts==n);
    start(base,p); packet(q,0,90,90,0,2); send(q); CHECK(stopped());
    packet(p,1,95,90,1000,3); send(p); CHECK(htim3.enabled!=0);
    CHECK(BabyCare_RequestMotor(20,1000)==0); CHECK(htim2.enabled==0);
    CHECK(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE)==0);
    init(base); fail_channel=TIM_CHANNEL_2; packet(p,1,95,90,1000,1); send(p);
    CHECK(stopped()); CHECK(BabyCare_GetStatus()->fault_flags);
    fail_channel=UINT32_MAX; packet(p,1,95,90,1000,2); send(p); CHECK(stopped());
    init(base); tick+=100; BabyCare_Task(); CHECK(strstr(telemetry,"bench_link v=1")!=NULL);
    CHECK(strstr(telemetry,"external=0")!=NULL);
}
static void independent_axes(void)
{
    const unsigned targets[][2]={{85,90},{95,90},{90,85},{90,95},{90,90}};
    for (unsigned n=0; n<sizeof(targets)/sizeof(targets[0]); ++n) {
        uint8_t p[32];
        const unsigned pan=targets[n][0], tilt=targets[n][1];
        init(0U);
        packet(p,1,pan,tilt,1500,1); send(p);
        CHECK(htim3.enabled==((1U<<TIM_CHANNEL_1)|(1U<<TIM_CHANNEL_2)));
        CHECK(htim3.compare[TIM_CHANNEL_1]==1500U);
        CHECK(htim3.compare[TIM_CHANNEL_2]==1500U);
        for (unsigned step=1; step<=5; ++step) {
            const unsigned pangle=90U+(pan>90U?step:0U)-(pan<90U?step:0U);
            const unsigned tangle=90U+(tilt<90U?step:0U)-(tilt>90U?step:0U);
            tick+=100;
            packet(p,2,pan,tilt,0,step+1); send(p);
            CHECK(htim3.compare[TIM_CHANNEL_1]==1000U+1000U*pangle/180U);
            CHECK(htim3.compare[TIM_CHANNEL_2]==1000U+1000U*tangle/180U);
        }
        packet(p,0,90,90,0,7); send(p); CHECK(stopped());
    }
}

int main(void)
{
    cases(0U); cases(UINT32_MAX-250U);
    independent_axes();
    printf("PASS servo bench: %lu assertions, simulated HAL; not physical motion.\n",(unsigned long)checks);
    return 0;
}
