#define GIMBAL_MUX_ENTRY baseline_mux_main
#include "test_usb_log_mux.c"
#include "gimbal_control.h"
static void control_put32(uint8_t *p,uint32_t v) { for(unsigned i=0;i<4U;++i) { p[i]=(uint8_t)(v>>(i*8U)); } }
static void control_frame(uint8_t *p,uint8_t op)
{
    memset(p,0,32);memcpy(p,"GC03",4);
    control_put32(p+4,GimbalControl_Epoch());control_put32(p+8,1U);
    control_put32(p+12,tick);control_put32(p+16,1234U);
    p[20]=100U;p[22]=90U;p[24]=op;p[26]=232U;p[27]=3U;
    if(op==4U) { p[20]=20U;p[22]=232U;p[23]=3U; }
    uint32_t c=UINT32_MAX;
    for(unsigned i=0;i<28U;++i) { c^=p[i];for(unsigned b=0;b<8U;++b) { c=(c>>1)^((c&1U)?0xedb88320U:0U); } }
    control_put32(p+28,~c);GimbalControl_Advertise(tick);
}
static void receive_bytes(const uint8_t *bytes,uint32_t size)
{
    memcpy(port.rx[1],bytes,size);port.received[1]=size;
    assert(USBD_LL_DataOutStage(&hUsbDeviceFS,1,port.rx[1])==USBD_OK);
}
int main(void)
{
    assert(baseline_mux_main()==0);
    for(unsigned motor=0;motor<2U;++motor) for(unsigned cancel=0;cancel<3U;++cancel) {
        uint8_t p[32];
        start();
        simulate_arm_latch=1U;
        BABY_CARE_INTERLOCK_PORT->ODR=BABY_CARE_INTERLOCK_STOP_PIN|BABY_CARE_INTERLOCK_POWER_PIN;
        BabyCare_Init();
        BabyCare_Task();tick+=BABY_CARE_INTERLOCK_STABLE_MS;BabyCare_Task();
        BABY_CARE_INTERLOCK_PORT->ODR|=BABY_CARE_INTERLOCK_ARMED_PIN;
        BabyCare_Task();tick+=BABY_CARE_INTERLOCK_STABLE_MS;BabyCare_Task();
        assert(!BabyCare_GetStatus()->safety_latched);BabyCare_Task();
        control_frame(p,motor?4U:1U);
        receive_bytes(p,13U);BabyCare_Task();assert(!GimbalControl_Lease());
        receive_bytes(p+13,19U);BabyCare_Task();
        assert(GimbalControl_Lease()==1234U);
        assert(motor?(htim2.enabled!=0U && htim3.enabled==0U):(htim3.enabled!=0U && htim2.enabled==0U));
        if(cancel==0U) { assert(setup(0x21,CDC_SET_CONTROL_LINE_STATE,0,0,0)==USBD_OK); }
        if(cancel==1U) { assert(USBD_LL_Reset(&hUsbDeviceFS)==USBD_OK); }
        if(cancel==2U) { control_frame(p,0U);receive_bytes(p,32U); }
        BabyCare_Task();
        assert(!GimbalControl_Lease() && htim3.enabled==0U && htim2.enabled==0U);
        check_guards();
    }
    puts("PASS gimbal USB: gimbal and rocking, fragmented actual CDC RX, DTR close, bus reset, STOP; actual ST stack with simulated HAL.");
    return 0;
}
