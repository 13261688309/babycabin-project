#define INDEPENDENT_ENTRY independent_regression_main
#include "test_independent_actuators.c"
#include "gimbal_control.h"
static unsigned gchecks;
#define G(expr) do { assert(expr); ++gchecks; } while(0)
static void put32(uint8_t *p, uint32_t v) { for(unsigned i=0;i<4U;++i) { p[i]=(uint8_t)(v>>(i*8U)); } }
static void checksum(uint8_t *p)
{
    uint32_t c=UINT32_MAX;
    for(unsigned i=0;i<28U;++i) { c^=p[i]; for(unsigned b=0;b<8U;++b) { c=(c>>1)^((c&1U)?0xedb88320U:0U); } }
    put32(p+28,~c);
}
static void packet(uint8_t p[32], uint8_t op, unsigned pan, unsigned tilt, unsigned duration, uint32_t seq)
{
    memset(p,0,32); memcpy(p,"GC03",4);
    put32(p+4,GimbalControl_Epoch()); put32(p+8,seq); put32(p+12,tick); put32(p+16,1234);
    p[20]=(uint8_t)pan; p[21]=(uint8_t)(pan>>8); p[22]=(uint8_t)tilt; p[23]=(uint8_t)(tilt>>8); p[24]=op;
    p[26]=(uint8_t)duration; p[27]=(uint8_t)(duration>>8);
    checksum(p); GimbalControl_Advertise(tick);
}
static void send(uint8_t p[32]) { GimbalControl_OnBytes(p,32); BabyCare_Task(); }
static void start_test(uint32_t base)
{
    uint8_t p[32];
    ready(base);
    packet(p,1,110,95,1000,1); send(p);
    G(GimbalControl_Lease()==1234 && BabyCare_GetStatus()->motion==BABY_CARE_MOTION_GIMBAL);
    G(htim2.enabled==0U && htim3.enabled!=0U);
}
static void control_cases(uint32_t base)
{
    uint8_t p[32], q[32]; uint32_t count, old_epoch;
    start_test(base); old_epoch=GimbalControl_Epoch();
    tick+=100U; packet(p,2,100,85,0,2); send(p);
    G(BabyCare_GetStatus()->pan_target_deg==100U && BabyCare_GetStatus()->tilt_target_deg==85U);
    G(GimbalControl_Accepted()==2U);
    send(p); G(GimbalControl_Rejected()==1U); /* duplicate */
    packet(p,2,121,85,0,3); send(p); G(GimbalControl_Accepted()==2U);
    packet(p,2,100,74,0,3); send(p); G(GimbalControl_Accepted()==2U);
    packet(p,2,100,85,0,3); p[20]^=1U; send(p); G(GimbalControl_Accepted()==2U);
    tick+=600U; BabyCare_Task(); G(stopped() && GimbalControl_Lease()==0U);
    G(GimbalControl_Epoch()!=old_epoch); send(p); G(stopped());
    start_test(base); packet(p,2,100,85,0,2);
    app_voice(VOICE_CMD_STOP_1); send(p);
    G(stopped() && !GimbalControl_Lease());
    for(unsigned i=0;i<4U;++i) { send(p); G(stopped()); }
    start_test(base); voice(VOICE_CMD_TURN_LEFT);
    G(!GimbalControl_Lease() && BabyCare_GetStatus()->pan_target_deg==100U);
    start_test(base); app_remote("$B080#"); BabyCare_Task();
    G(!GimbalControl_Lease() && BabyCare_GetStatus()->pan_target_deg==80U);
    start_test(base); GimbalControl_OnSessionReset(); BabyCare_Task(); G(stopped());
    start_test(base); packet(p,2,100,85,0,2);
    GimbalControl_OnBytes(p,16); app_voice(VOICE_CMD_STOP_2); BabyCare_Task();
    GimbalControl_OnBytes(p+16,16); BabyCare_Task(); G(stopped());
    ready(base); packet(p,1,110,95,1000,1); packet(q,0,0,0,0,2);
    GimbalControl_OnBytes(p,32); GimbalControl_OnBytes(q,32); BabyCare_Task();
    G(stopped() && GimbalControl_Accepted()==0U);
    start_test(base); packet(p,3,90,90,0,2); send(p);
    G(stopped() && GimbalControl_Lease()==0U);
    start_test(base); packet(p,3,90,90,0,2);
    voice(VOICE_CMD_TURN_LEFT); send(p);
    G(BabyCare_GetStatus()->motion==BABY_CARE_MOTION_GIMBAL && htim3.enabled!=0U);
    G(BabyCare_GetStatus()->pan_target_deg==100U && GimbalControl_Lease()==0U);
    packet(p,0,90,90,0,3); send(p); G(stopped());
    ready(base); packet(p,1,110,95,1000,1); packet(q,3,90,90,0,2);
    GimbalControl_OnBytes(p,32); GimbalControl_OnBytes(q,32); BabyCare_Task();
    G(stopped() && GimbalControl_Accepted()==0U);
    start_test(base); packet(p,3,90,90,0,2); put32(p+16,9999U); checksum(p); send(p);
    G(BabyCare_GetStatus()->motion==BABY_CARE_MOTION_GIMBAL && GimbalControl_Lease()==1234U);
    ready(base); packet(p,1,110,95,1000,1); memcpy(p,"GC01",4); checksum(p); send(p);
    G(stopped() && GimbalControl_Accepted()==0U);
    start_test(base); packet(p,2,100,85,0,2);
    for(unsigned i=0;i<9U;++i) { GimbalControl_OnBytes(p,32); }
    BabyCare_Task(); G(stopped());
    start_test(base); count=GimbalControl_Accepted();
    for(unsigned i=0;i<10U;++i) { tick+=100U; packet(p,2,100,90,0,i+2U); send(p); }
    G(stopped() && GimbalControl_Accepted()==count+9U); /* deadline cannot renew */
    ready(base); packet(p,1,100,90,1000,1); tick+=600U; send(p); G(stopped());
    start_test(base); BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_STOP_PIN;
    packet(p,2,100,90,0,2); send(p); G(stopped() && BabyCare_GetStatus()->safety_latched);
}
static void rocking_cases(uint32_t base)
{
    uint8_t p[32],q[32];
    ready(base);packet(p,4,20,500,2000,1);send(p);
    G(BabyCare_GetStatus()->motion==BABY_CARE_MOTION_MOTOR && htim3.enabled==0U);
    G(BabyCare_GetStatus()->motor_target_percent==20 && GimbalControl_Lease()==1234U);
    for(unsigned i=1;i<=20U;++i) {
        advance(100U);packet(p,5,20,500,0,i+1U);send(p);
        if(i==5U) { G(BabyCare_GetStatus()->motor_target_percent==-20); }
        if(i==10U) { G(BabyCare_GetStatus()->motor_target_percent==20); }
        G(htim3.enabled==0U && unsafe_direction_changes==0U);
    }
    G(stopped() && GimbalControl_Lease()==0U);
    ready(base);packet(p,4,20,1000,3000,1);send(p);advance(600U);G(stopped());
    ready(base);packet(p,4,20,1000,3000,1);send(p);packet(q,3,90,90,0,2);
    voice(VOICE_CMD_TURN_RIGHT);send(q);
    G(BabyCare_GetStatus()->motion==BABY_CARE_MOTION_GIMBAL && htim2.enabled==0U);
    ready(base);packet(p,4,20,1000,3000,1);packet(q,0,90,90,0,2);
    GimbalControl_OnBytes(p,32);GimbalControl_OnBytes(q,32);BabyCare_Task();G(stopped());
    ready(base);packet(p,4,20,1000,3000,1);send(p);voice(VOICE_CMD_STOP_1);G(stopped());
    for(unsigned i=0;i<4U;++i) {
        ready(base);packet(p,4,i==0U?21U:20U,i==1U?499U:i==2U?2001U:1000U,i==3U?5001U:3000U,1);send(p);G(stopped());
    }
    ready(base);packet(p,4,20,1000,3000,1);send(p);
    packet(p,5,15,1000,0,2);send(p);G(GimbalControl_Accepted()==1U);
    packet(p,2,90,90,0,3);send(p);G(GimbalControl_Accepted()==1U);
    BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_STOP_PIN;
    BabyCare_Task();G(stopped());
}
int main(void)
{
    G(independent_regression_main()==0);
    control_cases(0U); control_cases(UINT32_MAX-250U);
    rocking_cases(0U); rocking_cases(UINT32_MAX-750U);
    printf("PASS gimbal control: %u assertions; real application and parsers, simulated HAL.\n",gchecks);
    return 0;
}
