#include "gimbal_control.h"
#include "baby_care_app.h"
#include "debug_uart_tx.h"
#include <stdio.h>
#include <string.h>

#define SLOTS 8U
#define LEASE_TIMEOUT_MS 600U
typedef struct { uint8_t bytes[64]; uint32_t count, at; } Chunk;
static Chunk chunks[SLOTS];
static volatile uint8_t head, tail, reset_pending;
static uint8_t partial[32], used, have_sequence, ad_count, ad_next;
static uint32_t epoch, lease, sequence, refreshed, last_byte, accepted, rejected, last_report;
static uint32_t advertised[4];
static uint8_t rocking, rock_speed, rock_phase;
static uint16_t rock_period;
static uint32_t rock_started;

static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
static uint32_t crc(const uint8_t *p)
{
    uint32_t c = UINT32_MAX;
    for (unsigned i=0; i<28U; ++i) {
        c ^= p[i];
        for (unsigned b=0; b<8U; ++b) { c=(c>>1)^((c&1U)?0xedb88320U:0U); }
    }
    return ~c;
}
void GimbalControl_Invalidate(void)
{
    const uint32_t mask=__get_PRIMASK();
    __disable_irq();
    ++epoch;
    lease=used=ad_count=ad_next=have_sequence=0U;
    rocking=0U;
    head=tail=0U;
    __set_PRIMASK(mask);
}
void GimbalControl_Init(void)
{
    epoch=accepted=rejected=last_report=0U;
    reset_pending=0U;
    GimbalControl_Invalidate();
}
uint32_t GimbalControl_Epoch(void) { return epoch; }
uint32_t GimbalControl_Lease(void) { return lease; }
uint32_t GimbalControl_Accepted(void) { return accepted; }
uint32_t GimbalControl_Rejected(void) { return rejected; }
void GimbalControl_OnSessionReset(void) { reset_pending=1U; }
void GimbalControl_OnBytes(const uint8_t *data, uint32_t length)
{
    uint32_t mask;
    uint8_t next;
    if (!data || !length) { return; }
    mask=__get_PRIMASK(); __disable_irq();
    next=(uint8_t)((head+1U)%SLOTS);
    if (length>64U || next==tail) { reset_pending=1U; }
    else {
        memcpy(chunks[head].bytes,data,length);
        chunks[head].count=length;
        chunks[head].at=HAL_GetTick();
        head=next;
    }
    __set_PRIMASK(mask);
}
void GimbalControl_Advertise(uint32_t now)
{
    advertised[ad_next]=now;
    ad_next=(uint8_t)((ad_next+1U)%4U);
    if (ad_count<4U) { ++ad_count; }
}
static uint8_t valid(const uint8_t *p)
{
    return memcmp(p,"GC03",4U)==0 && p[25]==0U && p[24]<=5U && crc(p)==u32(p+28);
}
static void apply(const uint8_t *p)
{
    const uint32_t now=HAL_GetTick(), token=u32(p+16), anchor=u32(p+12), seq=u32(p+8);
    const uint16_t pan=u16(p+20), tilt=u16(p+22), duration=u16(p+26);
    const uint8_t motor=p[24]>=4U, starting=p[24]==1U || p[24]==4U;
    uint8_t known=0U, ok;
    for (unsigned i=0; i<ad_count; ++i) { if (advertised[i]==anchor) { known=1U; } }
    if (!valid(p) || u32(p+4)!=epoch || token==0U || !known || now-anchor>=LEASE_TIMEOUT_MS ||
        (have_sequence && (seq-sequence==0U || seq-sequence>=0x80000000U)) ||
        (starting ? lease!=0U || duration==0U || duration>(motor?5000U:10000U) :
         lease!=token || duration!=0U || motor!=rocking) ||
        (motor ? pan<1U || pan>20U || tilt<500U || tilt>2000U ||
            (!starting && (pan!=rock_speed || tilt!=rock_period)) :
            pan<60U || pan>120U || tilt<75U || tilt>105U)) {
        ++rejected; return;
    }
    /* Both APIs retain physical STOP/ARM/PG and non-extendable deadlines. */
    ok=motor ? (starting ? BabyCare_RequestMotor((int16_t)pan,duration) : 1U) :
        BabyCare_RequestGimbal(pan,tilt,starting?duration:10000U);
    if (!ok) {
        ++rejected; return;
    }
    if (motor && starting) {
        rocking=1U; rock_speed=(uint8_t)pan; rock_period=tilt; rock_phase=0U; rock_started=now;
    }
    lease=token; sequence=seq; have_sequence=1U; refreshed=now; ++accepted;
}
void GimbalControl_Task(void)
{
    Chunk local[SLOTS-1U];
    uint8_t frames[14][32], count=0U, nframes=0U, reset;
    uint32_t mask=__get_PRIMASK(), before;
    __disable_irq();
    reset=reset_pending; reset_pending=0U;
    while (tail!=head && count<SLOTS-1U) { local[count++]=chunks[tail]; tail=(uint8_t)((tail+1U)%SLOTS); }
    __set_PRIMASK(mask);
    if (reset || (lease && HAL_GetTick()-refreshed>=LEASE_TIMEOUT_MS)) {
        if (lease) { BabyCare_Stop(); } else { GimbalControl_Invalidate(); }
        count=0U;
    }
    for (unsigned n=0; n<count; ++n) {
        if (used && local[n].at-last_byte>200U) { used=0U; }
        for (unsigned i=0; i<local[n].count; ++i) {
            partial[used++]=local[n].bytes[i]; last_byte=local[n].at;
            while (used>=4U && memcmp(partial,"GC03",4U)!=0) {
                memmove(partial,partial+1,--used);
            }
            if (used==32U) {
                if (valid(partial)) {
                    /* A STOP anywhere in this batch wins before any PWM starts. */
                    if (partial[24]==0U) { BabyCare_Stop(); count=0U; nframes=0U; break; }
                    if (partial[24]==3U) {
                        const uint32_t token=u32(partial+16);
                        uint8_t owned=token!=0U && token==lease;
                        /* Cancel only this USB lease, never a newly voice-owned action.
                         * Also cancel a START queued earlier in this same USB batch. */
                        for (unsigned f=0; token!=0U && f<nframes; ++f) {
                            if ((frames[f][24]==1U || frames[f][24]==4U) && u32(frames[f]+16)==token && u32(frames[f]+4)==epoch) { owned=1U; }
                        }
                        if (owned) { BabyCare_Stop(); count=0U; nframes=0U; break; }
                        used=0U;
                        continue;
                    }
                    if (nframes<14U) { memcpy(frames[nframes++],partial,32U); }
                    used=0U;
                } else { ++rejected; memmove(partial,partial+1,31U); used=31U; }
            }
        }
    }
    for (unsigned i=0; i<nframes; ++i) {
        before=epoch; apply(frames[i]);
        /* Starting/switching owners invalidates older queued frames. */
        if (before!=epoch) { break; }
    }
    if (rocking && lease) {
        const uint8_t phase=(uint8_t)(((HAL_GetTick()-rock_started)/rock_period)&1U);
        if (phase!=rock_phase) {
            rock_phase=phase;
            /* Existing motor driver ramps through zero before reversing direction. */
            if (!BabyCare_RequestMotor(phase?-(int16_t)rock_speed:(int16_t)rock_speed,5000U)) { BabyCare_Stop(); }
        }
    }
    if (HAL_GetTick()-last_report>=100U) {
        char line[256];
        const BabyCare_StatusTypeDef *s=BabyCare_GetStatus();
        last_report=HAL_GetTick(); GimbalControl_Advertise(last_report);
        const int length=snprintf(line,sizeof(line),
            "gimbal_link v=3 epoch=%lu lease=%lu seq=%lu now_ms=%lu accepted=%lu rejected=%lu pan=%u tilt=%u remaining=%lu enabled=%u trip=%u rock=%u motor=%d\r\n",
            (unsigned long)epoch,(unsigned long)lease,(unsigned long)sequence,(unsigned long)last_report,
            (unsigned long)accepted,(unsigned long)rejected,s->pan_target_deg,s->tilt_target_deg,
            (unsigned long)s->motion_remaining_ms,s->interlock_enabled,s->interlock_trip,rocking,(int)s->motor_target_percent);
        if (length>0 && (unsigned)length<sizeof(line)) { (void)DebugTx_Write(line,(size_t)length,false); }
    }
}
