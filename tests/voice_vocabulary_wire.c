/* Replay workbook bytes through the frozen driver ISR and its real trace formatter. */
#include "test_gimbal_control.c"
#include "voice_control_trace.h"

static void emit_trace(void)
{
    DebugTx_Task();
    for (unsigned i=0U;i<128U && tx_data!=NULL;++i) {
        if (tx_length>=10U && !memcmp(tx_data,"voice_cmd ",10U)) {
            printf("WIRE %.*s",(int)tx_length,tx_data);
        }
        tx_complete();
    }
}

int main(void)
{
    Voice_HandleTypeDef handle;
    Voice_FrameTypeDef delivered;
    unsigned long at;
    unsigned count,byte;
    ready(1000U);
    assert(DebugTx_Init(&huart1));
    tx_data=NULL;
    tx_length=0U;
    Voice_Init(&handle,&huart2,NULL);
    assert(Voice_UART_StartReceiveIT(&handle)==VOICE_OK);
    tick=1000U;
    VoiceControlTrace_Init();
    while (scanf("%lu %u",&at,&count)==2) {
        assert(at<=UINT32_MAX && count<=64U);
        tick=(uint32_t)at;
        for (unsigned i=0U;i<count;++i) {
            assert(scanf("%x",&byte)==1 && byte<=255U);
            *huart2.rx=(uint8_t)byte;
            (void)Voice_UART_RxCpltCallback(&handle,&huart2);
            assert(handle.restart_needed==0U);
        }
        VoiceControlTrace_Task(&handle,0U,7U);
        emit_trace();
        while (Voice_UART_GetLatestFrame(&handle,&delivered)==VOICE_OK) {}
        assert(stopped());
    }
    assert(!ferror(stdin));
    return 0;
}
