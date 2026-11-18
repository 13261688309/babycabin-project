#include "test_gimbal_control.c"
#include "voice_control_trace.h"

static unsigned vchecks;
#define V(expr) do { assert(expr); ++vchecks; } while(0)

static void rx(Voice_HandleTypeDef *handle, uint8_t kind, uint8_t id)
{
    const uint8_t bytes[5]={0xAAU,0x55U,kind,id,0xFBU};
    for (unsigned i=0;i<5U;++i) { (void)Voice_UART_ParseByte(handle,bytes[i],NULL); }
}

static int trace_line(char *line)
{
    int found=0;
    DebugTx_Task();
    for (unsigned i=0;i<128U && tx_data!=NULL;++i) {
        if (tx_length>=10U && !memcmp(tx_data,"voice_cmd ",10U)) {
            V(tx_length<384U);
            memcpy(line,tx_data,tx_length); line[tx_length]='\0'; found++;
        }
        tx_complete();
    }
    return found;
}

int main(void)
{
    Voice_HandleTypeDef voice;
    Voice_RxStatsTypeDef stats;
    Voice_FrameTypeDef delivered;
    char line[384];
    previous_gc_regression_main();
    trace_line(line);
    Voice_Init(&voice,&huart2,NULL);
    tick=1000U; VoiceControlTrace_Init();
    tick=1099U; VoiceControlTrace_Task(&voice,0U,7U); V(trace_line(line)==0);
    tick=1100U; VoiceControlTrace_Task(&voice,0U,7U); V(trace_line(line)==1);
    V(strstr(line,"frames=0 type=0 id=0 last_ms=0 now_ms=1100")!=NULL);
    tick=1200U; rx(&voice,0U,0xA1U);
    VoiceControlTrace_Task(&voice,0U,7U); V(trace_line(line)==1);
    V(strstr(line,"frames=1 type=0 id=161 last_ms=1200 now_ms=1200")!=NULL);
    V(Voice_UART_GetLatestFrame(&voice,&delivered)==VOICE_OK);
    tick=1299U; VoiceControlTrace_Task(&voice,0U,7U); V(trace_line(line)==0);
    tick=1300U; VoiceControlTrace_Task(&voice,0U,7U); V(trace_line(line)==1);
    V(strstr(line,"frames=1 type=0 id=161 last_ms=1200 now_ms=1300")!=NULL);
    tick=1410U; rx(&voice,0xFFU,0xA2U);
    test_primask=1U;
    V(Voice_UART_GetRxStats(&voice,&stats)==VOICE_OK);
    V(test_primask==1U && stats.frames==2U && stats.last_type==0xFFU && stats.last_id==0xA2U);
    test_primask=0U;
    VoiceControlTrace_Task(&voice,0U,7U); V(trace_line(line)==1);
    V(strstr(line,"frames=2 type=255 id=162 last_ms=1410")!=NULL);
    tick=1510U; rx(&voice,0U,VOICE_CMD_STOP_1); rx(&voice,0U,0xA5U);
    VoiceControlTrace_Task(&voice,1U,8U); V(trace_line(line)==1);
    V(strstr(line,"frames=4 type=0 id=165 last_ms=1510")!=NULL);
    V(strstr(line,"stops=1 applied=1 alarms=0 errors=0 dropped=1 epoch=8")!=NULL);
    tick=UINT32_MAX-50U; VoiceControlTrace_Init();
    tick=49U; VoiceControlTrace_Task(&voice,1U,8U); V(trace_line(line)==1);
    ready(20000U);
    for (unsigned id=0xA1U;id<=0xA8U;++id) {
        app_voice((uint8_t)id); BabyCare_Task(); V(stopped());
    }
    printf("PASS associated voice snapshot: %u assertions; atomic metadata, repeat, type, STOP/drop, tick wrap; extended IDs never drive firmware directly\n",vchecks);
    return 0;
}
