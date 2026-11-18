#include "voice_control_trace.h"
#include "debug_uart_tx.h"
#include <stdio.h>

static uint32_t last_report;

void VoiceControlTrace_Init(void)
{
    last_report=HAL_GetTick();
}

void VoiceControlTrace_Task(const Voice_HandleTypeDef *voice, uint32_t applied_stops, uint32_t epoch)
{
    Voice_RxStatsTypeDef rx;
    char line[384];
    const uint32_t now=HAL_GetTick();
    if (now-last_report<100U) { return; }
    last_report=now;
    /* GetRxStats copies count, timestamp and frame fields in one IRQ-masked snapshot. */
    if (Voice_UART_GetRxStats(voice,&rx)!=VOICE_OK) { return; }
    const int length=snprintf(line,sizeof(line),
        "voice_cmd v=1 frames=%lu type=%u id=%u last_ms=%lu now_ms=%lu stops=%lu applied=%lu alarms=%lu errors=%lu dropped=%lu epoch=%lu\r\n",
        (unsigned long)rx.frames,(unsigned)rx.last_type,(unsigned)rx.last_id,
        (unsigned long)rx.last_frame_ms,(unsigned long)now,(unsigned long)rx.stop_frames,
        (unsigned long)applied_stops,(unsigned long)rx.alarm_frames,
        (unsigned long)rx.uart_errors,(unsigned long)rx.dropped_frames,(unsigned long)epoch);
    if (length>0 && (unsigned)length<sizeof(line)) {
        (void)DebugTx_Write(line,(size_t)length,false);
    }
}
