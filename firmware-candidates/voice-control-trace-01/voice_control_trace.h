#ifndef VOICE_CONTROL_TRACE_H
#define VOICE_CONTROL_TRACE_H
#include "voice_interaction.h"
void VoiceControlTrace_Init(void);
void VoiceControlTrace_Task(const Voice_HandleTypeDef *voice, uint32_t applied_stops, uint32_t epoch);
#endif
