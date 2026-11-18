#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "inc/app_types.h"

typedef enum
{
    VOICE_EVENT_NONE = 0,
    VOICE_EVENT_WAKEUP,
    VOICE_EVENT_COMMAND,
    VOICE_EVENT_UNKNOWN
} voice_event_type_t;

typedef struct
{
    voice_event_type_t type;
    uint16_t command_id;
    bool valid;
} voice_event_t;

bool voice_module_init(void);
bool voice_module_play_command(device_command_t command);
bool voice_module_poll_event(voice_event_t *event);
bool voice_module_event_to_command(const voice_event_t *event, device_command_t *command, uint32_t *duration_ms);
