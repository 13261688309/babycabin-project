#pragma once

#include "inc/app_types.h"

typedef struct
{
    baby_status_t current_status;
    uint32_t last_change_ms;
    uint32_t cry_start_ms;
    uint32_t quiet_start_ms;
    bool cry_tracking;
    bool quiet_tracking;
} state_machine_t;

void state_machine_init(state_machine_t *machine);
baby_status_t state_machine_update(state_machine_t *machine, const system_snapshot_t *snapshot);
