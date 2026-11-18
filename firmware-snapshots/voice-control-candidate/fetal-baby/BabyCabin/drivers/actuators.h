#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "inc/app_types.h"

bool actuators_init(void);
void actuators_stop_all(void);
void actuators_execute(device_command_t command, uint32_t duration_ms);
void actuators_task(void);
