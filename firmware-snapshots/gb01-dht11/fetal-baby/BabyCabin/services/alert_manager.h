#pragma once

#include <stdbool.h>

#include "inc/app_types.h"

void alert_manager_init(void);
bool alert_manager_handle_status(baby_status_t status, const system_snapshot_t *snapshot);
