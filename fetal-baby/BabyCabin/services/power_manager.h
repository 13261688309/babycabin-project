#pragma once

#include <stdbool.h>

#include "inc/app_types.h"

void power_manager_init(void);
bool power_manager_should_sleep(const system_snapshot_t *snapshot);

