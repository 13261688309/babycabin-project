#pragma once

#include <stdbool.h>

#include "inc/app_types.h"

bool sensors_init(void);
bool sensors_read(system_snapshot_t *snapshot);

