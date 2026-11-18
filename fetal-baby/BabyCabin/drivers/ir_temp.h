#pragma once

#include <stdbool.h>

#include "inc/app_types.h"

bool ir_temp_init(void);
bool ir_temp_read(float *object_c, float *ambient_c, sensor_health_status_t *health);
