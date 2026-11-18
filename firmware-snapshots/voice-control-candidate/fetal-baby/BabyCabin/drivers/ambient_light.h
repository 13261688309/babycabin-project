#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "inc/app_types.h"

bool ambient_light_init(void);
bool ambient_light_read(uint16_t *lux, sensor_health_status_t *health);
