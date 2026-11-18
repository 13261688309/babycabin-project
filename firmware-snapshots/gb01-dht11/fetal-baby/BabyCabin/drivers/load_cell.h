#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "inc/app_types.h"

bool load_cell_init(void);
bool load_cell_read(uint16_t *weight_g, sensor_health_status_t *health);
