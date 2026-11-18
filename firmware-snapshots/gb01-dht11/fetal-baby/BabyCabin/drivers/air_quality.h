#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "inc/app_types.h"

bool air_quality_init(void);
bool air_quality_read(uint16_t *co2eq, uint16_t *tvoc, sensor_health_status_t *health);
bool air_quality_is_warmed_up(uint32_t elapsed_ms);
