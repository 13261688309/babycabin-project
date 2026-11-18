#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "inc/app_types.h"

bool presence_detector_init(void);
bool presence_detector_poll(bool *baby_in_bed, uint8_t *distance_cm, sensor_health_status_t *health);
