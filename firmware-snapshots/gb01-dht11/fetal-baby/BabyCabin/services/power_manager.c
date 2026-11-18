#include "power_manager.h"

#include <stddef.h>

#include "inc/app_config.h"

void power_manager_init(void)
{
}

bool power_manager_should_sleep(const system_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return false;
    }

    return snapshot->mpu6050_health == SENSOR_HEALTH_OK &&
        snapshot->microphone_health == SENSOR_HEALTH_OK &&
        !snapshot->motion_active && snapshot->sound_level >= 0.0f &&
        snapshot->sound_level < SLEEP_SOUND_THRESHOLD;
}
