#include "state_machine.h"

#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "inc/app_config.h"

void state_machine_init(state_machine_t *machine)
{
    if (machine == NULL)
    {
        return;
    }

    machine->current_status = BABY_STATUS_DATA_UNAVAILABLE;
    machine->last_change_ms = 0U;
    machine->cry_start_ms = 0U;
    machine->quiet_start_ms = 0U;
    machine->cry_tracking = false;
    machine->quiet_tracking = false;
}

static bool is_crying_now(const system_snapshot_t *snapshot)
{
    return snapshot->microphone_health == SENSOR_HEALTH_OK &&
        snapshot->mpu6050_health == SENSOR_HEALTH_OK &&
        isfinite(snapshot->sound_level) && snapshot->sound_level <= 1.0f &&
        snapshot->sound_level > CRY_SOUND_THRESHOLD && snapshot->motion_active;
}

static bool failed(sensor_health_status_t health)
{
    return health == SENSOR_HEALTH_STALE || health == SENSOR_HEALTH_READ_FAILED;
}

baby_status_t state_machine_update(state_machine_t *machine, const system_snapshot_t *snapshot)
{
    baby_status_t next_status;
    bool crying;
    bool sound_valid;
    bool ambient_valid;
    bool quiet;

    if (machine == NULL || snapshot == NULL)
    {
        return BABY_STATUS_DATA_UNAVAILABLE;
    }

    crying = is_crying_now(snapshot);
    sound_valid = snapshot->microphone_health == SENSOR_HEALTH_OK &&
        isfinite(snapshot->sound_level) && snapshot->sound_level >= 0.0f &&
        snapshot->sound_level <= 1.0f;
    ambient_valid = snapshot->dht11_health == SENSOR_HEALTH_OK &&
        isfinite(snapshot->temperature_c) && isfinite(snapshot->humidity_rh);
    quiet = sound_valid && snapshot->mpu6050_health == SENSOR_HEALTH_OK &&
        snapshot->sound_level < SLEEP_SOUND_THRESHOLD && !snapshot->motion_active;

    /* Track cry duration */
    if (crying)
    {
        if (!machine->cry_tracking)
        {
            machine->cry_start_ms = snapshot->timestamp_ms;
        }
    }
    else
    {
        if (quiet)
        {
            if (!machine->quiet_tracking)
            {
                machine->quiet_start_ms = snapshot->timestamp_ms;
            }
        }
    }
    machine->cry_tracking = crying;
    machine->quiet_tracking = quiet;

    /* Priority-ordered state evaluation */
    if (snapshot->wet_probe_health == SENSOR_HEALTH_OK && snapshot->wet_detected)
    {
        next_status = BABY_STATUS_WET;
    }
    else if (snapshot->ir_temp_health == SENSOR_HEALTH_OK &&
             isfinite(snapshot->ir_temp_object_c) && snapshot->ir_temp_object_c >= TEMP_ALERT_C)
    {
        next_status = BABY_STATUS_TEMP_ALERT;
    }
    else if (snapshot->presence_health == SENSOR_HEALTH_OK && !snapshot->baby_in_bed)
    {
        next_status = BABY_STATUS_PRESENCE_ALERT;
    }
    else if (crying &&
             (snapshot->timestamp_ms - machine->cry_start_ms) >= CRY_SUSTAIN_MS)
    {
        next_status = BABY_STATUS_ABNORMAL_CRYING;
    }
    else if ((snapshot->air_quality_health == SENSOR_HEALTH_OK &&
              (snapshot->air_quality_voc >= AIR_QUALITY_VOC_WARN ||
               snapshot->air_quality_co2 >= AIR_QUALITY_CO2_WARN)) ||
             (snapshot->ambient_light_health == SENSOR_HEALTH_OK &&
              snapshot->ambient_light_lux >= AMBIENT_LIGHT_HIGH_LUX) ||
             (ambient_valid && (snapshot->temperature_c < AMBIENT_TEMP_WARN_LOW_C ||
                                snapshot->temperature_c > AMBIENT_TEMP_WARN_HIGH_C)))
    {
        next_status = BABY_STATUS_ENV_WARNING;
    }
    else if (failed(snapshot->dht11_health) || failed(snapshot->microphone_health) ||
             failed(snapshot->mpu6050_health) || failed(snapshot->presence_health) ||
             failed(snapshot->wet_probe_health) || failed(snapshot->camera_health) ||
             failed(snapshot->battery_health) || failed(snapshot->ambient_light_health) ||
             failed(snapshot->air_quality_health) || failed(snapshot->ir_temp_health) ||
             failed(snapshot->load_cell_health) ||
             (snapshot->dht11_health == SENSOR_HEALTH_OK && !ambient_valid) ||
             (snapshot->microphone_health == SENSOR_HEALTH_OK && !sound_valid))
    {
        next_status = BABY_STATUS_SENSOR_FAULT;
    }
    else if (!ambient_valid || !sound_valid || snapshot->mpu6050_health != SENSOR_HEALTH_OK ||
             snapshot->presence_health != SENSOR_HEALTH_OK)
    {
        next_status = BABY_STATUS_DATA_UNAVAILABLE;
    }
    else if (quiet &&
             (snapshot->timestamp_ms - machine->quiet_start_ms) >= SLEEP_QUIET_MS)
    {
        next_status = BABY_STATUS_SLEEPY;
    }
    else if (snapshot->ir_temp_health == SENSOR_HEALTH_OK &&
             isfinite(snapshot->ir_temp_object_c) && snapshot->ir_temp_object_c >= TEMP_WARN_C)
    {
        next_status = BABY_STATUS_UNCOMFORTABLE;
    }
    else
    {
        next_status = BABY_STATUS_NORMAL;
    }

    if (machine->current_status != next_status)
    {
        machine->current_status = next_status;
        machine->last_change_ms = snapshot->timestamp_ms;
    }

    return machine->current_status;
}
