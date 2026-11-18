#include "sensors.h"

#include <stddef.h>
#include <math.h>

#include "bsp/bsp.h"
#include "drivers/air_quality.h"
#include "drivers/ambient_light.h"
#include "drivers/camera_module.h"
#include "drivers/presence_detector.h"
#include "drivers/sensor_adapters.h"
#include "inc/app_config.h"

static dht11_reading_t g_dht11_cache;
#if SENSOR_ENABLE_DHT11
static uint32_t g_last_dht11_read_ms;
static sensor_health_status_t g_dht11_health;
#endif

bool sensors_init(void)
{
    g_dht11_cache = (dht11_reading_t){0};
#if SENSOR_ENABLE_DHT11
    g_last_dht11_read_ms = bsp_millis();
    g_dht11_health = SENSOR_HEALTH_STALE;
#endif

    (void)sensor_adapters_init();
    (void)camera_module_init();
    (void)ambient_light_init();
    (void)air_quality_init();
    (void)presence_detector_init();
    return true;
}

bool sensors_read(system_snapshot_t *snapshot)
{
    uint32_t now_ms;
    bool microphone_ok;
    bool motion_ok;
    bool wet_probe_ok;
    bool battery_ok;
    bool camera_ok;
    bool ambient_ok;
    bool air_ok;
    bool presence_ok;
    uint16_t ambient_lux;
    uint16_t air_co2;
    uint16_t air_voc;
    bool baby_in_bed_local;
    uint8_t presence_dist;
    sensor_health_status_t ambient_health;
    sensor_health_status_t air_health;
    sensor_health_status_t presence_health;
    mpu6050_reading_t motion;
    camera_detection_t camera;
    microphone_reading_t microphone;
    wet_probe_reading_t wet_probe;
    battery_reading_t battery;

    if (snapshot == NULL)
    {
        return false;
    }

    now_ms = bsp_millis();

#if SENSOR_ENABLE_DHT11
    if ((now_ms - g_last_dht11_read_ms) >= DHT11_MIN_READ_PERIOD_MS)
    {
        dht11_reading_t candidate = {0};
        /* Failed attempts need the same bus cooldown as successful readings. */
        g_last_dht11_read_ms = now_ms;
        if (sensor_adapters_read_dht11(&candidate) && candidate.valid &&
            isfinite(candidate.temperature_c) && isfinite(candidate.humidity_rh) &&
            candidate.humidity_rh >= 0.0f && candidate.humidity_rh <= 100.0f)
        {
            g_dht11_cache = candidate;
            g_dht11_health = SENSOR_HEALTH_OK;
        }
        else
        {
            g_dht11_cache.valid = false;
            g_dht11_health = SENSOR_HEALTH_READ_FAILED;
        }
    }
#endif

    microphone_ok = sensor_adapters_read_microphone(&microphone);
    motion_ok = sensor_adapters_read_mpu6050(&motion);
    wet_probe_ok = sensor_adapters_read_wet_probe(&wet_probe);
    battery_ok = sensor_adapters_read_battery(&battery);
    camera_ok = camera_module_read_detection(&camera);
    ambient_ok = sensor_adapters_read_ambient_light(&ambient_lux, &ambient_health);
    air_ok = sensor_adapters_read_air_quality(&air_co2, &air_voc, &air_health);
    presence_ok = sensor_adapters_read_presence(&baby_in_bed_local, &presence_dist, &presence_health);

    snapshot->timestamp_ms = now_ms;
    snapshot->temperature_c = g_dht11_cache.temperature_c;
    snapshot->humidity_rh = g_dht11_cache.humidity_rh;
    snapshot->sound_level = microphone_ok ? microphone.sound_level : 0.0f;
    snapshot->motion_active = motion_ok && motion.motion_active;
    snapshot->wet_detected = wet_probe_ok && wet_probe.wet_detected;
    snapshot->battery_percent = battery_ok ? battery.percent : 0U;
    snapshot->camera_detected = camera_ok && camera.valid;
    snapshot->camera_x = camera.middle_x;
    snapshot->camera_y = camera.middle_y;
    snapshot->camera_area = camera.area;
    snapshot->camera_id = camera.id;
    snapshot->ambient_light_lux = ambient_ok ? ambient_lux : 0U;
    snapshot->air_quality_co2 = air_ok ? air_co2 : 0U;
    snapshot->air_quality_voc = air_ok ? air_voc : 0U;
    snapshot->presence_detected = presence_ok && baby_in_bed_local;
    snapshot->presence_distance_cm = presence_ok ? presence_dist : (uint8_t)0U;
    snapshot->baby_in_bed = presence_ok ? baby_in_bed_local : true;
    snapshot->ir_temp_object_c = 0.0f;
    snapshot->ir_temp_ambient_c = 0.0f;
    snapshot->baby_weight_g = 0U;

#if SENSOR_ENABLE_DHT11
    snapshot->dht11_health = g_dht11_health;
#else
    snapshot->dht11_health = SENSOR_HEALTH_DISABLED;
#endif

#if SENSOR_ENABLE_MPU6050
    snapshot->mpu6050_health = motion_ok ? SENSOR_HEALTH_OK : SENSOR_HEALTH_READ_FAILED;
#else
    snapshot->mpu6050_health = SENSOR_HEALTH_DISABLED;
#endif

#if SENSOR_ENABLE_MICROPHONE_ADC
    snapshot->microphone_health = microphone_ok ? SENSOR_HEALTH_OK : SENSOR_HEALTH_READ_FAILED;
#else
    snapshot->microphone_health = SENSOR_HEALTH_DISABLED;
#endif

#if SENSOR_ENABLE_WET_PROBE
    snapshot->wet_probe_health = wet_probe_ok ? SENSOR_HEALTH_OK : SENSOR_HEALTH_READ_FAILED;
#else
    snapshot->wet_probe_health = SENSOR_HEALTH_DISABLED;
#endif

#if SENSOR_ENABLE_BATTERY_ADC
    snapshot->battery_health = battery_ok ? SENSOR_HEALTH_OK : SENSOR_HEALTH_READ_FAILED;
#else
    snapshot->battery_health = SENSOR_HEALTH_DISABLED;
#endif

#if SENSOR_ENABLE_ESP32_CAMERA
    snapshot->camera_health = camera_ok ? SENSOR_HEALTH_OK : SENSOR_HEALTH_READ_FAILED;
#else
    snapshot->camera_health = SENSOR_HEALTH_DISABLED;
#endif

#if SENSOR_ENABLE_BH1750
    snapshot->ambient_light_health = ambient_health;
#else
    snapshot->ambient_light_health = SENSOR_HEALTH_DISABLED;
#endif

#if SENSOR_ENABLE_SGP30
    snapshot->air_quality_health = air_health;
#else
    snapshot->air_quality_health = SENSOR_HEALTH_DISABLED;
#endif

#if SENSOR_ENABLE_LD2410
    snapshot->presence_health = presence_health;
#else
    snapshot->presence_health = SENSOR_HEALTH_DISABLED;
#endif

    snapshot->ir_temp_health = SENSOR_HEALTH_DISABLED;
    snapshot->load_cell_health = SENSOR_HEALTH_DISABLED;

    return true;
}
