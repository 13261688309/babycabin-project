#ifndef BABY_CABIN_APP_TYPES_H
#define BABY_CABIN_APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BABY_STATUS_NORMAL = 0,
    BABY_STATUS_SLEEPY,
    BABY_STATUS_HUNGRY,
    BABY_STATUS_WET,
    BABY_STATUS_UNCOMFORTABLE,
    BABY_STATUS_ABNORMAL_CRYING,
    BABY_STATUS_TEMP_ALERT,
    BABY_STATUS_ENV_WARNING,
    BABY_STATUS_PRESENCE_ALERT,
    BABY_STATUS_DATA_UNAVAILABLE,
    BABY_STATUS_SENSOR_FAULT,
    BABY_STATUS_COUNT
} baby_status_t;

typedef enum
{
    DEVICE_CMD_NONE = 0,
    DEVICE_CMD_START_WHITE_NOISE,
    DEVICE_CMD_STOP_WHITE_NOISE,
    DEVICE_CMD_START_ROCKING,
    DEVICE_CMD_STOP_ROCKING,
    DEVICE_CMD_START_VIBRATION,
    DEVICE_CMD_STOP_VIBRATION,
    DEVICE_CMD_PLAY_MOTHER_VOICE
} device_command_t;

typedef enum
{
    SENSOR_HEALTH_DISABLED = 0,
    SENSOR_HEALTH_OK,
    SENSOR_HEALTH_STALE,
    SENSOR_HEALTH_READ_FAILED
} sensor_health_status_t;

typedef struct
{
    uint32_t timestamp_ms;
    baby_status_t status;
    bool esp32_link_online;
    float temperature_c;
    float humidity_rh;
    float sound_level;
    bool motion_active;
    bool wet_detected;
    uint8_t battery_percent;
    bool camera_detected;
    uint16_t camera_x;
    uint16_t camera_y;
    uint16_t camera_area;
    int16_t camera_id;
    uint16_t ambient_light_lux;
    uint16_t air_quality_voc;
    uint16_t air_quality_co2;
    bool presence_detected;
    uint8_t presence_distance_cm;
    bool baby_in_bed;
    float ir_temp_object_c;
    float ir_temp_ambient_c;
    uint16_t baby_weight_g;
    sensor_health_status_t dht11_health;
    sensor_health_status_t mpu6050_health;
    sensor_health_status_t microphone_health;
    sensor_health_status_t wet_probe_health;
    sensor_health_status_t battery_health;
    sensor_health_status_t camera_health;
    sensor_health_status_t ambient_light_health;
    sensor_health_status_t air_quality_health;
    sensor_health_status_t presence_health;
    sensor_health_status_t ir_temp_health;
    sensor_health_status_t load_cell_health;
} system_snapshot_t;

#endif
