#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "inc/app_types.h"

typedef struct
{
    float temperature_c;
    float humidity_rh;
    bool valid;
} dht11_reading_t;

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    bool motion_active;
    bool valid;
} mpu6050_reading_t;

typedef struct
{
    float sound_level;
    uint16_t raw_min;
    uint16_t raw_max;
    uint16_t raw_peak_to_peak;
    bool valid;
} microphone_reading_t;

typedef struct
{
    bool wet_detected;
    uint16_t raw_adc;
    bool valid;
} wet_probe_reading_t;

typedef struct
{
    uint16_t millivolts;
    uint8_t percent;
    bool valid;
} battery_reading_t;

bool sensor_adapters_init(void);
bool sensor_adapters_read_dht11(dht11_reading_t *reading);
bool sensor_adapters_read_mpu6050(mpu6050_reading_t *reading);
bool sensor_adapters_read_microphone(microphone_reading_t *reading);
bool sensor_adapters_read_wet_probe(wet_probe_reading_t *reading);
bool sensor_adapters_read_battery(battery_reading_t *reading);
bool sensor_adapters_read_wet_probe_digital(bool *wet_detected);
bool sensor_adapters_read_ambient_light(uint16_t *lux, sensor_health_status_t *health);
bool sensor_adapters_read_air_quality(uint16_t *co2eq, uint16_t *tvoc, sensor_health_status_t *health);
bool sensor_adapters_read_presence(bool *baby_in_bed, uint8_t *distance_cm, sensor_health_status_t *health);
