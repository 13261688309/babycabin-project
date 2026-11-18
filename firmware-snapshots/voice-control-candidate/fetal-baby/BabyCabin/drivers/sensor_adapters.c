#include "sensor_adapters.h"

#include <stddef.h>

#include "bsp/bsp.h"
#include "drivers/air_quality.h"
#include "drivers/ambient_light.h"
#include "drivers/presence_detector.h"
#include "inc/app_config.h"

#define MPU6050_REG_PWR_MGMT_1 0x6BU
#define MPU6050_REG_ACCEL_CONFIG 0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define DHT11_BIT_COUNT 40U
#define DHT11_DATA_BYTES 5U

#if SENSOR_ENABLE_MICROPHONE_ADC
static uint16_t g_microphone_samples[MICROPHONE_WINDOW_SAMPLES];
#endif

#if SENSOR_ENABLE_MPU6050
static int16_t g_mpu_last_ax;
static int16_t g_mpu_last_ay;
static int16_t g_mpu_last_az;
static bool g_mpu_has_last;
static bool g_mpu_ready;

static uint32_t abs_i16_delta(int16_t current, int16_t previous)
{
    int32_t delta = (int32_t)current - (int32_t)previous;
    return (uint32_t)(delta < 0 ? -delta : delta);
}
#endif

#if SENSOR_ENABLE_DHT11
static bool dht11_wait_for_level(bool expected_level, uint16_t timeout_us)
{
    bool level = false;

    while (timeout_us > 0U)
    {
        if (!bsp_dht11_read(&level))
        {
            return false;
        }

        if (level == expected_level)
        {
            return true;
        }

        bsp_delay_us(1U);
        timeout_us--;
    }

    return false;
}

static bool dht11_read_bytes(uint8_t *data)
{
    uint8_t bit_index;
    uint8_t byte_index;
    uint8_t checksum;
    bool level = false;

    if (data == NULL)
    {
        return false;
    }

    for (byte_index = 0U; byte_index < DHT11_DATA_BYTES; byte_index++)
    {
        data[byte_index] = 0U;
    }

    bsp_dht11_release();
    bsp_delay_ms(1U);
    bsp_dht11_drive_low();
    bsp_delay_ms(18U);
    bsp_dht11_release();
    bsp_delay_us(30U);

    if (!dht11_wait_for_level(false, 100U) ||
        !dht11_wait_for_level(true, 100U) ||
        !dht11_wait_for_level(false, 100U))
    {
        return false;
    }

    for (bit_index = 0U; bit_index < DHT11_BIT_COUNT; bit_index++)
    {
        if (!dht11_wait_for_level(true, 80U))
        {
            return false;
        }

        bsp_delay_us(40U);
        if (!bsp_dht11_read(&level))
        {
            return false;
        }

        data[bit_index / 8U] <<= 1U;
        if (level)
        {
            data[bit_index / 8U] |= 1U;
            if (!dht11_wait_for_level(false, 80U))
            {
                return false;
            }
        }
    }

    checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    return checksum == data[4];
}
#endif

#if SENSOR_ENABLE_MPU6050
static int16_t i16_from_be(uint8_t high, uint8_t low)
{
    return (int16_t)((uint16_t)((uint16_t)high << 8U) | low);
}
#endif

#if SENSOR_ENABLE_MICROPHONE_ADC
static float normalize_adc_span(uint16_t peak_to_peak, uint16_t noise_floor)
{
    uint32_t adjusted;

    if (peak_to_peak <= noise_floor)
    {
        return 0.0f;
    }

    adjusted = (uint32_t)peak_to_peak - (uint32_t)noise_floor;
    if (adjusted >= ADC_MAX_COUNTS)
    {
        return 1.0f;
    }

    return (float)adjusted / (float)ADC_MAX_COUNTS;
}
#endif

bool sensor_adapters_init(void)
{
#if SENSOR_ENABLE_MPU6050
    const uint8_t wake = 0x00U;
    const uint8_t accel_2g = 0x00U;
    g_mpu_ready = false;
    g_mpu_has_last = false;
    if (!bsp_i2c_mem_write(BSP_I2C_DEVICE_MPU6050, MPU6050_REG_PWR_MGMT_1, &wake, 1U) ||
        !bsp_i2c_mem_write(BSP_I2C_DEVICE_MPU6050, MPU6050_REG_ACCEL_CONFIG, &accel_2g, 1U)) {
        return false;
    }
    g_mpu_ready = true;
#endif

    return true;
}

bool sensor_adapters_read_dht11(dht11_reading_t *reading)
{
#if SENSOR_ENABLE_DHT11
    uint8_t raw[DHT11_DATA_BYTES] = {0};
#endif

    if (reading == NULL)
    {
        return false;
    }

    reading->temperature_c = 0.0f;
    reading->humidity_rh = 0.0f;
    reading->valid = false;

#if SENSOR_ENABLE_DHT11
    if (!dht11_read_bytes(raw))
    {
        return false;
    }

    reading->humidity_rh = (float)raw[0] + ((float)raw[1] / 10.0f);
    reading->temperature_c = (float)raw[2] + ((float)raw[3] / 10.0f);
    reading->valid = true;
    return true;
#else
    return false;
#endif
}

bool sensor_adapters_read_mpu6050(mpu6050_reading_t *reading)
{
#if SENSOR_ENABLE_MPU6050
    uint8_t raw[6] = {0};
#endif

    if (reading == NULL)
    {
        return false;
    }

    reading->ax = 0;
    reading->ay = 0;
    reading->az = 0;
    reading->motion_active = false;
    reading->valid = false;

#if SENSOR_ENABLE_MPU6050
    if (!g_mpu_ready) {
        return false;
    }
    if (!bsp_i2c_mem_read(BSP_I2C_DEVICE_MPU6050, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw)))
    {
        g_mpu_has_last = false;
        return false;
    }

    reading->ax = i16_from_be(raw[0], raw[1]);
    reading->ay = i16_from_be(raw[2], raw[3]);
    reading->az = i16_from_be(raw[4], raw[5]);

    if (g_mpu_has_last)
    {
        reading->motion_active =
            abs_i16_delta(reading->ax, g_mpu_last_ax) > MOTION_ACCEL_DELTA_THRESHOLD ||
            abs_i16_delta(reading->ay, g_mpu_last_ay) > MOTION_ACCEL_DELTA_THRESHOLD ||
            abs_i16_delta(reading->az, g_mpu_last_az) > MOTION_ACCEL_DELTA_THRESHOLD;
    }

    g_mpu_last_ax = reading->ax;
    g_mpu_last_ay = reading->ay;
    g_mpu_last_az = reading->az;
    g_mpu_has_last = true;
    reading->valid = true;
    return true;
#else
    return false;
#endif
}

bool sensor_adapters_read_microphone(microphone_reading_t *reading)
{
#if SENSOR_ENABLE_MICROPHONE_ADC
    uint16_t min_sample = ADC_MAX_COUNTS;
    uint16_t max_sample = 0U;
    size_t index;
#endif

    if (reading == NULL)
    {
        return false;
    }

    reading->sound_level = 0.0f;
    reading->raw_min = 0U;
    reading->raw_max = 0U;
    reading->raw_peak_to_peak = 0U;
    reading->valid = false;

#if SENSOR_ENABLE_MICROPHONE_ADC
    if (!bsp_adc_read_window(BSP_ADC_MICROPHONE, g_microphone_samples, MICROPHONE_WINDOW_SAMPLES))
    {
        return false;
    }

    for (index = 0U; index < MICROPHONE_WINDOW_SAMPLES; index++)
    {
        if (g_microphone_samples[index] < min_sample)
        {
            min_sample = g_microphone_samples[index];
        }

        if (g_microphone_samples[index] > max_sample)
        {
            max_sample = g_microphone_samples[index];
        }
    }

    reading->raw_min = min_sample;
    reading->raw_max = max_sample;
    reading->raw_peak_to_peak = (uint16_t)(max_sample - min_sample);
    reading->sound_level = normalize_adc_span(reading->raw_peak_to_peak, MICROPHONE_NOISE_FLOOR_ADC);
    reading->valid = true;
    return true;
#else
    return false;
#endif
}

bool sensor_adapters_read_wet_probe(wet_probe_reading_t *reading)
{
#if SENSOR_ENABLE_WET_PROBE
    uint16_t raw = 0U;
    bool wet_digital = false;
    static uint8_t wet_count;
    static uint8_t dry_count;
    static bool stable_wet;
    bool current_wet = false;
#endif

    if (reading == NULL)
    {
        return false;
    }

    reading->wet_detected = false;
    reading->raw_adc = 0U;
    reading->valid = false;

#if SENSOR_ENABLE_WET_PROBE
    if (!bsp_adc_read(BSP_ADC_WET_PROBE, &raw))
    {
        if (!bsp_gpio_read(BSP_GPIO_WET_DIGITAL, &wet_digital))
        {
            return false;
        }
        current_wet = wet_digital;
        reading->raw_adc = wet_digital ? WET_ADC_THRESHOLD : 0U;
    }
    else
    {
        reading->raw_adc = raw;
        current_wet = reading->raw_adc >= WET_ADC_THRESHOLD;
    }

    if (current_wet)
    {
        if (wet_count < WET_DEBOUNCE_SAMPLES)
        {
            wet_count++;
        }
        dry_count = 0U;
    }
    else
    {
        if (dry_count < WET_DEBOUNCE_SAMPLES)
        {
            dry_count++;
        }
        wet_count = 0U;
    }

    if (wet_count >= WET_DEBOUNCE_SAMPLES)
    {
        stable_wet = true;
    }
    else if (dry_count >= WET_DEBOUNCE_SAMPLES)
    {
        stable_wet = false;
    }

    reading->wet_detected = stable_wet;
    reading->valid = true;
    return true;
#else
    return false;
#endif
}

bool sensor_adapters_read_battery(battery_reading_t *reading)
{
#if SENSOR_ENABLE_BATTERY_ADC
    uint16_t raw = 0U;
    static uint16_t filtered_mv;
    static bool has_filtered_mv;
    uint16_t measured_mv;
#endif

    if (reading == NULL)
    {
        return false;
    }

    reading->millivolts = 0U;
    reading->percent = 0U;
    reading->valid = false;

#if SENSOR_ENABLE_BATTERY_ADC
    if (!bsp_adc_read(BSP_ADC_BATTERY, &raw))
    {
        return false;
    }

    measured_mv = (uint16_t)(((uint32_t)raw * BATTERY_ADC_FULL_SCALE_MV) / ADC_MAX_COUNTS);
    if (!has_filtered_mv)
    {
        filtered_mv = measured_mv;
        has_filtered_mv = true;
    }
    else
    {
        filtered_mv = (uint16_t)((((uint32_t)filtered_mv * ((1U << BATTERY_FILTER_SHIFT) - 1U)) +
                                  measured_mv) >>
                                 BATTERY_FILTER_SHIFT);
    }

    reading->millivolts = filtered_mv;
    if (reading->millivolts <= BATTERY_ADC_EMPTY_MV)
    {
        reading->percent = 0U;
    }
    else if (reading->millivolts >= BATTERY_ADC_FULL_MV)
    {
        reading->percent = 100U;
    }
    else
    {
        reading->percent = (uint8_t)(((uint32_t)(reading->millivolts - BATTERY_ADC_EMPTY_MV) * 100U) /
                                     (BATTERY_ADC_FULL_MV - BATTERY_ADC_EMPTY_MV));
    }

    reading->valid = true;
    return true;
#else
    return false;
#endif
}

bool sensor_adapters_read_wet_probe_digital(bool *wet_detected)
{
    if (wet_detected == NULL)
    {
        return false;
    }

    return bsp_gpio_read(BSP_GPIO_WET_DIGITAL, wet_detected);
}

bool sensor_adapters_read_ambient_light(uint16_t *lux, sensor_health_status_t *health)
{
    return ambient_light_read(lux, health);
}

bool sensor_adapters_read_air_quality(uint16_t *co2eq, uint16_t *tvoc, sensor_health_status_t *health)
{
    return air_quality_read(co2eq, tvoc, health);
}

bool sensor_adapters_read_presence(bool *baby_in_bed, uint8_t *distance_cm, sensor_health_status_t *health)
{
    return presence_detector_poll(baby_in_bed, distance_cm, health);
}
