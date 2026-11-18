#include "air_quality.h"

#include "bsp/bsp.h"
#include "inc/app_config.h"

#if SENSOR_ENABLE_SGP30
static uint32_t g_init_ms;
#endif
static bool g_initialized;

#if SENSOR_ENABLE_SGP30
static uint8_t sgp30_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFFU;
    size_t i;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        uint8_t bit;
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x31U);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return crc;
}
#endif

bool air_quality_init(void)
{
#if !SENSOR_ENABLE_SGP30
    g_initialized = false;
    return false;
#else
    uint8_t cmd[2];
    uint8_t buf[3];

    cmd[0] = (uint8_t)(SGP30_INIT_AIR_QUALITY >> 8U);
    cmd[1] = (uint8_t)(SGP30_INIT_AIR_QUALITY & 0xFFU);

    if (!bsp_i2c_mem_write(BSP_I2C_DEVICE_SGP30, 0, cmd, sizeof(cmd)))
    {
        g_initialized = false;
        return false;
    }

    bsp_delay_ms(10);

    if (!bsp_i2c_mem_read(BSP_I2C_DEVICE_SGP30, 0, buf, sizeof(buf)))
    {
        g_initialized = false;
        return false;
    }

    g_init_ms = bsp_millis();
    g_initialized = true;
    return true;
#endif
}

bool air_quality_is_warmed_up(uint32_t elapsed_ms)
{
#if !SENSOR_ENABLE_SGP30
    (void)elapsed_ms;
    return false;
#else
    return g_initialized && (elapsed_ms >= SGP30_WARMUP_MS);
#endif
}

bool air_quality_read(uint16_t *co2eq, uint16_t *tvoc, sensor_health_status_t *health)
{
#if !SENSOR_ENABLE_SGP30
    if (co2eq != NULL)
    {
        *co2eq = 0U;
    }
    if (tvoc != NULL)
    {
        *tvoc = 0U;
    }
    if (health != NULL)
    {
        *health = SENSOR_HEALTH_DISABLED;
    }
    return false;
#else
    uint8_t cmd[2];
    uint8_t buf[6];
    uint8_t crc_expected;

    if (co2eq == NULL || tvoc == NULL || health == NULL)
    {
        return false;
    }

    if (!g_initialized)
    {
        *co2eq = 0U;
        *tvoc = 0U;
        *health = SENSOR_HEALTH_READ_FAILED;
        return false;
    }

    if (!air_quality_is_warmed_up(bsp_millis() - g_init_ms))
    {
        *co2eq = 0U;
        *tvoc = 0U;
        *health = SENSOR_HEALTH_STALE;
        return false;
    }

    cmd[0] = (uint8_t)(SGP30_MEASURE_AIR_QUALITY >> 8U);
    cmd[1] = (uint8_t)(SGP30_MEASURE_AIR_QUALITY & 0xFFU);

    if (!bsp_i2c_mem_write(BSP_I2C_DEVICE_SGP30, 0, cmd, sizeof(cmd)))
    {
        *co2eq = 0U;
        *tvoc = 0U;
        *health = SENSOR_HEALTH_READ_FAILED;
        return false;
    }

    bsp_delay_ms(12);

    if (!bsp_i2c_mem_read(BSP_I2C_DEVICE_SGP30, 0, buf, sizeof(buf)))
    {
        *co2eq = 0U;
        *tvoc = 0U;
        *health = SENSOR_HEALTH_READ_FAILED;
        return false;
    }

    crc_expected = sgp30_crc8(&buf[0], 2);
    if (crc_expected != buf[2])
    {
        *co2eq = 0U;
        *tvoc = 0U;
        *health = SENSOR_HEALTH_READ_FAILED;
        return false;
    }

    crc_expected = sgp30_crc8(&buf[3], 2);
    if (crc_expected != buf[5])
    {
        *co2eq = 0U;
        *tvoc = 0U;
        *health = SENSOR_HEALTH_READ_FAILED;
        return false;
    }

    *co2eq = (uint16_t)(((uint16_t)buf[0] << 8U) | (uint16_t)buf[1]);
    *tvoc = (uint16_t)(((uint16_t)buf[3] << 8U) | (uint16_t)buf[4]);
    *health = SENSOR_HEALTH_OK;
    return true;
#endif
}
