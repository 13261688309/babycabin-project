#include "ambient_light.h"

#include "bsp/bsp.h"
#include "inc/app_config.h"

bool ambient_light_init(void)
{
#if !SENSOR_ENABLE_BH1750
    return false;
#else
    uint8_t cmd = BH1750_POWER_ON;
    if (!bsp_i2c_mem_write(BSP_I2C_DEVICE_BH1750, 0, &cmd, 1))
    {
        return false;
    }

    bsp_delay_ms(10);
    cmd = BH1750_MODE_CONT_HIRES;
    if (!bsp_i2c_mem_write(BSP_I2C_DEVICE_BH1750, 0, &cmd, 1))
    {
        return false;
    }

    bsp_delay_ms(180);
    return true;
#endif
}

bool ambient_light_read(uint16_t *lux, sensor_health_status_t *health)
{
#if !SENSOR_ENABLE_BH1750
    if (lux != NULL)
    {
        *lux = 0U;
    }
    if (health != NULL)
    {
        *health = SENSOR_HEALTH_DISABLED;
    }
    return false;
#else
    uint8_t data[2] = {0};

    if (lux == NULL || health == NULL)
    {
        return false;
    }

    if (!bsp_i2c_mem_read(BSP_I2C_DEVICE_BH1750, 0, data, sizeof(data)))
    {
        *lux = 0U;
        *health = SENSOR_HEALTH_READ_FAILED;
        return false;
    }

    *lux = (uint16_t)(((uint16_t)data[0] << 8U) | (uint16_t)data[1]);
    *lux = (uint16_t)((float)(*lux) / 1.2f);
    *health = SENSOR_HEALTH_OK;
    return true;
#endif
}
