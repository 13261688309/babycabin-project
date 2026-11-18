#include "ir_temp.h"

#include "bsp/bsp.h"
#include "inc/app_config.h"

#if SENSOR_ENABLE_MLX90614
static float mlx90614_temperature_from_raw(uint8_t *raw)
{
    uint16_t value = (uint16_t)(((uint16_t)raw[1] << 8U) | (uint16_t)raw[0]);
    float temp_k = (float)value * 0.02f;
    return temp_k - 273.15f;
}
#endif

bool ir_temp_init(void)
{
#if !SENSOR_ENABLE_MLX90614
    return false;
#else
    return true;
#endif
}

bool ir_temp_read(float *object_c, float *ambient_c, sensor_health_status_t *health)
{
#if !SENSOR_ENABLE_MLX90614
    if (object_c != NULL)
    {
        *object_c = 0.0f;
    }
    if (ambient_c != NULL)
    {
        *ambient_c = 0.0f;
    }
    if (health != NULL)
    {
        *health = SENSOR_HEALTH_DISABLED;
    }
    return false;
#else
    uint8_t data[3];

    if (object_c == NULL || ambient_c == NULL || health == NULL)
    {
        return false;
    }

    if (!bsp_i2c_mem_read(BSP_I2C_DEVICE_MLX90614, MLX90614_REG_AMBIENT, data, 3))
    {
        *object_c = 0.0f;
        *ambient_c = 0.0f;
        *health = SENSOR_HEALTH_READ_FAILED;
        return false;
    }

    *ambient_c = mlx90614_temperature_from_raw(data);

    if (!bsp_i2c_mem_read(BSP_I2C_DEVICE_MLX90614, MLX90614_REG_OBJECT, data, 3))
    {
        *object_c = 0.0f;
        *health = SENSOR_HEALTH_READ_FAILED;
        return false;
    }

    *object_c = mlx90614_temperature_from_raw(data);
    *health = SENSOR_HEALTH_OK;
    return true;
#endif
}
