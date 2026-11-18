#include "camera_module.h"

#include <stddef.h>

#include "bsp/bsp.h"
#include "inc/app_config.h"

#if SENSOR_ENABLE_ESP32_CAMERA
static uint16_t u16_from_be(uint8_t high, uint8_t low)
{
    return (uint16_t)(((uint16_t)high << 8U) | (uint16_t)low);
}
#endif

bool camera_module_init(void)
{
#if SENSOR_ENABLE_ESP32_CAMERA
    uint8_t reset = 1U;
    uint8_t release = 0U;

    (void)bsp_i2c_mem_write(BSP_I2C_DEVICE_ESP32_CAMERA, ESP32_CAMERA_REG_RESET, &reset, 1U);
    bsp_delay_ms(20U);
    (void)bsp_i2c_mem_write(BSP_I2C_DEVICE_ESP32_CAMERA, ESP32_CAMERA_REG_RESET, &release, 1U);
    return camera_module_set_mode((camera_ai_mode_t)ESP32_CAMERA_DEFAULT_AI_MODE);
#else
    return true;
#endif
}

bool camera_module_set_mode(camera_ai_mode_t mode)
{
    if (mode > CAMERA_AI_MODE_QR_CODE)
    {
        return false;
    }

#if SENSOR_ENABLE_ESP32_CAMERA
    const uint8_t value = (uint8_t)mode;
    return bsp_i2c_mem_write(BSP_I2C_DEVICE_ESP32_CAMERA, ESP32_CAMERA_REG_AI_MODE, &value, 1U);
#else
    (void)mode;
    return false;
#endif
}

bool camera_module_read_detection(camera_detection_t *detection)
{
#if SENSOR_ENABLE_ESP32_CAMERA
    uint8_t raw[ESP32_CAMERA_RESULT_BYTES] = {0};
#endif

    if (detection == NULL)
    {
        return false;
    }

    detection->middle_x = 0U;
    detection->middle_y = 0U;
    detection->id = 0;
    detection->area = 0U;
    detection->valid = false;

#if SENSOR_ENABLE_ESP32_CAMERA
    if (!bsp_i2c_mem_read(BSP_I2C_DEVICE_ESP32_CAMERA, ESP32_CAMERA_REG_RESULT_BASE, raw, sizeof(raw)))
    {
        return false;
    }

    detection->middle_x = u16_from_be(raw[0], raw[1]);
    detection->middle_y = u16_from_be(raw[2], raw[3]);
    detection->id = (int16_t)u16_from_be(raw[4], raw[5]);
    detection->area = u16_from_be(raw[6], raw[7]);
    detection->valid = detection->area > 0U;
    return true;
#else
    return false;
#endif
}
