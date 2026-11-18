#include "presence_detector.h"

#include "bsp/bsp.h"
#include "inc/app_config.h"

bool presence_detector_init(void)
{
#if !SENSOR_ENABLE_LD2410
    return false;
#else
    return true;
#endif
}

bool presence_detector_poll(bool *baby_in_bed, uint8_t *distance_cm, sensor_health_status_t *health)
{
#if !SENSOR_ENABLE_LD2410
    if (baby_in_bed != NULL)
    {
        *baby_in_bed = true;
    }
    if (distance_cm != NULL)
    {
        *distance_cm = 0U;
    }
    if (health != NULL)
    {
        *health = SENSOR_HEALTH_DISABLED;
    }
    return false;
#else
    bool active = false;

    if (baby_in_bed == NULL || distance_cm == NULL || health == NULL)
    {
        return false;
    }

    if (!bsp_gpio_read(BSP_GPIO_PRESENCE_DET, &active))
    {
        *baby_in_bed = false;
        *distance_cm = 0U;
        *health = SENSOR_HEALTH_READ_FAILED;
        return false;
    }

    *baby_in_bed = active;
    *distance_cm = 0U;
    *health = SENSOR_HEALTH_OK;
    return true;
#endif
}
