#include "load_cell.h"

#include "bsp/bsp.h"
#include "inc/app_config.h"

bool load_cell_init(void)
{
#if !SENSOR_ENABLE_HX711
    return false;
#else
    return true;
#endif
}

bool load_cell_read(uint16_t *weight_g, sensor_health_status_t *health)
{
#if !SENSOR_ENABLE_HX711
    if (weight_g != NULL)
    {
        *weight_g = 0U;
    }
    if (health != NULL)
    {
        *health = SENSOR_HEALTH_DISABLED;
    }
    return false;
#else
    /* HX711: wire DAT to a GPIO input, SCK to a GPIO output.
       Full bit-bang read: 24 clock pulses, reading DAT after each. */
    if (weight_g == NULL || health == NULL)
    {
        return false;
    }

    /* Placeholder until HX711 GPIO pins are assigned in CubeMX */
    *weight_g = 0U;
    *health = SENSOR_HEALTH_READ_FAILED;
    return false;
#endif
}
