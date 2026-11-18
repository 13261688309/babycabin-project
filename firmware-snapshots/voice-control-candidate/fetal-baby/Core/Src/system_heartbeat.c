#include "system_heartbeat.h"
#include "baby_care_config.h"

static uint32_t s_last_ms;
static GPIO_PinState s_level;

void SystemHeartbeat_Init(void)
{
    s_level = GPIO_PIN_RESET;
    HAL_GPIO_WritePin(BABY_CARE_HEARTBEAT_PORT, BABY_CARE_HEARTBEAT_PIN, s_level);
    s_last_ms = HAL_GetTick();
}

void SystemHeartbeat_Service(void)
{
    const uint32_t now = HAL_GetTick();
    if ((now - s_last_ms) >= BABY_CARE_HEARTBEAT_TOGGLE_MS) {
        /* A late task emits one edge, never a catch-up burst that masks a stall. */
        s_last_ms = now;
        s_level = (s_level == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(BABY_CARE_HEARTBEAT_PORT, BABY_CARE_HEARTBEAT_PIN, s_level);
    }
}
