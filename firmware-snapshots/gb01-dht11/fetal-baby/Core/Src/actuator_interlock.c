#include "actuator_interlock.h"
#include "baby_care_config.h"
#include <string.h>

static Interlock_Status s_state;
static uint32_t s_stop_since;
static uint32_t s_arm_since;
static uint8_t s_stop_tracking;
static uint8_t s_arm_tracking;
static uint8_t s_bootstrap;
static uint8_t s_was_armed;
static uint8_t s_power_seen;
static uint32_t s_enable_since;

static void set_ready(uint8_t ready)
{
    HAL_GPIO_WritePin(BABY_CARE_INTERLOCK_PORT, BABY_CARE_INTERLOCK_READY_PIN,
                      ready ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s_state.ready = ready;
}

void Interlock_Disable(void)
{
    HAL_GPIO_WritePin(BABY_CARE_INTERLOCK_PORT, BABY_CARE_INTERLOCK_ENABLE_PIN,
                      GPIO_PIN_RESET);
    s_state.enabled = 0U;
    s_state.power_pending = 0U;
    s_power_seen = 0U;
}

static void trip(Interlock_TripReason reason)
{
    Interlock_Disable();
    set_ready(0U);
    s_bootstrap = 0U;
    s_arm_tracking = 0U;
    s_was_armed = 0U;
    s_state.qualified = 0U;
    if (!s_state.tripped) {
        s_state.trip_count++;
        s_state.reason = reason;
    }
    s_state.tripped = 1U;
}

void Interlock_Trip(void) { trip(INTERLOCK_TRIP_SOFTWARE); }

void Interlock_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_stop_tracking = 0U;
    s_arm_tracking = 0U;
    s_bootstrap = 1U;
    s_was_armed = 0U;
    Interlock_Disable();
    set_ready(0U);
}

void Interlock_Poll(void)
{
    const uint32_t now = HAL_GetTick();
    s_state.stop_ok = HAL_GPIO_ReadPin(BABY_CARE_INTERLOCK_PORT,
                                     BABY_CARE_INTERLOCK_STOP_PIN) == GPIO_PIN_SET;
    s_state.armed = HAL_GPIO_ReadPin(BABY_CARE_INTERLOCK_PORT,
                                   BABY_CARE_INTERLOCK_ARMED_PIN) == GPIO_PIN_SET;
    s_state.power_good = HAL_GPIO_ReadPin(BABY_CARE_INTERLOCK_PORT,
                                        BABY_CARE_INTERLOCK_POWER_PIN) == GPIO_PIN_SET;
    if (!s_state.stop_ok) {
        s_stop_tracking = 0U;
        trip(INTERLOCK_TRIP_STOP);
        return;
    }
    if (!s_stop_tracking) { s_stop_since = now; s_stop_tracking = 1U; }
    /* A power trip may also clear ARM in hardware before the next poll. */
    if (s_state.enabled) {
        if (s_state.power_good) {
            s_power_seen = 1U;
            s_state.power_pending = 0U;
        } else if (s_power_seen ||
                   (now - s_enable_since) >= BABY_CARE_INTERLOCK_POWER_TIMEOUT_MS) {
            trip(INTERLOCK_TRIP_POWER);
            return;
        }
    }
    /* A high feedback while READY is low cannot be a valid latched ARM. */
    if ((!s_state.ready && s_state.armed) ||
        ((s_state.enabled || s_was_armed) && !s_state.armed)) {
        trip(INTERLOCK_TRIP_FEEDBACK);
        return;
    }
    if (s_bootstrap && (now - s_stop_since) >= BABY_CARE_INTERLOCK_STABLE_MS) {
        set_ready(1U);
        s_bootstrap = 0U;
    }
    if (s_state.ready && s_state.armed) {
        if (!s_arm_tracking) { s_arm_since = now; s_arm_tracking = 1U; }
    } else {
        s_arm_tracking = 0U;
    }
    s_state.qualified = s_state.ready && s_arm_tracking &&
        (now - s_stop_since) >= BABY_CARE_INTERLOCK_STABLE_MS &&
        (now - s_arm_since) >= BABY_CARE_INTERLOCK_STABLE_MS;
    if (s_state.qualified) { s_was_armed = 1U; }
}

uint8_t Interlock_Enable(void)
{
    Interlock_Poll();
    if (s_state.tripped || !s_state.qualified) { return 0U; }
    if (s_state.enabled) { return 1U; }
    /* A repeated request must not restart the power-up deadline. */
    s_enable_since = HAL_GetTick();
    s_power_seen = 0U;
    s_state.power_pending = 1U;
    HAL_GPIO_WritePin(BABY_CARE_INTERLOCK_PORT, BABY_CARE_INTERLOCK_ENABLE_PIN,
                      GPIO_PIN_SET);
    s_state.enabled = 1U;
    /* Recheck the raw inputs; positive qualification must not delay a trip. */
    Interlock_Poll();
    return s_state.enabled;
}

uint8_t Interlock_Reset(void)
{
    Interlock_Disable();
    Interlock_Poll();
    if (!s_state.stop_ok || !s_stop_tracking ||
        (HAL_GetTick() - s_stop_since) < BABY_CARE_INTERLOCK_STABLE_MS) { return 0U; }
    if (!s_state.ready) {
        if (s_state.armed) { return 0U; }
        set_ready(1U);
        s_arm_tracking = 0U;
        return 0U;
    }
    if (!s_state.qualified) { return 0U; }
    s_state.tripped = 0U;
    s_state.reason = INTERLOCK_TRIP_NONE;
    return 1U;
}

const Interlock_Status *Interlock_GetStatus(void) { return &s_state; }
