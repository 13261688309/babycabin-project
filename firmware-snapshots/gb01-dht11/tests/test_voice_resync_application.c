/* Exercise production IRQ -> command queue -> application stop with HAL doubles. */
#define main existing_application_regression_main
#include "test_firmware.c"
#undef main

int main(void)
{
    const uint8_t prefix[] = {0xAA, 0x55, 0, VOICE_CMD_FORWARD};
    const uint8_t urgent[] = {VOICE_CMD_STOP_1, VOICE_CMD_STOP_2, VOICE_CMD_ALARM};
    const uint32_t bases[] = {0U, UINT32_MAX - 100U};
    unsigned cases = 0;
    pwm_result = tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    simulate_arm_latch = 1U;
    for (size_t base = 0; base < 2; ++base) {
        for (unsigned motor = 0; motor < 2; ++motor) {
            for (size_t length = 1; length <= sizeof(prefix); ++length) {
                for (size_t id = 0; id < sizeof(urgent); ++id) {
                    tick = bases[base];
                    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
                    BabyCare_Init(); BabyCare_Task();
                    tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
                    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
                    BabyCare_Task(); tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
                    if (motor) {
                        assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
                        tick += 20U; BabyCare_Task();
                        assert(htim2.enabled != 0U);
                    } else {
                        app_remote("$A110#$B080#"); BabyCare_Task(); BabyCare_Task();
                    }
                    assert(Interlock_GetStatus()->enabled && htim3.enabled != 0U);
                    for (size_t i = 0; i < length; ++i) {
                        assert(huart2.rx); *huart2.rx = prefix[i];
                        BabyCare_OnUartRxComplete(&huart2);
                    }
                    app_voice(urgent[id]);
                    app_remote("$A100#"); /* A queued remote target must not undo STOP. */
                    tick++; BabyCare_Task();
                    if (htim2.enabled || htim3.enabled || Interlock_GetStatus()->enabled) {
                        printf("FAIL application STOP lost: base=%u motor=%u prefix=%u id=%u\n",
                               (unsigned)base, motor, (unsigned)length, urgent[id]);
                        return 1;
                    }
                    if (urgent[id] == VOICE_CMD_ALARM) {
                        assert(BabyCare_GetStatus()->mode == BABY_CARE_MODE_ALERT && BabyCare_GetStatus()->safety_latched);
                    } else {
                        assert(BabyCare_GetStatus()->mode == BABY_CARE_MODE_IDLE);
                    }
                    BabyCare_Task(); BabyCare_Task();
                    assert(htim2.enabled == 0U && htim3.enabled == 0U && !Interlock_GetStatus()->enabled);
                    cases++;
                }
            }
        }
    }
    printf("PASS application voice resync: %u cases; PWM disabled and gate request removed in host model only.\n", cases);
    return cases == 48U ? 0 : 1;
}
