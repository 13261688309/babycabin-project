/* Exercise actual UART parsers and application with simulated HAL/interlocks. */
#define main previous_firmware_regression_main
#include "test_firmware.c"
#undef main

static void arm_for_test(uint32_t base, unsigned motor)
{
    tick = base;
    pwm_result = tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    tx_data = NULL;
    simulate_arm_latch = 1U;
    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
    BabyCare_Init();
    BabyCare_Task();
    tick += BABY_CARE_INTERLOCK_STABLE_MS;
    BabyCare_Task();
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    BabyCare_Task();
    tick += BABY_CARE_INTERLOCK_STABLE_MS;
    BabyCare_Task();
    if (motor) {
        assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    } else {
        app_remote("$A110#");
        BabyCare_Task();
    }
    assert(Interlock_GetStatus()->enabled && htim3.enabled != 0U);
}

static unsigned stopped(void)
{
    return htim2.enabled == 0U && htim3.enabled == 0U &&
           !Interlock_GetStatus()->enabled;
}

int main(void)
{
    const uint32_t bases[] = {0U, UINT32_MAX - 100U};
    unsigned cases = 0U, failures = 0U;
    for (unsigned base = 0; base < 2U; ++base) {
        for (unsigned motor = 0; motor < 2U; ++motor) {
            for (unsigned source = 0; source < 6U; ++source) {
                arm_for_test(bases[base], motor);
                if (source < 2U) {
                    /* More than one remote target survives a one-cycle guard. */
                    app_remote("$A120#$B080#$A100#");
                    app_voice(source == 0U ? VOICE_CMD_STOP_1 : VOICE_CMD_STOP_2);
                } else if (source < 4U) {
                    app_voice(VOICE_CMD_FORWARD);
                    app_voice(VOICE_CMD_BACKWARD);
                    app_voice(VOICE_CMD_ROTATE_LEFT);
                    app_remote(source == 2U ? "$STOP#" : "$7,0,0,0#");
                } else if (source == 4U) {
                    app_remote("$A120#$B080#");
                    app_voice(VOICE_CMD_FORWARD);
                    app_voice(VOICE_CMD_BACKWARD);
                    BabyCare_Stop();
                } else {
                    app_remote("$A120#$B080#");
                    app_voice(VOICE_CMD_FORWARD);
                    app_voice(VOICE_CMD_BACKWARD);
                    assert(BabyCare_RequestMode(BABY_CARE_MODE_IDLE));
                }
                unsigned failed = 0U;
                for (unsigned cycle = 0; cycle < 8U; ++cycle) {
                    ++tick;
                    BabyCare_Task();
                    if (!stopped()) { failed = 1U; }
                }
                if (failed) {
                    printf("FAIL queued motion after STOP: base=%u motor=%u source=%u\n", base, motor, source);
                }
                failures += failed;
                ++cases;
            }
        }
    }
    /* A partial frame begun before STOP is stale even if completed later. */
    for (unsigned source = 0; source < 2U; ++source) {
        arm_for_test(0U, 0U);
        if (source == 0U) {
            app_remote("$A1");
            app_voice(VOICE_CMD_STOP_1);
            BabyCare_Task();
            app_remote("20#");
        } else {
            const uint8_t prefix[] = {0xAA, 0x55, VOICE_UART_CMD_TYPE};
            for (size_t i = 0; i < sizeof(prefix); ++i) {
                *huart2.rx = prefix[i]; BabyCare_OnUartRxComplete(&huart2);
            }
            app_remote("$STOP#");
            BabyCare_Task();
            *huart2.rx = VOICE_CMD_FORWARD; BabyCare_OnUartRxComplete(&huart2);
            *huart2.rx = 0xFB; BabyCare_OnUartRxComplete(&huart2);
        }
        BabyCare_Task();
        if (!stopped()) { ++failures; printf("FAIL partial motion after STOP: source=%u\n", source); }
        ++cases;
    }
    /* Purging ordinary commands must retain independently pending urgent input. */
    arm_for_test(0U, 0U);
    app_voice(VOICE_CMD_ALARM);
    app_remote("$STOP#");
    BabyCare_Stop();
    BabyCare_Task();
    assert(stopped() && BabyCare_GetStatus()->mode == BABY_CARE_MODE_ALERT);
    assert(BabyCare_GetStatus()->safety_latched);
    ++cases;

    /* Do not silently remove motion: genuinely new commands still work when armed. */
    arm_for_test(0U, 0U);
    app_voice(VOICE_CMD_STOP_1); BabyCare_Task(); BabyCare_Task();
    assert(stopped());
    app_remote("$A120#"); BabyCare_Task();
    assert(htim3.enabled != 0U && Interlock_GetStatus()->enabled);
    app_remote("$STOP#"); BabyCare_Task(); BabyCare_Task();
    app_voice(VOICE_CMD_FORWARD); BabyCare_Task();
    assert(htim2.enabled != 0U && htim3.enabled != 0U);
    BabyCare_Stop();
    assert(stopped());
    ++cases;
    printf("%s STOP queue barrier: %u cases, %u failures; host HAL only, no physical qualification.\n",
           failures ? "FAIL" : "PASS", cases, failures);
    return failures ? 1 : 0;
}
