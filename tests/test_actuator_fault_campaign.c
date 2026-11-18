/* Reuse only the existing HAL fixture; production drivers are linked separately. */
#define main original_regression_main
#define HAL_TIM_PWM_Start fixture_pwm_start
#define HAL_TIM_PWM_Stop fixture_pwm_stop
#include "test_firmware.c"
#undef HAL_TIM_PWM_Stop
#undef HAL_TIM_PWM_Start
#undef main

static unsigned fail_start_mask, fail_stop_mask;
static HAL_StatusTypeDef injected_error;
static unsigned start_attempts[3], stop_attempts[3];
static unsigned require_gate_off_during_stop;
static unsigned cases, persistent_stop_cases, start_failure_cases, controls;

static unsigned actuator_index(TIM_HandleTypeDef *timer, uint32_t channel)
{
    if (timer == &htim3 && channel == TIM_CHANNEL_1) { return 0U; }
    if (timer == &htim3 && channel == TIM_CHANNEL_2) { return 1U; }
    assert(timer == &htim2 && channel == TIM_CHANNEL_3);
    return 2U;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    const unsigned index = actuator_index(timer, channel);
    start_attempts[index]++;
    if (fail_start_mask & (1U << index)) { return injected_error; }
    return fixture_pwm_start(timer, channel);
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
    const unsigned index = actuator_index(timer, channel);
    stop_attempts[index]++;
    if (require_gate_off_during_stop) {
        assert(!Interlock_GetStatus()->enabled);
        assert((BABY_CARE_INTERLOCK_PORT->ODR & BABY_CARE_INTERLOCK_ENABLE_PIN) == 0U);
    }
    if (fail_stop_mask & (1U << index)) { return injected_error; }
    return fixture_pwm_stop(timer, channel);
}

static void prepare(uint32_t base)
{
    fail_start_mask = fail_stop_mask = require_gate_off_during_stop = 0U;
    injected_error = HAL_ERROR;
    memset(start_attempts, 0, sizeof(start_attempts));
    memset(stop_attempts, 0, sizeof(stop_attempts));
    memset(test_gpio, 0, sizeof(test_gpio));
    memset(&huart1, 0, sizeof(huart1));
    memset(&huart2, 0, sizeof(huart2));
    memset(&huart3, 0, sizeof(huart3));
    memset(&htim2, 0, sizeof(htim2));
    memset(&htim3, 0, sizeof(htim3));
    htim2.ARR = 99U; htim3.ARR = 19999U;
    pwm_result = tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    override_motor_stop = check_motor_gpio_init = 0U;
    tx_data = voice_tx_data = esp_tx_data = NULL;
    simulate_arm_latch = 1U;
    tick = base;
    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
    BabyCare_Init(); BabyCare_Task();
    tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    BabyCare_Task();
    tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
    assert(Interlock_GetStatus()->qualified && !Interlock_GetStatus()->enabled);
    assert(BabyCare_GetStatus()->fault_flags == 0U);
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    memset(stop_attempts, 0, sizeof(stop_attempts));
}

static void assert_latched_and_cannot_restart(void)
{
    const unsigned attempts = start_attempts[0] + start_attempts[1] + start_attempts[2];
    const BabyCare_StatusTypeDef *state = BabyCare_GetStatus();
    assert(state->safety_latched && state->mode == BABY_CARE_MODE_FAULT);
    assert(state->fault_flags != 0U);
    assert(!Interlock_GetStatus()->enabled && !Interlock_GetStatus()->ready);
    assert((BABY_CARE_INTERLOCK_PORT->ODR & (BABY_CARE_INTERLOCK_ENABLE_PIN | BABY_CARE_INTERLOCK_READY_PIN)) == 0U);
    /* A repaired HAL call alone is not permission to clear an actuator fault. */
    fail_start_mask = fail_stop_mask = 0U;
    assert(!BabyCare_ClearSafetyLatch());
    app_voice(VOICE_CMD_FORWARD);
    app_remote("$A110#$B110#");
    for (unsigned i = 0U; i < 4U; i++) {
        tick += 20U; BabyCare_Task();
        assert(!BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
        assert(BabyCare_GetStatus()->safety_latched);
        assert(!Interlock_GetStatus()->enabled && !Interlock_GetStatus()->ready);
    }
    assert(start_attempts[0] + start_attempts[1] + start_attempts[2] == attempts);
}

static void inject_stop(unsigned trigger, uint32_t started)
{
    switch (trigger) {
    case 0U: app_voice(VOICE_CMD_STOP_1); break;
    case 1U: app_remote("$STOP#"); break;
    case 2U: app_voice(VOICE_CMD_ALARM); break;
    case 3U: BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_STOP_PIN; break;
    case 4U: BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_POWER_PIN; break;
    case 5U: BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_ARMED_PIN; break;
    case 6U: tick = started + BABY_CARE_SOOTHE_MAX_MS; break;
    default: assert(0);
    }
    BabyCare_Task();
}

static void test_stopping(uint32_t base, unsigned trigger, unsigned mask, HAL_StatusTypeDef error)
{
    uint32_t started;
    unsigned expected_faults = 0U;
    prepare(base);
    assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    started = tick;
    for (unsigned i = 0U; i < 45U; i++) { tick += 20U; BabyCare_Task(); }
    assert(htim2.enabled != 0U && htim3.enabled != 0U);
    assert(htim2.compare[TIM_CHANNEL_3] != 0U);
    fail_stop_mask = mask; injected_error = error;
    require_gate_off_during_stop = 1U;
    inject_stop(trigger, started);
    for (unsigned i = 0U; i < 3U; i++) { assert(stop_attempts[i] != 0U); }
    assert(!Interlock_GetStatus()->enabled);
    assert(htim2.compare[TIM_CHANNEL_3] == 0U);
    if (mask & 3U) { expected_faults |= BABY_CARE_FAULT_SERVO; }
    if (mask & 4U) { expected_faults |= BABY_CARE_FAULT_MOTOR; }
    assert(BabyCare_GetStatus()->fault_flags == expected_faults);
    if (mask != 0U) {
        /* Failed HAL stop leaves the modelled PWM active: the external gate is essential. */
        assert(htim3.enabled == (((mask & 1U) ? (1U << TIM_CHANNEL_1) : 0U) |
                                ((mask & 2U) ? (1U << TIM_CHANNEL_2) : 0U)));
        assert((htim2.enabled != 0U) == ((mask & 4U) != 0U));
        assert_latched_and_cannot_restart();
        persistent_stop_cases++;
    } else {
        assert(htim2.enabled == 0U && htim3.enabled == 0U);
        controls++;
    }
    cases++;
}

static void test_starting(uint32_t base, unsigned axis, HAL_StatusTypeDef error)
{
    prepare(base);
    fail_start_mask = 1U << axis; injected_error = error;
    assert(!BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    assert(BabyCare_GetStatus()->fault_flags == (axis == 2U ? BABY_CARE_FAULT_MOTOR : BABY_CARE_FAULT_SERVO));
    assert(start_attempts[axis] == 1U);
    for (unsigned i = axis + 1U; i < 3U; i++) { assert(start_attempts[i] == 0U); }
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    require_gate_off_during_stop = 1U;
    assert_latched_and_cannot_restart();
    cases++; start_failure_cases++;
}

int main(void)
{
    const uint32_t bases[] = {0U, UINT32_MAX - 100U};
    const HAL_StatusTypeDef errors[] = {HAL_BUSY, HAL_ERROR, HAL_TIMEOUT};
    for (unsigned b = 0U; b < 2U; b++) {
        for (unsigned trigger = 0U; trigger < 7U; trigger++) {
            test_stopping(bases[b], trigger, 0U, HAL_OK);
            for (unsigned mask = 1U; mask < 8U; mask++) {
                for (unsigned error = 0U; error < 3U; error++) {
                    test_stopping(bases[b], trigger, mask, errors[error]);
                }
            }
        }
        for (unsigned axis = 0U; axis < 3U; axis++) {
            for (unsigned error = 0U; error < 3U; error++) { test_starting(bases[b], axis, errors[error]); }
        }
    }
    assert(cases == 326U && persistent_stop_cases == 294U && start_failure_cases == 18U && controls == 14U);
    printf("PASS actuator fault campaign: %u cases; %u persistent-stop, %u partial-start, %u healthy controls\n",
           cases, persistent_stop_cases, start_failure_cases, controls);
    puts("Host HAL model only: not measured physical power cut, stop latency or mechanical safety.");
    return 0;
}
