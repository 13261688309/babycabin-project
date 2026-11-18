/* Actual app, UART parsers and PWM drivers; only HAL and external pins are modeled. */
#define STOP_QUEUE_ENTRY previous_stop_regression_main
#include "test_stop_queue_barrier.c"

static unsigned checks;
#define CHECK(expr) do { assert(expr); ++checks; } while (0)

static void ready(uint32_t base)
{
    override_motor_stop = 0U;
    start_fail_timer = NULL;
    memset(test_gpio, 0, sizeof(test_gpio));
    arm_for_test(base, 0U);
    BabyCare_Stop();
    BabyCare_Task();
    CHECK(stopped());
    CHECK(!BabyCare_GetStatus()->safety_latched);
}

static void advance(uint32_t ms)
{
    for (uint32_t i = 0U; i < ms; ++i) { ++tick; BabyCare_Task(); }
}

static void voice(uint8_t id)
{
    app_voice(id);
    BabyCare_Task();
}

static void independent_voice(uint32_t base)
{
    const BabyCare_StatusTypeDef *s;
    ready(base);
    voice(VOICE_CMD_FORWARD);
    s = BabyCare_GetStatus();
    CHECK(s->motion == BABY_CARE_MOTION_MOTOR);
    CHECK(s->motor_target_percent == BABY_CARE_MANUAL_MOTOR_MAX_PERCENT);
    CHECK(htim2.enabled != 0U && htim3.enabled == 0U);
    advance(200U);
    CHECK(htim2.compare[TIM_CHANNEL_3] > 0U);
    CHECK((GPIOA->ODR & GPIO_PIN_4) != 0U && !(GPIOA->ODR & GPIO_PIN_5));
    voice(VOICE_CMD_BACKWARD);
    CHECK(s->motor_target_percent == -BABY_CARE_MANUAL_MOTOR_MAX_PERCENT);
    advance(400U);
    CHECK(htim2.compare[TIM_CHANNEL_3] > 0U);
    CHECK(!(GPIOA->ODR & GPIO_PIN_4) && (GPIOA->ODR & GPIO_PIN_5));
    CHECK(unsafe_direction_changes == 0U);
    voice(VOICE_CMD_TURN_LEFT);
    CHECK(s->motion == BABY_CARE_MOTION_GIMBAL);
    CHECK(htim2.enabled == 0U && htim3.enabled != 0U);
    CHECK(s->motor_target_percent == 0 && s->pan_target_deg == 80U);
    CHECK(s->tilt_target_deg == 90U);
    advance(200U);
    CHECK(htim3.compare[TIM_CHANNEL_1] < 1500U);
    voice(VOICE_CMD_TURN_RIGHT);
    CHECK(s->pan_target_deg == 90U && s->tilt_target_deg == 90U);
    voice(VOICE_CMD_STOP_1);
    CHECK(stopped() && s->motion == BABY_CARE_MOTION_NONE);
    CHECK(s->voice_stop_applied == 1U);
    BabyCare_Task();
    voice(VOICE_CMD_ROTATE_LEFT);
    CHECK(s->mode == BABY_CARE_MODE_SOOTHE && s->motion == BABY_CARE_MOTION_SOOTHE);
    CHECK(htim2.enabled != 0U && htim3.enabled != 0U);
    voice(VOICE_CMD_STOP_2);
    CHECK(stopped() && s->voice_stop_applied == 2U);
}

static void bounds_and_targets(uint32_t base)
{
    ready(base);
    CHECK(!BabyCare_RequestMotor(21, 1000U));
    CHECK(!BabyCare_RequestMotor(-32768, 1000U));
    CHECK(!BabyCare_RequestMotor(20, 0U));
    CHECK(!BabyCare_RequestMotor(20, UINT32_MAX));
    CHECK(!BabyCare_RequestGimbal(59U, 90U, 1000U));
    CHECK(!BabyCare_RequestGimbal(120U, 106U, 1000U));
    CHECK(!BabyCare_RequestGimbal(65535U, 90U, 1000U));
    CHECK(!BabyCare_RequestGimbal(90U, 90U, 0U));
    CHECK(stopped());
    CHECK(BabyCare_RequestGimbal(100U, 95U, 1000U));
    advance(200U);
    CHECK(htim3.compare[TIM_CHANNEL_1] > 1500U);
    CHECK(htim3.compare[TIM_CHANNEL_2] < 1500U);
    app_remote("$B180#"); BabyCare_Task();
    CHECK(BabyCare_GetStatus()->pan_target_deg == 120U);
    CHECK(BabyCare_GetStatus()->tilt_target_deg == 95U);
    app_remote("$A000#"); BabyCare_Task();
    CHECK(BabyCare_GetStatus()->pan_target_deg == 120U);
    CHECK(BabyCare_GetStatus()->tilt_target_deg == 75U);
    for (unsigned i = 0; i < 20U; ++i) { voice(VOICE_CMD_TURN_LEFT); }
    CHECK(BabyCare_GetStatus()->pan_target_deg == 60U);
    CHECK(BabyCare_RequestMotor(0, 0U));
    CHECK(stopped());
}

static void deadlines(uint32_t base)
{
    ready(base);
    uint32_t start = tick;
    voice(VOICE_CMD_FORWARD);
    tick = start + BABY_CARE_VOICE_MOTOR_MS - 1U;
    voice(VOICE_CMD_BACKWARD);
    CHECK(BabyCare_GetStatus()->motion_remaining_ms == 1U);
    app_voice(VOICE_CMD_FORWARD);
    app_remote("$A110#$B110#");
    ++tick; BabyCare_Task();
    CHECK(stopped());
    for (unsigned i = 0; i < 5U; ++i) { BabyCare_Task(); CHECK(stopped()); }
    ready(base);
    start = tick;
    CHECK(BabyCare_RequestMotor(10, BABY_CARE_MOTION_SESSION_MAX_MS));
    tick = start + 1000U; BabyCare_Task();
    CHECK(BabyCare_RequestGimbal(100U, 90U, BABY_CARE_MOTION_SESSION_MAX_MS));
    tick = start + 3000U; BabyCare_Task();
    CHECK(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    tick = start + BABY_CARE_MOTION_SESSION_MAX_MS - 1U; BabyCare_Task();
    CHECK(BabyCare_GetStatus()->motion_remaining_ms == 1U);
    ++tick;
    CHECK(!BabyCare_RequestMotor(10, 1000U));
    CHECK(stopped());
    BabyCare_Task();
    CHECK(BabyCare_RequestGimbal(90U, 90U, 500U));
    tick += 500U;
    CHECK(!BabyCare_RequestGimbal(100U, 95U, 1000U));
    CHECK(stopped());
    BabyCare_Task();
    CHECK(BabyCare_RequestMotor(10, 1U));
    ++tick; BabyCare_Task();
    CHECK(stopped());
}

static void faults(uint32_t base)
{
    for (unsigned gimbal = 0U; gimbal < 2U; ++gimbal) {
        for (unsigned fault = 0U; fault < 3U; ++fault) {
            ready(base);
            if (gimbal) { CHECK(BabyCare_RequestGimbal(110U, 100U, 1000U)); }
            else { CHECK(BabyCare_RequestMotor(20, 1000U)); }
            BabyCare_Task();
            const uint16_t pins[] = {BABY_CARE_INTERLOCK_STOP_PIN,
                BABY_CARE_INTERLOCK_ARMED_PIN, BABY_CARE_INTERLOCK_POWER_PIN};
            BABY_CARE_INTERLOCK_PORT->ODR &= ~pins[fault];
            BabyCare_Task();
            CHECK(stopped() && BabyCare_GetStatus()->safety_latched);
            CHECK(!BabyCare_RequestMotor(10, 1000U));
            CHECK(!BabyCare_RequestGimbal(90U, 90U, 1000U));
            BABY_CARE_INTERLOCK_PORT->ODR |= pins[fault];
            BabyCare_Task(); CHECK(stopped());
        }
        ready(base);
        pwm_result = HAL_ERROR;
        if (gimbal) { CHECK(!BabyCare_RequestGimbal(110U, 100U, 1000U)); }
        else { CHECK(!BabyCare_RequestMotor(20, 1000U)); }
        CHECK(!Interlock_GetStatus()->enabled && BabyCare_GetStatus()->safety_latched);
        pwm_result = HAL_OK;
        ready(base);
        if (gimbal) { CHECK(BabyCare_RequestGimbal(110U, 100U, 1000U)); }
        else { CHECK(BabyCare_RequestMotor(20, 1000U)); }
        pwm_result = HAL_ERROR;
        BabyCare_Stop();
        CHECK(!Interlock_GetStatus()->enabled && BabyCare_GetStatus()->fault_flags);
        CHECK(!BabyCare_ClearSafetyLatch());
        pwm_result = HAL_OK;
    }
    for (unsigned channel = 0U; channel < 3U; ++channel) {
        ready(base);
        start_fail_timer = channel == 0U ? &htim2 : &htim3;
        start_fail_channel = channel == 0U ? TIM_CHANNEL_3 :
            (channel == 1U ? TIM_CHANNEL_1 : TIM_CHANNEL_2);
        if (channel == 0U) { CHECK(!BabyCare_RequestMotor(20, 1000U)); }
        else { CHECK(!BabyCare_RequestGimbal(110U, 100U, 1000U)); }
        CHECK(stopped() && BabyCare_GetStatus()->safety_latched);
        CHECK(BabyCare_GetStatus()->fault_flags ==
            (channel == 0U ? BABY_CARE_FAULT_MOTOR : BABY_CARE_FAULT_SERVO));
    }
    ready(base);
    BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_POWER_PIN;
    uint32_t start = tick;
    CHECK(BabyCare_RequestMotor(10, 1000U));
    tick = start + BABY_CARE_INTERLOCK_POWER_TIMEOUT_MS - 1U; BabyCare_Task();
    CHECK(!BabyCare_RequestGimbal(100U, 90U, 1000U));
    CHECK(BabyCare_RequestMotor(20, 1000U));
    ++tick; BabyCare_Task();
    CHECK(stopped() && BabyCare_GetStatus()->safety_latched);
    ready(base);
    BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_ARMED_PIN;
    CHECK(!BabyCare_RequestMotor(10, 1000U));
    CHECK(!BabyCare_RequestGimbal(90U, 90U, 1000U));
    CHECK(stopped());
}

#ifndef INDEPENDENT_ENTRY
#define INDEPENDENT_ENTRY main
#endif
int INDEPENDENT_ENTRY(void)
{
    const uint32_t bases[] = {0U, UINT32_MAX - 250U};
    CHECK(previous_stop_regression_main() == 0);
    for (unsigned i = 0; i < 2U; ++i) {
        independent_voice(bases[i]);
        bounds_and_targets(bases[i]);
        deadlines(bases[i]);
        faults(bases[i]);
    }
    printf("PASS independent actuators: %u checks; actual app/drivers, simulated HAL only.\n", checks);
    return 0;
}
