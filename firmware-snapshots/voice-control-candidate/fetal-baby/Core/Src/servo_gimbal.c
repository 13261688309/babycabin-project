#include "servo_gimbal.h"

static uint8_t servo_is_valid_channel(uint32_t channel)
{
    switch (channel) {
    case TIM_CHANNEL_1:
    case TIM_CHANNEL_2:
    case TIM_CHANNEL_3:
    case TIM_CHANNEL_4:
        return 1U;
#ifdef TIM_CHANNEL_5
    case TIM_CHANNEL_5:
        return 1U;
#endif
#ifdef TIM_CHANNEL_6
    case TIM_CHANNEL_6:
        return 1U;
#endif
    default:
        return 0U;
    }
}

static uint8_t servo_is_valid_handle(const Servo_HandleTypeDef *hservo)
{
    return (hservo != NULL &&
            hservo->htim != NULL &&
            servo_is_valid_channel(hservo->channel) &&
            hservo->period_us > 0U &&
            hservo->min_pulse_us < hservo->max_pulse_us &&
            hservo->min_angle < hservo->max_angle &&
            hservo->soft_min_angle >= hservo->min_angle &&
            hservo->soft_max_angle <= hservo->max_angle &&
            hservo->soft_min_angle < hservo->soft_max_angle &&
            hservo->step_degrees > 0U && hservo->step_period_ms > 0U);
}

static uint8_t servo_clamp_angle(const Servo_HandleTypeDef *hservo, uint8_t angle)
{
    if (angle < hservo->soft_min_angle) {
        return hservo->soft_min_angle;
    }

    if (angle > hservo->soft_max_angle) {
        return hservo->soft_max_angle;
    }

    return angle;
}

static uint16_t servo_angle_to_pulse_us(const Servo_HandleTypeDef *hservo, uint8_t angle)
{
    const uint32_t angle_span = (uint32_t)hservo->max_angle - hservo->min_angle;
    const uint32_t pulse_span = (uint32_t)hservo->max_pulse_us - hservo->min_pulse_us;
    const uint32_t angle_offset = (uint32_t)angle - hservo->min_angle;

    return (uint16_t)(hservo->min_pulse_us + ((pulse_span * angle_offset) / angle_span));
}

static uint32_t servo_pulse_us_to_compare(const Servo_HandleTypeDef *hservo, uint16_t pulse_us)
{
    const uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(hservo->htim) + 1U;

    return ((uint32_t)pulse_us * period_counts) / hservo->period_us;
}

static uint8_t servo_reverse_angle(const Servo_HandleTypeDef *hservo, uint8_t angle)
{
    return (uint8_t)(hservo->min_angle + hservo->max_angle - angle);
}

Servo_StatusTypeDef Servo_Init(Servo_HandleTypeDef *hservo,
                               TIM_HandleTypeDef *htim,
                               uint32_t channel)
{
    if (hservo == NULL || htim == NULL || !servo_is_valid_channel(channel)) {
        return SERVO_ERROR_PARAM;
    }

    hservo->htim = htim;
    hservo->channel = channel;
    hservo->min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
    hservo->max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;
    hservo->period_us = SERVO_DEFAULT_PERIOD_US;
    hservo->min_angle = SERVO_DEFAULT_MIN_ANGLE;
    hservo->max_angle = SERVO_DEFAULT_MAX_ANGLE;
    hservo->soft_min_angle = SERVO_DEFAULT_MIN_ANGLE;
    hservo->soft_max_angle = SERVO_DEFAULT_MAX_ANGLE;
    hservo->current_angle = SERVO_DEFAULT_CENTER_ANGLE;
    hservo->target_angle = SERVO_DEFAULT_CENTER_ANGLE;
    hservo->step_degrees = SERVO_DEFAULT_STEP_DEGREES;
    hservo->step_period_ms = SERVO_DEFAULT_STEP_MS;
    hservo->last_step_ms = HAL_GetTick();
    hservo->running = 0U;

    return Servo_SetAngle(hservo, hservo->current_angle);
}

Servo_StatusTypeDef Servo_Start(Servo_HandleTypeDef *hservo)
{
    if (!servo_is_valid_handle(hservo)) {
        return SERVO_ERROR_PARAM;
    }

    if (HAL_TIM_PWM_Start(hservo->htim, hservo->channel) != HAL_OK) {
        return SERVO_ERROR_HAL;
    }
    hservo->last_step_ms = HAL_GetTick();
    hservo->running = 1U;
    return SERVO_OK;
}

Servo_StatusTypeDef Servo_Stop(Servo_HandleTypeDef *hservo)
{
    if (!servo_is_valid_handle(hservo)) {
        return SERVO_ERROR_PARAM;
    }

    if (HAL_TIM_PWM_Stop(hservo->htim, hservo->channel) != HAL_OK) {
        return SERVO_ERROR_HAL;
    }
    hservo->running = 0U;
    hservo->target_angle = hservo->current_angle;
    return SERVO_OK;
}

Servo_StatusTypeDef Servo_SetAngle(Servo_HandleTypeDef *hservo, uint8_t angle)
{
    uint16_t pulse_us;

    if (!servo_is_valid_handle(hservo)) {
        return SERVO_ERROR_PARAM;
    }

    angle = servo_clamp_angle(hservo, angle);
    pulse_us = servo_angle_to_pulse_us(hservo, angle);
    hservo->current_angle = angle;
    hservo->target_angle = angle;

    return Servo_SetPulseUs(hservo, pulse_us);
}

Servo_StatusTypeDef Servo_SetPulseUs(Servo_HandleTypeDef *hservo, uint16_t pulse_us)
{
    uint32_t compare;

    if (!servo_is_valid_handle(hservo)) {
        return SERVO_ERROR_PARAM;
    }

    if (pulse_us < hservo->min_pulse_us) {
        pulse_us = hservo->min_pulse_us;
    } else if (pulse_us > hservo->max_pulse_us) {
        pulse_us = hservo->max_pulse_us;
    }

    compare = servo_pulse_us_to_compare(hservo, pulse_us);
    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, compare);

    return SERVO_OK;
}

void Servo_SetPulseRange(Servo_HandleTypeDef *hservo, uint16_t min_pulse_us, uint16_t max_pulse_us)
{
    if (hservo == NULL || min_pulse_us >= max_pulse_us) {
        return;
    }

    hservo->min_pulse_us = min_pulse_us;
    hservo->max_pulse_us = max_pulse_us;
    (void)Servo_SetAngle(hservo, hservo->current_angle);
}

void Servo_SetAngleRange(Servo_HandleTypeDef *hservo, uint8_t min_angle, uint8_t max_angle)
{
    if (hservo == NULL || min_angle >= max_angle) {
        return;
    }

    hservo->min_angle = min_angle;
    hservo->max_angle = max_angle;
    if (hservo->soft_min_angle < min_angle) { hservo->soft_min_angle = min_angle; }
    if (hservo->soft_max_angle > max_angle) { hservo->soft_max_angle = max_angle; }
    if (hservo->soft_min_angle >= hservo->soft_max_angle) {
        hservo->soft_min_angle = min_angle;
        hservo->soft_max_angle = max_angle;
    }
    (void)Servo_SetAngle(hservo, hservo->current_angle);
}

void Servo_SetPeriodUs(Servo_HandleTypeDef *hservo, uint16_t period_us)
{
    if (hservo == NULL || period_us == 0U) {
        return;
    }

    hservo->period_us = period_us;
    (void)Servo_SetAngle(hservo, hservo->current_angle);
}

uint8_t Servo_GetAngle(const Servo_HandleTypeDef *hservo)
{
    if (hservo == NULL) {
        return 0U;
    }

    return hservo->current_angle;
}

uint8_t Servo_GetTargetAngle(const Servo_HandleTypeDef *hservo)
{
    return hservo == NULL ? 0U : hservo->target_angle;
}

Servo_StatusTypeDef Servo_SetSoftLimits(Servo_HandleTypeDef *hservo, uint8_t min_angle, uint8_t max_angle)
{
    if (hservo == NULL || min_angle >= max_angle || min_angle < hservo->min_angle || max_angle > hservo->max_angle) {
        return SERVO_ERROR_PARAM;
    }
    hservo->soft_min_angle = min_angle;
    hservo->soft_max_angle = max_angle;
    hservo->target_angle = servo_clamp_angle(hservo, hservo->target_angle);
    if (hservo->current_angle < min_angle || hservo->current_angle > max_angle) {
        return Servo_SetAngle(hservo, hservo->current_angle);
    }
    return SERVO_OK;
}

Servo_StatusTypeDef Servo_SetMotionProfile(Servo_HandleTypeDef *hservo, uint8_t step_degrees, uint16_t step_period_ms)
{
    if (hservo == NULL || step_degrees == 0U || step_period_ms == 0U) { return SERVO_ERROR_PARAM; }
    hservo->step_degrees = step_degrees;
    hservo->step_period_ms = step_period_ms;
    return SERVO_OK;
}

Servo_StatusTypeDef Servo_SetTargetAngle(Servo_HandleTypeDef *hservo, uint8_t angle)
{
    if (!servo_is_valid_handle(hservo)) { return SERVO_ERROR_PARAM; }
    hservo->target_angle = servo_clamp_angle(hservo, angle);
    return SERVO_OK;
}

Servo_StatusTypeDef Servo_Task(Servo_HandleTypeDef *hservo, uint32_t now_ms)
{
    uint8_t next;
    uint8_t target;
    Servo_StatusTypeDef status;
    if (!servo_is_valid_handle(hservo)) { return SERVO_ERROR_PARAM; }
    if (hservo->running == 0U || hservo->current_angle == hservo->target_angle ||
        (now_ms - hservo->last_step_ms) < hservo->step_period_ms) { return SERVO_OK; }
    hservo->last_step_ms = now_ms;
    target = hservo->target_angle;
    if (hservo->current_angle < target) {
        uint16_t candidate = (uint16_t)hservo->current_angle + hservo->step_degrees;
        next = candidate > target ? target : (uint8_t)candidate;
    } else {
        next = hservo->current_angle - target <= hservo->step_degrees
            ? target : (uint8_t)(hservo->current_angle - hservo->step_degrees);
    }
    status = Servo_SetAngle(hservo, next);
    hservo->target_angle = target;
    return status;
}

Servo_StatusTypeDef ServoGimbal_Init(ServoGimbal_HandleTypeDef *hgimbal,
                                     TIM_HandleTypeDef *pan_htim,
                                     uint32_t pan_channel,
                                     TIM_HandleTypeDef *tilt_htim,
                                     uint32_t tilt_channel)
{
    Servo_StatusTypeDef status;

    if (hgimbal == NULL) {
        return SERVO_ERROR_PARAM;
    }

    status = Servo_Init(&hgimbal->pan, pan_htim, pan_channel);
    if (status != SERVO_OK) {
        return status;
    }

    status = Servo_Init(&hgimbal->tilt, tilt_htim, tilt_channel);
    if (status != SERVO_OK) {
        return status;
    }

    hgimbal->tilt_reverse = 1U;

    return SERVO_OK;
}

Servo_StatusTypeDef ServoGimbal_Start(ServoGimbal_HandleTypeDef *hgimbal)
{
    Servo_StatusTypeDef status;

    if (hgimbal == NULL) {
        return SERVO_ERROR_PARAM;
    }

    status = Servo_Start(&hgimbal->pan);
    if (status != SERVO_OK) {
        return status;
    }

    status = Servo_Start(&hgimbal->tilt);
    if (status != SERVO_OK) { (void)Servo_Stop(&hgimbal->pan); }
    return status;
}

Servo_StatusTypeDef ServoGimbal_Stop(ServoGimbal_HandleTypeDef *hgimbal)
{
    Servo_StatusTypeDef status;
    Servo_StatusTypeDef tilt_status;

    if (hgimbal == NULL) {
        return SERVO_ERROR_PARAM;
    }

    status = Servo_Stop(&hgimbal->pan);
    tilt_status = Servo_Stop(&hgimbal->tilt);
    return status != SERVO_OK ? status : tilt_status;
}

Servo_StatusTypeDef ServoGimbal_SetAngles(ServoGimbal_HandleTypeDef *hgimbal,
                                          uint8_t pan_angle,
                                          uint8_t tilt_angle)
{
    Servo_StatusTypeDef status;

    if (hgimbal == NULL) {
        return SERVO_ERROR_PARAM;
    }

    status = Servo_SetTargetAngle(&hgimbal->pan, pan_angle);
    if (status != SERVO_OK) {
        return status;
    }

    if (hgimbal->tilt_reverse) {
        tilt_angle = servo_reverse_angle(&hgimbal->tilt, tilt_angle);
    }

    return Servo_SetTargetAngle(&hgimbal->tilt, tilt_angle);
}

Servo_StatusTypeDef ServoGimbal_SetByIndex(ServoGimbal_HandleTypeDef *hgimbal,
                                           uint8_t index,
                                           uint8_t angle)
{
    if (hgimbal == NULL) {
        return SERVO_ERROR_PARAM;
    }

    if (index == 0U) {
        return Servo_SetTargetAngle(&hgimbal->pan, angle);
    }

    if (index == 1U) {
        if (hgimbal->tilt_reverse) {
            angle = servo_reverse_angle(&hgimbal->tilt, angle);
        }
        return Servo_SetTargetAngle(&hgimbal->tilt, angle);
    }

    return SERVO_ERROR_PARAM;
}

Servo_StatusTypeDef ServoGimbal_ParseRemoteFrame(ServoGimbal_HandleTypeDef *hgimbal,
                                                 const uint8_t *frame,
                                                 uint16_t len)
{
    uint8_t index;
    uint8_t angle;

    if (hgimbal == NULL || ServoGimbal_DecodeRemoteFrame(frame, len, &index, &angle) != SERVO_OK) {
        return SERVO_ERROR_PARAM;
    }
    return ServoGimbal_SetByIndex(hgimbal, index, angle);
}

Servo_StatusTypeDef ServoGimbal_DecodeRemoteFrame(const uint8_t *frame, uint16_t len,
                                                 uint8_t *index, uint8_t *angle)
{
    uint16_t parsed;

    if (frame == NULL || index == NULL || angle == NULL || len != 4U) {
        return SERVO_ERROR_PARAM;
    }

    if (frame[0] != 'A' && frame[0] != 'B') {
        return SERVO_ERROR_PARAM;
    }

    if (frame[1] < '0' || frame[1] > '9' ||
        frame[2] < '0' || frame[2] > '9' ||
        frame[3] < '0' || frame[3] > '9') {
        return SERVO_ERROR_PARAM;
    }

    parsed = (uint16_t)((frame[1] - '0') * 100U +
                       (frame[2] - '0') * 10U +
                       (frame[3] - '0'));

    if (parsed > SERVO_DEFAULT_MAX_ANGLE) {
        return SERVO_ERROR_PARAM;
    }
    /* Yahboom APP: A is vertical/tilt, B is horizontal/pan. */
    *index = (frame[0] == 'A') ? 1U : 0U;
    *angle = (uint8_t)parsed;
    return SERVO_OK;
}

void ServoGimbal_SetTiltReverse(ServoGimbal_HandleTypeDef *hgimbal, uint8_t enable)
{
    if (hgimbal == NULL) {
        return;
    }

    hgimbal->tilt_reverse = enable ? 1U : 0U;
}

Servo_StatusTypeDef ServoGimbal_SetSoftLimits(ServoGimbal_HandleTypeDef *hgimbal,
                                              uint8_t pan_min, uint8_t pan_max,
                                              uint8_t tilt_min, uint8_t tilt_max)
{
    Servo_StatusTypeDef status;
    if (hgimbal == NULL || pan_min >= pan_max || tilt_min >= tilt_max ||
        pan_min < hgimbal->pan.min_angle || pan_max > hgimbal->pan.max_angle ||
        tilt_min < hgimbal->tilt.min_angle || tilt_max > hgimbal->tilt.max_angle) {
        return SERVO_ERROR_PARAM;
    }
    status = Servo_SetSoftLimits(&hgimbal->pan, pan_min, pan_max);
    if (status != SERVO_OK) { return status; }
    return Servo_SetSoftLimits(&hgimbal->tilt, tilt_min, tilt_max);
}

Servo_StatusTypeDef ServoGimbal_SetMotionProfile(ServoGimbal_HandleTypeDef *hgimbal,
                                                 uint8_t step_degrees,
                                                 uint16_t step_period_ms)
{
    Servo_StatusTypeDef status;
    if (hgimbal == NULL || step_degrees == 0U || step_period_ms == 0U) { return SERVO_ERROR_PARAM; }
    status = Servo_SetMotionProfile(&hgimbal->pan, step_degrees, step_period_ms);
    if (status != SERVO_OK) { return status; }
    return Servo_SetMotionProfile(&hgimbal->tilt, step_degrees, step_period_ms);
}

Servo_StatusTypeDef ServoGimbal_Task(ServoGimbal_HandleTypeDef *hgimbal, uint32_t now_ms)
{
    Servo_StatusTypeDef pan_status;
    Servo_StatusTypeDef tilt_status;
    if (hgimbal == NULL) { return SERVO_ERROR_PARAM; }
    pan_status = Servo_Task(&hgimbal->pan, now_ms);
    tilt_status = Servo_Task(&hgimbal->tilt, now_ms);
    return pan_status != SERVO_OK ? pan_status : tilt_status;
}
