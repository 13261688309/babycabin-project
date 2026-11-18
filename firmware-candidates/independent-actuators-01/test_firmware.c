#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "baby_care_app.h"
#include "dht11.h"
#include "dc_motor_pwm.h"
#include "servo_gimbal.h"
#include "voice_interaction.h"
#include "esp32_camera_link.h"
#include "bsp/bsp.h"
#include "inc/app_config.h"
#include "protocol/device_protocol.h"
#include "services/state_machine.h"
#include "services/power_manager.h"
#include "services/alert_manager.h"
#include "drivers/comm_bridge.h"
#include "debug_uart_tx.h"
#include "actuator_interlock.h"
#include "system_heartbeat.h"

GPIO_TypeDef test_gpio[8];
uint32_t test_primask;
TIM_HandleTypeDef htim2 = {99U}, htim3 = {19999U};
UART_HandleTypeDef huart1, huart2, huart3;
static uint32_t tick;
static uint32_t alerts;
static char report[PROTOCOL_BUFFER_SIZE];
static uint32_t report_count;
static uint32_t unsafe_direction_changes;
static HAL_StatusTypeDef pwm_result;
static TIM_HandleTypeDef *start_fail_timer;
static uint32_t start_fail_channel;
static uint8_t check_motor_gpio_init;
static uint32_t motor_gpio_init_calls, pwm_stop_calls;
static uint8_t override_motor_stop;
static HAL_StatusTypeDef motor_stop_result;
static HAL_StatusTypeDef tx_result, tx_abort_result;
static HAL_StatusTypeDef rx_result, rx_abort_result;
static uint32_t rx_calls, rx_primask;
static uint8_t *tx_data;
static uint16_t tx_length;
static uint8_t *voice_tx_data, *esp_tx_data;
static uint16_t voice_tx_length, esp_tx_length;
static uint32_t tx_calls, debug_blocking_calls;
static uint32_t esp_tx_calls;
static uint32_t voice_tx_calls;
static uint8_t simulate_arm_latch;
static uint32_t heartbeat_writes;
static uint32_t servo_start_advance[7], servo_started_at[7];
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    if (check_motor_gpio_init && port == GPIOA &&
        (gpio->Pin == GPIO_PIN_4 || gpio->Pin == GPIO_PIN_5)) {
        assert((htim2.enabled & (1U << TIM_CHANNEL_3)) == 0U);
        assert(htim2.compare[TIM_CHANNEL_3] == 0U);
        assert((GPIOA->ODR & (GPIO_PIN_4 | GPIO_PIN_5)) == 0U);
        motor_gpio_init_calls++;
    }
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    uint32_t next = state ? port->ODR | pin : port->ODR & ~pin;
    if (port == GPIOA && (pin == GPIO_PIN_4 || pin == GPIO_PIN_5) &&
        next != port->ODR && htim2.compare[TIM_CHANNEL_3] != 0U) { unsafe_direction_changes++; }
    port->ODR = next;
    if (port == BABY_CARE_HEARTBEAT_PORT && pin == BABY_CARE_HEARTBEAT_PIN) {
        heartbeat_writes++;
    }
    if (simulate_arm_latch && port == BABY_CARE_INTERLOCK_PORT &&
        pin == BABY_CARE_INTERLOCK_READY_PIN && state == GPIO_PIN_RESET) {
        port->ODR &= ~BABY_CARE_INTERLOCK_ARMED_PIN;
    }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) { return (port->ODR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    if (timer == start_fail_timer && channel == start_fail_channel) { return HAL_ERROR; }
    if (pwm_result == HAL_OK) { timer->enabled |= 1U << channel; }
    if (timer == &htim3 && (channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2)) {
        tick += servo_start_advance[channel];
        servo_start_advance[channel] = 0U;
        servo_started_at[channel] = tick;
    }
    return pwm_result;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
    HAL_StatusTypeDef result = (override_motor_stop && timer == &htim2) ? motor_stop_result : pwm_result;
    pwm_stop_calls++;
    if (result == HAL_OK) { timer->enabled &= ~(1U << channel); }
    return result;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len, uint32_t timeout)
{ if (uart == &huart1) { debug_blocking_calls++; } (void)data; (void)len; (void)timeout; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{
    tx_calls++;
    if (tx_result == HAL_OK) {
        if (uart == &huart1) { tx_data = data; tx_length = len; }
        else if (uart == &huart2) { voice_tx_data = data; voice_tx_length = len; voice_tx_calls++; }
        else if (uart == &huart3) { esp_tx_data = data; esp_tx_length = len; esp_tx_calls++; }
        else { assert(0); }
    }
    return tx_result;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{
    if (tx_abort_result == HAL_OK) {
        if (uart == &huart1) { tx_data = NULL; tx_length = 0U; }
        else if (uart == &huart2) { voice_tx_data = NULL; voice_tx_length = 0U; }
        else if (uart == &huart3) { esp_tx_data = NULL; esp_tx_length = 0U; }
        else { assert(0); }
    }
    return tx_abort_result;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{
    assert(len == 1U);
    rx_calls++;
    rx_primask = test_primask;
    if (rx_result == HAL_OK) { uart->rx = data; }
    return rx_result;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    uart->abort_count++;
    if (rx_abort_result == HAL_OK) { uart->rx = NULL; }
    return rx_abort_result;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i, uint16_t a, uint16_t r, uint16_t s, uint8_t *d, uint16_t l, uint32_t t)
{ (void)i; (void)a; (void)r; (void)s; (void)d; (void)l; (void)t; return HAL_ERROR; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i, uint16_t a, uint16_t r, uint16_t s, uint8_t *d, uint16_t l, uint32_t t)
{ return HAL_I2C_Mem_Write(i,a,r,s,d,l,t); }
static uint8_t fake_dht_pending, fake_dht_start_busy;
static uint32_t fake_dht_started_ms, fake_dht_requests;
void DHT11_Init(DHT11_HandleTypeDef *h, GPIO_TypeDef *p, uint16_t pin)
{ (void)h; (void)p; (void)pin; fake_dht_pending=0U; fake_dht_start_busy=0U; fake_dht_requests=0U; }
DHT11_StatusTypeDef DHT11_StartRead(DHT11_HandleTypeDef *h)
{ (void)h; fake_dht_requests++; if (fake_dht_pending) return DHT11_ERROR_PARAM;
  if (fake_dht_start_busy) return DHT11_BUSY;
  fake_dht_pending=1U; fake_dht_started_ms=tick; return DHT11_OK; }
DHT11_StatusTypeDef DHT11_Task(DHT11_HandleTypeDef *h, DHT11_DataTypeDef *d)
{ (void)h; (void)d; if (!fake_dht_pending) return DHT11_ERROR_PARAM;
  if ((tick - fake_dht_started_ms) < 20U) return DHT11_BUSY;
  fake_dht_pending=0U; return DHT11_ERROR_TIMEOUT; }
DHT11_StatusTypeDef DHT11_Read(DHT11_HandleTypeDef *h, DHT11_DataTypeDef *d) { (void)h; (void)d; return DHT11_ERROR_TIMEOUT; }
float DHT11_GetTemperature(const DHT11_DataTypeDef *d) { (void)d; return 0.0f; }
float DHT11_GetHumidity(const DHT11_DataTypeDef *d) { (void)d; return 0.0f; }
void bsp_init(void) {}
uint32_t bsp_millis(void) { return tick; }
void bsp_send_packet(const char *s) { assert(strlen(s) < sizeof(report)); strcpy(report,s); report_count++; }
void bsp_send_alert_packet(const char *s) { assert(strstr(s,"alert") != NULL); alerts++; }

static system_snapshot_t healthy(void)
{
    system_snapshot_t s = {0};
    s.temperature_c = 25.0f;
    s.humidity_rh = 50.0f;
    s.dht11_health = SENSOR_HEALTH_OK;
    s.microphone_health = SENSOR_HEALTH_OK;
    s.mpu6050_health = SENSOR_HEALTH_OK;
    s.presence_health = SENSOR_HEALTH_OK;
    s.baby_in_bed = true;
    return s;
}

static void test_state_machine(void)
{
    state_machine_t m;
    system_snapshot_t s = {0};
    state_machine_init(&m);
    assert(state_machine_update(&m, &s) == BABY_STATUS_DATA_UNAVAILABLE);
    s.timestamp_ms = SLEEP_QUIET_MS + 1000U;
    assert(state_machine_update(&m, &s) == BABY_STATUS_DATA_UNAVAILABLE);
    assert(!power_manager_should_sleep(&s));
    s = healthy();
    state_machine_init(&m);
    assert(state_machine_update(&m, &s) == BABY_STATUS_NORMAL);
    s.timestamp_ms = SLEEP_QUIET_MS;
    assert(state_machine_update(&m, &s) == BABY_STATUS_SLEEPY);
    s.microphone_health = SENSOR_HEALTH_STALE;
    assert(state_machine_update(&m, &s) == BABY_STATUS_SENSOR_FAULT);
    assert(!m.quiet_tracking);
    assert(!power_manager_should_sleep(&s));
    s = healthy();
    s.temperature_c = 38.0f;
    assert(state_machine_update(&m, &s) == BABY_STATUS_ENV_WARNING);
    s.temperature_c = 25.0f;
    s.ir_temp_health = SENSOR_HEALTH_OK;
    s.ir_temp_object_c = TEMP_ALERT_C;
    assert(state_machine_update(&m, &s) == BABY_STATUS_TEMP_ALERT);
    s = healthy();
    s.wet_detected = true;
    s.air_quality_voc = UINT16_MAX;
    s.camera_detected = true;
    s.ir_temp_object_c = 100.0f;
    state_machine_init(&m);
    assert(state_machine_update(&m, &s) == BABY_STATUS_NORMAL);
    s.temperature_c = NAN;
    assert(state_machine_update(&m, &s) == BABY_STATUS_SENSOR_FAULT);
    s = healthy();
    s.sound_level = 0.9f;
    s.motion_active = true;
    s.timestamp_ms = UINT32_MAX - 1000U;
    state_machine_init(&m);
    assert(state_machine_update(&m, &s) == BABY_STATUS_NORMAL);
    s.timestamp_ms += CRY_SUSTAIN_MS;
    assert(state_machine_update(&m, &s) == BABY_STATUS_ABNORMAL_CRYING);
    s.microphone_health = SENSOR_HEALTH_DISABLED;
    assert(state_machine_update(&m, &s) == BABY_STATUS_DATA_UNAVAILABLE);
    puts("PASS state: unavailable, failure, thermal semantics, zero-tick and wraparound");
}

static void test_protocol(void)
{
    system_snapshot_t s = {0};
    char buffer[PROTOCOL_BUFFER_SIZE];
    struct { char before; char text[16]; char after; } small = {'X', {0}, 'Y'};
    device_command_t commands[4];
    uint32_t durations[4];
    size_t length;
    s.timestamp_ms = 10000U;
    s.dht11_health = SENSOR_HEALTH_READ_FAILED;
    s.status = BABY_STATUS_SENSOR_FAULT;
    length = device_protocol_encode_snapshot(&s, buffer, sizeof(buffer));
    assert(length > 512U && length == strlen(buffer));
    assert(strstr(buffer, "\"temp\":null") && strstr(buffer, "\"babyInBed\":null"));
    assert(strstr(buffer, "\"cameraDetected\":null"));
    assert(device_protocol_encode_snapshot(&s, small.text, sizeof(small.text)) == 0U);
    assert(small.text[0] == '\0' && small.before == 'X' && small.after == 'Y');
    assert(device_protocol_encode_alert(BABY_STATUS_SENSOR_FAULT,&s,small.text,sizeof(small.text)) == 0U);
    s = healthy();
    s.timestamp_ms = UINT32_MAX;
    s.temperature_c = FLT_MAX;
    s.humidity_rh = -FLT_MAX;
    s.sound_level = FLT_MAX;
    s.ir_temp_health = SENSOR_HEALTH_OK;
    s.ir_temp_object_c = FLT_MAX;
    s.ir_temp_ambient_c = -FLT_MAX;
    s.camera_x = s.camera_y = s.camera_area = UINT16_MAX;
    s.camera_id = INT16_MIN;
    length = device_protocol_encode_snapshot(&s,buffer,sizeof(buffer));
    assert(length > 0U && length < sizeof(buffer));
    s.temperature_c = NAN;
    s.humidity_rh = INFINITY;
    assert(device_protocol_encode_snapshot(&s,buffer,sizeof(buffer)) > 0U);
    assert(strstr(buffer,"\"temp\":null") && strstr(buffer,"\"hum\":null"));

    assert(device_protocol_parse_commands(
        " {\"command\":\"start_rocking\",\"durationSec\":5,\"id\":17} ",
        commands, durations, 4U) == 1U);
    assert(commands[0] == DEVICE_CMD_START_ROCKING && durations[0] == 5000U);
    assert(device_protocol_parse_commands(
        "{\"commands\":[{\"command\":\"stop_rocking\"},"
        "{\"command\":\"start_vibration\",\"durationMs\":1234}]}",
        commands, durations, 4U) == 2U);
    assert(commands[0] == DEVICE_CMD_STOP_ROCKING && durations[0] == 0U);
    assert(commands[1] == DEVICE_CMD_START_VIBRATION && durations[1] == 1234U);

    {
        static const char *invalid[] = {
            "{\"note\":\"start_rocking\"}",
            "{\"command\":\"start_rocking_extra\"}",
            "{\"command\":\"start_rocking\",\"durationMs\":0}",
            "{\"command\":\"start_rocking\",\"durationMs\":300001}",
            "{\"command\":\"start_rocking\",\"durationMs\":1,\"durationSec\":1}",
            "{\"command\":\"stop_rocking\",\"durationMs\":1}",
            "{\"command\":\"start_rocking\",\"command\":\"stop_rocking\"}",
            "{\"command\":\"start_rocking\"} trailing",
            "{\"command\":\"start_rocking\"",
            "\"start_rocking\""
        };
        size_t index;
        for (index = 0U; index < sizeof(invalid) / sizeof(invalid[0]); index++)
        {
            assert(device_protocol_parse_commands(invalid[index], commands, durations, 4U) == 0U);
        }
    }
    assert(device_protocol_parse_commands(
        "[{\"command\":\"stop_rocking\"},{\"command\":\"stop_vibration\"}]",
        commands, durations, 1U) == 0U);
    puts("PASS protocol: bounded encoding and strict structured command parsing");
}

static void test_motor_initialization(void)
{
    DC_Motor_HandleTypeDef motor;
    pwm_result = HAL_OK;
    GPIOA->ODR = GPIO_PIN_4 | GPIO_PIN_5;
    htim2.enabled = 1U << TIM_CHANNEL_3;
    htim2.compare[TIM_CHANNEL_3] = 60U;
    check_motor_gpio_init = 1U;
    motor_gpio_init_calls = 0U;
    assert(DC_Motor_Init(&motor, &htim2, TIM_CHANNEL_3,
                         GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5) == DC_MOTOR_OK);
    assert(motor_gpio_init_calls == 2U);
    assert(motor.initialized == 1U && motor.running == 0U && motor.speed == 0 && motor.target_speed == 0);
    check_motor_gpio_init = 0U;
    puts("PASS motor init: PWM disabled and low ODR preloaded before GPIO mode changes");
}

static void test_motor_init_failures(void)
{
    DC_Motor_HandleTypeDef motor;
    uint32_t old_stop_calls;
    HAL_StatusTypeDef errors[] = {HAL_BUSY, HAL_ERROR, HAL_TIMEOUT};
    check_motor_gpio_init = 1U;
    for (size_t i = 0U; i < sizeof(errors) / sizeof(errors[0]); i++) {
        pwm_result = errors[i];
        GPIOA->ODR = GPIO_PIN_4 | GPIO_PIN_5;
        htim2.enabled = 1U << TIM_CHANNEL_3;
        htim2.compare[TIM_CHANNEL_3] = 60U;
        motor_gpio_init_calls = 0U;
        old_stop_calls = pwm_stop_calls;
        assert(DC_Motor_Init(&motor, &htim2, TIM_CHANNEL_3,
                            GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5) == DC_MOTOR_ERROR_HAL);
        assert(pwm_stop_calls == old_stop_calls + 1U);
        assert(motor_gpio_init_calls == 0U);
        assert(GPIOA->ODR == (GPIO_PIN_4 | GPIO_PIN_5));
        assert(htim2.enabled == (1U << TIM_CHANNEL_3));
        assert(motor.initialized == 0U && motor.running == 0U);
        assert(DC_Motor_Start(&motor) == DC_MOTOR_ERROR_PARAM);
        assert(DC_Motor_SetSpeed(&motor, 20) == DC_MOTOR_ERROR_PARAM);
        assert(DC_Motor_SetStopMode(&motor, DC_MOTOR_STOP_INPUTS_HIGH) == DC_MOTOR_ERROR_PARAM);
        tick += 100U;
        DC_Motor_Task(&motor);
        assert(GPIOA->ODR == (GPIO_PIN_4 | GPIO_PIN_5));
        assert(htim2.enabled == (1U << TIM_CHANNEL_3));
    }
    pwm_result = HAL_OK;
    old_stop_calls = pwm_stop_calls;
    assert(DC_Motor_Init(&motor, &htim2, TIM_CHANNEL_3, GPIOA,
                        GPIO_PIN_4 | GPIO_PIN_5, GPIOA, GPIO_PIN_5) == DC_MOTOR_ERROR_PARAM);
    assert(DC_Motor_Init(&motor, &htim2, TIM_CHANNEL_3, GPIOA,
                        GPIO_PIN_4, GPIOA, GPIO_PIN_4) == DC_MOTOR_ERROR_PARAM);
    assert(DC_Motor_Init(&motor, &htim2, TIM_CHANNEL_3, GPIOA,
                        0U, GPIOA, GPIO_PIN_5) == DC_MOTOR_ERROR_PARAM);
    assert(pwm_stop_calls == old_stop_calls);
    assert(GPIOA->ODR == (GPIO_PIN_4 | GPIO_PIN_5));
    assert(DC_Motor_Init(&motor, &htim2, TIM_CHANNEL_3,
                        GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5) == DC_MOTOR_OK);
    check_motor_gpio_init = 0U;
    puts("PASS motor init failures: no GPIO reconfiguration after stop errors or invalid pin bindings");
}

static void test_motor_stop_patterns(void)
{
    DC_Motor_HandleTypeDef motor, invalid = {0};
    pwm_result = HAL_OK;
    assert(DC_Motor_Init(&motor, &htim2, TIM_CHANNEL_3,
                        GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5) == DC_MOTOR_OK);
    assert(DC_Motor_SetStopMode(NULL, DC_MOTOR_STOP_INPUTS_HIGH) == DC_MOTOR_ERROR_PARAM);
    assert(DC_Motor_SetStopMode(&invalid, DC_MOTOR_STOP_INPUTS_HIGH) == DC_MOTOR_ERROR_PARAM);
    assert(DC_Motor_SetStopMode(&motor, (DC_Motor_StopModeTypeDef)-1) == DC_MOTOR_ERROR_PARAM);
    assert(DC_Motor_SetStopMode(&motor, (DC_Motor_StopModeTypeDef)2) == DC_MOTOR_ERROR_PARAM);
    assert(motor.stop_mode == DC_MOTOR_STOP_INPUTS_LOW);
    assert((GPIOA->ODR & (GPIO_PIN_4 | GPIO_PIN_5)) == 0U);
    assert(DC_Motor_SetStopMode(&motor, DC_MOTOR_STOP_INPUTS_HIGH) == DC_MOTOR_OK);
    assert((GPIOA->ODR & (GPIO_PIN_4 | GPIO_PIN_5)) == (GPIO_PIN_4 | GPIO_PIN_5));
    assert(htim2.compare[TIM_CHANNEL_3] == 0U);
    assert(DC_Motor_Start(&motor) == DC_MOTOR_OK);
    assert(DC_Motor_SetSpeed(&motor, 10) == DC_MOTOR_OK);
    tick += 10U;
    DC_Motor_Task(&motor);
    assert(motor.speed == 5);
    assert(DC_Motor_SetStopMode(&motor, DC_MOTOR_STOP_INPUTS_LOW) == DC_MOTOR_OK);
    assert((GPIOA->ODR & (GPIO_PIN_4 | GPIO_PIN_5)) == GPIO_PIN_4);
    assert(DC_Motor_SetSpeed(&motor, 0) == DC_MOTOR_OK);
    assert((GPIOA->ODR & (GPIO_PIN_4 | GPIO_PIN_5)) == 0U);
    assert(DC_Motor_SetStopMode(&motor, DC_MOTOR_STOP_INPUTS_HIGH) == DC_MOTOR_OK);
    assert(DC_Motor_Stop(&motor) == DC_MOTOR_OK);
    assert((GPIOA->ODR & (GPIO_PIN_4 | GPIO_PIN_5)) == (GPIO_PIN_4 | GPIO_PIN_5));
    assert(htim2.compare[TIM_CHANNEL_3] == 0U && motor.running == 0U);
    puts("PASS motor zero-input patterns: explicit LOW/HIGH, invalid handles rejected, no coast/brake claim");
}

static void test_motor(void)
{
    DC_Motor_HandleTypeDef motor;
    tick = UINT32_MAX - 100U;
    memset(test_gpio,0,sizeof(test_gpio));
    htim2.compare[TIM_CHANNEL_3] = 0U;
    assert(DC_Motor_Init(&motor,&htim2,TIM_CHANNEL_3,GPIOA,GPIO_PIN_4,GPIOA,GPIO_PIN_5) == DC_MOTOR_OK);
    assert(DC_Motor_SetSpeed(&motor,35) == DC_MOTOR_ERROR_PARAM);
    assert(DC_Motor_Start(&motor) == DC_MOTOR_OK);
    assert(DC_Motor_SetSpeed(&motor,35) == DC_MOTOR_OK);
    for (int i=0;i<7;i++) { tick += 10U; DC_Motor_Task(&motor); }
    assert(motor.speed == 35);
    assert(DC_Motor_SetSpeed(&motor,-35) == DC_MOTOR_OK);
    for (int i=0;i<7;i++) { tick += 10U; DC_Motor_Task(&motor); }
    assert(motor.speed == 0 && htim2.compare[TIM_CHANNEL_3] == 0U);
    tick += 49U; DC_Motor_Task(&motor);
    assert(motor.speed == 0);
    tick += 10U; DC_Motor_Task(&motor);
    assert(motor.speed < 0);
    assert(DC_Motor_SetSpeed(&motor,0) == DC_MOTOR_OK);
    assert(motor.speed == 0 && htim2.compare[TIM_CHANNEL_3] == 0U);
    assert(unsafe_direction_changes == 0U);
    assert(DC_Motor_Stop(&motor) == DC_MOTOR_OK);
    tick += 1000U; DC_Motor_Task(&motor);
    assert(motor.speed == 0 && !motor.running);
    puts("PASS motor: ramp, direction changes at zero PWM, pause, immediate stop and tick wrap");
}

static void test_servo(void)
{
    ServoGimbal_HandleTypeDef gimbal;
    uint32_t pan_center;
    tick = UINT32_MAX - 10U;
    htim3.compare[TIM_CHANNEL_1] = htim3.compare[TIM_CHANNEL_2] = 0U;
    htim3.enabled = 0U;
    assert(ServoGimbal_Init(&gimbal,&htim3,TIM_CHANNEL_1,&htim3,TIM_CHANNEL_2) == SERVO_OK);
    pan_center = htim3.compare[TIM_CHANNEL_1];
    assert(pan_center == 1500U);
    assert(ServoGimbal_SetSoftLimits(&gimbal,60,120,75,105) == SERVO_OK);
    assert(ServoGimbal_SetMotionProfile(&gimbal,1,20) == SERVO_OK);
    assert(ServoGimbal_SetSoftLimits(&gimbal,121,120,75,105) == SERVO_ERROR_PARAM);
    assert(gimbal.pan.soft_min_angle == 60U && gimbal.tilt.soft_min_angle == 75U);
    assert(ServoGimbal_Start(&gimbal) == SERVO_OK);
    assert(ServoGimbal_SetByIndex(&gimbal,0,180) == SERVO_OK);
    assert(Servo_GetTargetAngle(&gimbal.pan) == 120U);
    assert(Servo_GetAngle(&gimbal.pan) == 90U && htim3.compare[TIM_CHANNEL_1] == pan_center);
    tick += 19U;
    assert(ServoGimbal_Task(&gimbal,tick) == SERVO_OK && Servo_GetAngle(&gimbal.pan) == 90U);
    tick += 1U;
    assert(ServoGimbal_Task(&gimbal,tick) == SERVO_OK && Servo_GetAngle(&gimbal.pan) == 91U);
    for (int i=0;i<40;i++) { tick += 20U; assert(ServoGimbal_Task(&gimbal,tick) == SERVO_OK); }
    assert(Servo_GetAngle(&gimbal.pan) == 120U && htim3.compare[TIM_CHANNEL_1] == 1833U);
    assert(ServoGimbal_SetByIndex(&gimbal,1,0) == SERVO_OK);
    assert(Servo_GetTargetAngle(&gimbal.tilt) == 105U); /* Reversed, then physically clamped. */
    tick += 20U; assert(ServoGimbal_Task(&gimbal,tick) == SERVO_OK);
    assert(Servo_GetAngle(&gimbal.tilt) == 91U);
    assert(ServoGimbal_Stop(&gimbal) == SERVO_OK);
    assert(htim3.enabled == 0U);
    assert(Servo_GetTargetAngle(&gimbal.pan) == Servo_GetAngle(&gimbal.pan));
    tick += 1000U; assert(ServoGimbal_Task(&gimbal,tick) == SERVO_OK);
    assert(Servo_GetAngle(&gimbal.pan) == 120U);
    puts("PASS servo: independent soft limits, gradual targets, wraparound, reverse and immediate PWM stop");
}

static void voice_feed(Voice_HandleTypeDef *voice, uint8_t id)
{
    const uint8_t bytes[] = {0xAA,0x55,0x00,id,0xFB};
    for (size_t i=0;i<sizeof(bytes);i++) { (void)Voice_UART_ParseByte(voice,bytes[i],NULL); }
}
static void esp_feed(ESP32Cam_HandleTypeDef *esp, const char *text)
{
    for (;*text;text++) { esp->rx_byte = (uint8_t)*text; ESP32Cam_OnUartRxComplete(esp,esp->huart); }
}
static void voice_irq_feed(Voice_HandleTypeDef *voice, uint8_t id)
{
    const uint8_t bytes[] = {0xAA,0x55,0x00,id,0xFB};
    for (size_t i=0;i<sizeof(bytes);i++) {
        voice->rx_byte = bytes[i];
        (void)Voice_UART_RxCpltCallback(voice,voice->huart);
    }
}
static void test_queues(void)
{
    Voice_HandleTypeDef voice;
    Voice_FrameTypeDef frame;
    ESP32Cam_HandleTypeDef esp;
    ESP32Cam_LinkStatusTypeDef status;
    uint8_t remote[16], axis, angle;
    uint16_t length;
    tick = 0U;
    Voice_Init(&voice,&huart2,NULL);
    assert(Voice_UART_PlayBroadcast(&voice, VOICE_LEGACY_INIT) == VOICE_OK);
    assert(voice.tx_busy && voice_tx_length == VOICE_UART_FRAME_LEN);
    assert(!memcmp(voice_tx_data, "\xAA\x55\xFF\x67\xFB", VOICE_UART_FRAME_LEN));
    assert(Voice_UART_PlayBroadcast(&voice, VOICE_LEGACY_INIT) == VOICE_ERROR_BUSY);
    Voice_UART_OnTxComplete(&voice,&huart2); Voice_UART_Task(&voice);
    assert(!voice.tx_busy);
    tick = UINT32_MAX - 50U;
    assert(Voice_UART_PlayBroadcast(&voice, VOICE_LEGACY_INIT) == VOICE_OK);
    tick += VOICE_TX_TIMEOUT_MS; Voice_UART_Task(&voice);
    assert(!voice.tx_busy && voice.tx_timeouts == 1U && voice.tx_failures == 1U);
    tick = 0U;
    voice_feed(&voice,VOICE_CMD_FORWARD);
    voice_feed(&voice,VOICE_CMD_SLEEP);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_OK && frame.id == VOICE_CMD_FORWARD);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_OK && frame.id == VOICE_CMD_SLEEP);
    for (int i=0;i<20;i++) { voice_feed(&voice,VOICE_CMD_FORWARD); }
    voice_feed(&voice,VOICE_CMD_STOP_1);
    voice_feed(&voice,VOICE_CMD_FORWARD);
    assert(voice.dropped_frames != 0U);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_OK && frame.id == VOICE_CMD_STOP_1);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_ERROR_NO_FRAME);
    Voice_UART_OnError(&voice,&huart2);
    Voice_UART_Task(&voice);
    assert(!voice.restart_needed && huart2.abort_count != 0U);
    tick = UINT32_MAX - 50U;
    (void)Voice_UART_ParseByte(&voice,0xAA,NULL);
    (void)Voice_UART_ParseByte(&voice,0x55,NULL);
    tick += VOICE_FRAME_GAP_MS;
    (void)Voice_UART_ParseByte(&voice,0x00,NULL);
    (void)Voice_UART_ParseByte(&voice,VOICE_CMD_FORWARD,NULL);
    (void)Voice_UART_ParseByte(&voice,0xFB,NULL);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_ERROR_NO_FRAME);
    voice_feed(&voice,VOICE_CMD_SLEEP);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_OK && frame.id == VOICE_CMD_SLEEP);
    tick = 0U;
    assert(ESP32Cam_Init(&esp,&huart3) == ESP32_CAM_OK);
    assert(ESP32Cam_RequestStaIp(&esp) == ESP32_CAM_OK);
    assert(esp.tx_busy && esp_tx_length == 6U && !memcmp(esp_tx_data,"sta_ip",6U));
    assert(ESP32Cam_RequestApIp(&esp) == ESP32_CAM_ERROR_BUSY);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    assert(!esp.tx_busy);
    tick = UINT32_MAX - 50U;
    assert(ESP32Cam_RequestApIp(&esp) == ESP32_CAM_OK);
    tick += ESP32_CAM_TX_TIMEOUT_MS; ESP32Cam_Task(&esp);
    assert(!esp.tx_busy && esp.tx_timeouts == 1U && esp.tx_failures == 1U);
    tick = 0U;
    assert(ESP32Cam_Init(&esp,&huart3) == ESP32_CAM_OK);
    assert(ESP32Cam_ConfigureDefault(&esp) == ESP32_CAM_OK);
    tick = 1999U; ESP32Cam_Task(&esp);
    assert(!esp.tx_busy && esp.configure_step == 0U);
    tick = 2000U; ESP32Cam_Task(&esp);
    assert(esp.tx_busy && esp.configure_tx_pending && esp.configure_step == 0U);
    assert(esp_tx_length == 11U && !memcmp(esp_tx_data,"wifi_mode:2",11U));
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    assert(!esp.tx_busy && !esp.configure_tx_pending && esp.configure_step == 0U);
    assert(esp.configure_waiting_ack);
    esp_feed(&esp,"OK\r\n"); ESP32Cam_Task(&esp);
    assert(!esp.configure_waiting_ack && esp.configure_step == 1U);
    assert(ESP32Cam_Init(&esp,&huart3) == ESP32_CAM_OK);
    tick = 0U;
    for (int i=0;i<20;i++) { esp_feed(&esp,"$A090#"); }
    esp_feed(&esp,"$STOP#$A100#");
    assert(ESP32Cam_GetRemoteFrame(&esp,remote,sizeof(remote),&length) == ESP32_CAM_OK);
    assert(length == 4U && !memcmp(remote,"STOP",4U));
    assert(ESP32Cam_GetRemoteFrame(&esp,remote,sizeof(remote),&length) == ESP32_CAM_OK && length == 0U);
    assert(esp.dropped_frames != 0U);
    for (int i=0;i<20;i++) { esp_feed(&esp,"$B100#"); }
    esp_feed(&esp,"$7,0,0,0#$A080#");
    assert(ESP32Cam_GetRemoteFrame(&esp,remote,sizeof(remote),&length) == ESP32_CAM_OK);
    assert(length == 4U && !memcmp(remote,"STOP",4U));
    assert(ESP32Cam_GetRemoteFrame(&esp,remote,sizeof(remote),&length) == ESP32_CAM_OK && length == 0U);
    esp_feed(&esp,"$7,0,0,0x#");
    assert(ESP32Cam_GetRemoteFrame(&esp,remote,sizeof(remote),&length) == ESP32_CAM_OK);
    assert(length == 8U && !memcmp(remote,"7,0,0,0x",8U));
    {
        const char *invalid_ips[] = {"ap_ip:connected\n", "ap_ip:999.168.4.1\n",
            "ap_ip:192.168.4.1junk\n", "ap_ip:192.168.4\n", "ap_ip:0.0.0.0\n",
            "ap_ip:127.0.0.1\n", "ap_ip:192.168.04.1\n", "ap_ip:\n"};
        for (size_t i=0;i<sizeof(invalid_ips)/sizeof(invalid_ips[0]);i++) {
            esp_feed(&esp,invalid_ips[i]);
            assert(ESP32Cam_GetStatus(&esp,&status) == ESP32_CAM_OK && !status.ap_ip_valid);
        }
    }
    esp_feed(&esp,"ap_ip:192.168.4.1\n");
    assert(ESP32Cam_GetStatus(&esp,&status) == ESP32_CAM_OK && status.ap_ip_valid);
    tick = 15000U; ESP32Cam_Task(&esp);
    assert(ESP32Cam_GetStatus(&esp,&status) == ESP32_CAM_OK && !status.ap_ip_valid);
    esp_feed(&esp,"$A0");
    tick += 250U; ESP32Cam_Task(&esp);
    esp_feed(&esp,"90#");
    assert(ESP32Cam_GetRemoteFrame(&esp,remote,sizeof(remote),&length) == ESP32_CAM_OK && length == 0U);
    for (int i=0;i<80;i++) { esp_feed(&esp,"x"); }
    esp_feed(&esp,"ap_ip:192.168.4.1\n");
    assert(ESP32Cam_GetStatus(&esp,&status) == ESP32_CAM_OK && !status.ap_ip_valid);
    esp_feed(&esp,"ap_ip:192.168.4.2");
    esp.rx_byte = 0U; ESP32Cam_OnUartRxComplete(&esp,&huart3);
    esp_feed(&esp,"hidden\n");
    assert(ESP32Cam_GetStatus(&esp,&status) == ESP32_CAM_OK && !status.ap_ip_valid);
    assert(ServoGimbal_DecodeRemoteFrame((const uint8_t*)"A090",4,&axis,&angle) == SERVO_OK);
    assert(axis == 1U && angle == 90U);
    assert(ServoGimbal_DecodeRemoteFrame((const uint8_t*)"B110",4,&axis,&angle) == SERVO_OK);
    assert(axis == 0U && angle == 110U);
    assert(ServoGimbal_DecodeRemoteFrame((const uint8_t*)"A090x",5,&axis,&angle) == SERVO_ERROR_PARAM);
    assert(ServoGimbal_DecodeRemoteFrame((const uint8_t*)"A999",4,&axis,&angle) == SERVO_ERROR_PARAM);
    puts("PASS serial: FIFO, overload STOP priority, error recovery, stale links and frame boundaries");
}

static void test_voice_rx_recovery(void)
{
    Voice_HandleTypeDef voice;
    Voice_FrameTypeDef frame;
    const uint8_t urgent_ids[] = {VOICE_CMD_STOP_1, VOICE_CMD_STOP_2, VOICE_CMD_ALARM};
    uint32_t calls, aborts;
    tick = 0U;
    rx_result = HAL_OK; rx_abort_result = HAL_OK;
    Voice_Init(&voice,&huart2,NULL);
    assert(Voice_UART_StartReceiveIT(&voice) == VOICE_OK);
    voice_irq_feed(&voice,VOICE_CMD_FORWARD);
    Voice_UART_OnError(&voice,&huart3);
    assert(!voice.restart_needed);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_OK);
    voice_irq_feed(&voice,VOICE_CMD_FORWARD);
    (void)Voice_UART_ParseByte(&voice,0xAA,NULL);
    (void)Voice_UART_ParseByte(&voice,0x55,NULL);
    test_primask = 1U;
    Voice_UART_OnError(&voice,&huart2);
    assert(test_primask == 1U);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_ERROR_NO_FRAME);
    assert(voice.rx_index == 0U && voice.restart_needed);
    test_primask = 0U;

    calls = rx_calls; aborts = huart2.abort_count;
    rx_abort_result = HAL_TIMEOUT;
    voice_irq_feed(&voice,VOICE_CMD_FORWARD);
    Voice_UART_Task(&voice);
    assert(voice.restart_needed && voice.rx_index == 0U && rx_calls == calls);
    assert(huart2.abort_count == aborts + 1U);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_ERROR_NO_FRAME);
    assert(Voice_UART_PlayBroadcast(&voice,VOICE_LEGACY_INIT) == VOICE_OK);
    tick += VOICE_TX_TIMEOUT_MS;
    Voice_UART_Task(&voice);
    assert(voice.restart_needed && !voice.tx_busy && voice.tx_timeouts == 1U);

    rx_abort_result = HAL_OK; rx_result = HAL_BUSY;
    Voice_UART_Task(&voice);
    assert(voice.restart_needed && rx_calls == calls + 1U && huart2.rx == NULL);
    rx_result = HAL_ERROR; Voice_UART_Task(&voice);
    assert(voice.restart_needed && rx_calls == calls + 2U);
    rx_result = HAL_OK; Voice_UART_Task(&voice);
    assert(!voice.restart_needed && rx_primask == 1U && test_primask == 0U);
    voice_irq_feed(&voice,VOICE_CMD_SLEEP);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_OK && frame.id == VOICE_CMD_SLEEP);

    for (size_t i=0;i<sizeof(urgent_ids);i++) {
        voice_irq_feed(&voice,VOICE_CMD_FORWARD);
        voice_irq_feed(&voice,urgent_ids[i]);
        Voice_UART_OnError(&voice,&huart2);
        assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_OK && frame.id == urgent_ids[i]);
        assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_ERROR_NO_FRAME);
        Voice_UART_Task(&voice);
    }
    voice_irq_feed(&voice,VOICE_CMD_FORWARD);
    rx_result = HAL_BUSY;
    voice.rx_byte = 0xAA;
    (void)Voice_UART_RxCpltCallback(&voice,&huart2);
    assert(voice.restart_needed && voice.rx_index == 0U);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_ERROR_NO_FRAME);
    rx_result = HAL_OK; Voice_UART_Task(&voice);
    {
        const uint8_t prefix[] = {0xAA,0x55,0x00,VOICE_CMD_FORWARD};
        for (size_t i=0;i<sizeof(prefix);i++) {
            voice.rx_byte = prefix[i];
            (void)Voice_UART_RxCpltCallback(&voice,&huart2);
        }
    }
    rx_result = HAL_ERROR; voice.rx_byte = 0xFB;
    assert(Voice_UART_RxCpltCallback(&voice,&huart2) == VOICE_OK);
    assert(voice.restart_needed);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_ERROR_NO_FRAME);
    rx_result = HAL_OK; Voice_UART_Task(&voice);
    {
        const uint8_t prefix[] = {0xAA,0x55,0x00,VOICE_CMD_STOP_1};
        for (size_t i=0;i<sizeof(prefix);i++) {
            voice.rx_byte = prefix[i];
            (void)Voice_UART_RxCpltCallback(&voice,&huart2);
        }
    }
    rx_result = HAL_BUSY; voice.rx_byte = 0xFB;
    assert(Voice_UART_RxCpltCallback(&voice,&huart2) == VOICE_OK);
    assert(voice.restart_needed);
    assert(Voice_UART_GetLatestFrame(&voice,&frame) == VOICE_OK && frame.id == VOICE_CMD_STOP_1);
    test_primask = 1U; rx_result = HAL_OK; Voice_UART_Task(&voice);
    assert(!voice.restart_needed && test_primask == 1U && rx_primask == 1U);
    test_primask = 0U;
    Voice_UART_Task(NULL);
    puts("PASS voice RX: fault flush, urgent preservation, abort/rearm failures, late callbacks and IRQ ownership");
}

static void test_esp_rx_recovery(void)
{
    ESP32Cam_HandleTypeDef esp;
    uint8_t frame[16];
    uint16_t length;
    uint32_t calls, aborts;
    const char *stops[] = {"$STOP#", "$7,0,0,0#"};
    tick = 0U;
    assert(ESP32Cam_Init(&esp,&huart3) == ESP32_CAM_OK);
    assert(ESP32Cam_StartReceiveIT(&esp) == ESP32_CAM_OK);
    esp_feed(&esp,"$A090#");
    ESP32Cam_OnUartError(&esp,&huart2);
    assert(!esp.restart_needed);
    assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK && length == 4U);
    esp_feed(&esp,"$A090#ap_ip:192.168.4.");
    esp_feed(&esp,"$B1");
    test_primask = 1U;
    ESP32Cam_OnUartError(&esp,&huart3);
    assert(test_primask == 1U);
    assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK && length == 0U);
    assert(!esp.remote_receiving && !esp.remote_frame_length && !esp.line_length);
    test_primask = 0U;
    calls = rx_calls; aborts = huart3.abort_count;
    rx_abort_result = HAL_ERROR;
    esp_feed(&esp,"$B110#ap_ip:192.168.4.1\n");
    ESP32Cam_Task(&esp);
    assert(esp.restart_needed && rx_calls == calls && huart3.abort_count == aborts + 1U);
    assert(!esp.status.ap_ip_valid);
    assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK && length == 0U);
    assert(ESP32Cam_RequestApIp(&esp) == ESP32_CAM_OK);
    tick += ESP32_CAM_TX_TIMEOUT_MS; ESP32Cam_Task(&esp);
    assert(esp.restart_needed && !esp.tx_busy && esp.tx_timeouts == 1U);
    rx_abort_result = HAL_OK; rx_result = HAL_BUSY; ESP32Cam_Task(&esp);
    assert(esp.restart_needed && rx_calls == calls + 1U && huart3.rx == NULL);
    rx_result = HAL_ERROR; ESP32Cam_Task(&esp);
    assert(esp.restart_needed && rx_calls == calls + 2U);
    rx_result = HAL_OK; ESP32Cam_Task(&esp);
    assert(!esp.restart_needed && rx_primask == 1U && test_primask == 0U);
    esp_feed(&esp,"$B110#");
    assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK && length == 4U);
    assert(!memcmp(frame,"B110",4U));
    for (size_t i=0;i<sizeof(stops)/sizeof(stops[0]);i++) {
        esp_feed(&esp,"$A090#"); esp_feed(&esp,stops[i]);
        ESP32Cam_OnUartError(&esp,&huart3);
        assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK && length == 4U);
        assert(!memcmp(frame,"STOP",4U));
        assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK && length == 0U);
        ESP32Cam_Task(&esp);
    }
    esp_feed(&esp,"$A090#");
    rx_result = HAL_BUSY; esp_feed(&esp,"$");
    assert(esp.restart_needed && !esp.remote_receiving);
    assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK && length == 0U);
    rx_result = HAL_OK; ESP32Cam_Task(&esp);
    esp_feed(&esp,"$A090");
    rx_result = HAL_BUSY; esp_feed(&esp,"#");
    assert(esp.restart_needed);
    assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK && length == 0U);
    rx_result = HAL_OK; ESP32Cam_Task(&esp);
    esp_feed(&esp,"$STOP");
    rx_result = HAL_ERROR; esp_feed(&esp,"#");
    assert(esp.restart_needed);
    assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK && length == 4U);
    assert(!memcmp(frame,"STOP",4U));
    test_primask = 1U; rx_result = HAL_OK; ESP32Cam_Task(&esp);
    assert(!esp.restart_needed && test_primask == 1U && rx_primask == 1U);
    test_primask = 0U;
    ESP32Cam_Task(NULL);
    puts("PASS ESP RX: fault flush, STOP preservation, abort/rearm failures, late callbacks and IRQ ownership");
}

static void app_remote(const char *text)
{
    for (;*text;text++) { assert(huart3.rx); *huart3.rx = (uint8_t)*text; BabyCare_OnUartRxComplete(&huart3); }
}

static void app_voice(uint8_t id)
{
    const uint8_t bytes[] = {0xAA,0x55,0x00,id,0xFB};
    for (size_t i=0;i<sizeof(bytes);i++) {
        assert(huart2.rx);
        *huart2.rx = bytes[i];
        BabyCare_OnUartRxComplete(&huart2);
    }
}

static void esp_begin_config(ESP32Cam_HandleTypeDef *esp)
{
    tick = 0U;
    assert(ESP32Cam_Init(esp,&huart3) == ESP32_CAM_OK);
    assert(ESP32Cam_ConfigureDefault(esp) == ESP32_CAM_OK);
    tick = 2000U;
    ESP32Cam_Task(esp);
}

static void test_esp_configuration(void)
{
    ESP32Cam_HandleTypeDef esp;
    uint32_t calls;
    uint8_t frame[16];
    uint16_t length;
    tx_result = HAL_OK;
    tx_abort_result = HAL_OK;
    esp_begin_config(&esp);
    for (uint8_t step = 0U; step < 6U; step++) {
        assert(esp.configure_step == step && esp.tx_busy && esp.configure_tx_pending);
        assert(ESP32Cam_ConfigureDefault(&esp) == ESP32_CAM_ERROR_BUSY);
        assert(ESP32Cam_RequestApIp(&esp) == ESP32_CAM_ERROR_BUSY);
        assert(ESP32Cam_RequestStaIp(&esp) == ESP32_CAM_ERROR_BUSY);
        if (step == 0U) {
            esp_feed(&esp,"OK\r\n"); /* Early ACK must survive until TX completion. */
            ESP32Cam_Task(&esp);
            assert(esp.configure_step == 0U && esp.tx_busy);
        }
        ESP32Cam_OnUartTxComplete(&esp,&huart3);
        ESP32Cam_Task(&esp);
        if (step != 0U) {
            assert(esp.configure_waiting_ack && esp.configure_step == step);
            esp_feed(&esp,"O"); ESP32Cam_Task(&esp);
            esp_feed(&esp,"K"); ESP32Cam_Task(&esp);
            assert(esp.configure_step == step);
            esp_feed(&esp,"\r\n"); ESP32Cam_Task(&esp);
        }
        assert(esp.configure_step == step + 1U && !esp.configure_waiting_ack);
        assert(esp.configure_result == ESP32_CAM_CONFIG_RUNNING);
        tick += esp.configure_wait_ms;
        ESP32Cam_Task(&esp);
    }
    assert(!esp.configuring && !esp.configure_failed);
    assert(esp.configure_result == ESP32_CAM_CONFIG_ACKNOWLEDGED);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);

    esp_begin_config(&esp);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    calls = tx_calls;
    esp_feed(&esp,"BOOK\nOKAY\nnotOK\n$OK#");
    tick += ESP32_CAM_CONFIG_ACK_TIMEOUT_MS - 1U; ESP32Cam_Task(&esp);
    assert(esp.configure_waiting_ack && esp.configure_step == 0U);
    esp_feed(&esp,"$7,0,0,0#");
    assert(ESP32Cam_GetRemoteFrame(&esp,frame,sizeof(frame),&length) == ESP32_CAM_OK);
    assert(length == 4U && !memcmp(frame,"STOP",4U));
    tick++; ESP32Cam_Task(&esp);
    assert(esp.configure_failed && esp.configure_ack_timeouts == 1U);
    assert(esp.configure_result == ESP32_CAM_CONFIG_ACK_TIMEOUT && !esp.configuring);
    esp_feed(&esp,"OK\n"); tick += 10000U; ESP32Cam_Task(&esp);
    assert(esp.configure_step == 0U && tx_calls == calls);

    esp_begin_config(&esp);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    tick += ESP32_CAM_CONFIG_ACK_TIMEOUT_MS;
    esp_feed(&esp,"OK\n"); ESP32Cam_Task(&esp);
    assert(esp.configure_result == ESP32_CAM_CONFIG_ACK_TIMEOUT);
    esp_begin_config(&esp);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    tick += ESP32_CAM_CONFIG_ACK_TIMEOUT_MS - 1U;
    esp_feed(&esp,"OK\n"); tick += 10U; ESP32Cam_Task(&esp);
    assert(esp.configure_step == 1U && !esp.configure_failed);

    esp_begin_config(&esp);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    esp_feed(&esp,"fail,ssid is null!\r\n"); ESP32Cam_Task(&esp);
    assert(esp.configure_rejections == 1U && esp.configure_result == ESP32_CAM_CONFIG_REJECTED);
    assert(esp.configure_failed && !esp.configuring);
    esp_begin_config(&esp);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    ESP32Cam_OnUartError(&esp,&huart3);
    esp_feed(&esp,"OK\n"); ESP32Cam_Task(&esp);
    assert(esp.configure_result == ESP32_CAM_CONFIG_RX_FAILED && esp.configure_step == 0U);

    esp_begin_config(&esp);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    esp_feed(&esp,"O");
    rx_result = HAL_BUSY; esp_feed(&esp,"K");
    rx_result = HAL_OK; ESP32Cam_Task(&esp);
    assert(esp.configure_result == ESP32_CAM_CONFIG_RX_FAILED && esp.configure_step == 0U);

    esp_begin_config(&esp);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    esp_feed(&esp,"OK");
    esp.rx_byte = 0U; ESP32Cam_OnUartRxComplete(&esp,&huart3);
    esp_feed(&esp,"hidden\n"); ESP32Cam_Task(&esp);
    assert(esp.configure_waiting_ack && esp.configure_step == 0U);
    for (int i = 0; i < 80; i++) { esp_feed(&esp,"x"); }
    esp_feed(&esp,"OK\n"); ESP32Cam_Task(&esp);
    assert(esp.configure_step == 0U);
    esp_feed(&esp,"OK\n"); ESP32Cam_Task(&esp);
    assert(esp.configure_step == 1U);

    esp_begin_config(&esp);
    calls = tx_calls;
    tick += ESP32_CAM_TX_TIMEOUT_MS; ESP32Cam_Task(&esp);
    assert(esp.configure_result == ESP32_CAM_CONFIG_TX_FAILED && !esp.tx_busy);
    tick += 10000U; ESP32Cam_Task(&esp);
    assert(tx_calls == calls && esp.configure_step == 0U);
    esp_begin_config(&esp);
    calls = tx_calls;
    tx_abort_result = HAL_ERROR;
    tick += ESP32_CAM_TX_TIMEOUT_MS; ESP32Cam_Task(&esp);
    assert(esp.tx_busy && esp.configure_tx_pending && esp.configure_step == 0U);
    assert(ESP32Cam_ConfigureDefault(&esp) == ESP32_CAM_ERROR_BUSY);
    esp_feed(&esp,"OK\n"); ESP32Cam_Task(&esp);
    assert(esp.configure_step == 0U && tx_calls == calls);
    tx_abort_result = HAL_OK; ESP32Cam_Task(&esp);
    assert(!esp.tx_busy && esp.configure_result == ESP32_CAM_CONFIG_TX_FAILED);
    tx_result = HAL_ERROR;
    esp_begin_config(&esp);
    assert(esp.configure_retries == 1U && !esp.tx_busy);
    tick += 1000U; ESP32Cam_Task(&esp);
    tick += 1000U; ESP32Cam_Task(&esp);
    assert(esp.configure_retries == 3U && esp.configure_result == ESP32_CAM_CONFIG_TX_FAILED);
    tx_result = HAL_OK;

    tick = UINT32_MAX - 2000U;
    assert(ESP32Cam_Init(&esp,&huart3) == ESP32_CAM_OK);
    assert(ESP32Cam_ConfigureDefault(&esp) == ESP32_CAM_OK);
    tick += 2000U; ESP32Cam_Task(&esp);
    ESP32Cam_OnUartTxComplete(&esp,&huart3); ESP32Cam_Task(&esp);
    tick += ESP32_CAM_CONFIG_ACK_TIMEOUT_MS; ESP32Cam_Task(&esp);
    assert(esp.configure_result == ESP32_CAM_CONFIG_ACK_TIMEOUT && esp.configure_ack_timeouts == 1U);
    puts("PASS ESP configuration: TX/ACK separation, full sequence, strict replies, STOP, timeout, errors and wrap");
}
static void tx_complete(void)
{
    tx_data = NULL;
    tx_length = 0U;
    DebugTx_OnComplete(&huart1);
    DebugTx_Task();
}

static void test_debug_tx(void)
{
    DebugTx_Stats stats;
    char payload[] = "original\r\n";
    char large[DEBUG_TX_FRAME_SIZE + 1U];
    memset(large,0,sizeof(large));
    tick = 0U;
    tx_calls = 0U;
    assert(DebugTx_Init(&huart1));
    assert(DebugTx_Write(payload, sizeof(payload)-1U, false));
    memset(payload,'x',sizeof(payload));
    assert(tx_calls == 0U);
    DebugTx_Task();
    assert(tx_length == 10U && !memcmp(tx_data,"original\r\n",10U));
    for (int i=0;i<3;i++) { assert(DebugTx_Write("n\r\n",3U,false)); }
    assert(!DebugTx_Write("full",4U,false));
    assert(DebugTx_Write("alarm1\r\n",8U,true));
    assert(DebugTx_Write("alarm2\r\n",8U,true));
    assert(!DebugTx_Write("full",4U,true));
    DebugTx_Task();
    assert(!memcmp(tx_data,"original\r\n",10U));
    tx_complete();
    assert(tx_length == 8U && !memcmp(tx_data,"alarm1\r\n",8U));
    tx_complete();
    assert(!memcmp(tx_data,"alarm2\r\n",8U));
    for (int i=0;i<4;i++) { tx_complete(); }
    DebugTx_GetStats(&stats);
    assert(stats.queued == 0U && stats.completed == 6U && stats.dropped == 2U);
    assert(!DebugTx_Write(large,sizeof(large),false));
    assert(DebugTx_Write(large,DEBUG_TX_FRAME_SIZE,false));
    DebugTx_Task();
    DebugTx_OnComplete(&huart2); DebugTx_Task();
    assert(tx_length == DEBUG_TX_FRAME_SIZE);
    tx_complete();
    assert(DebugTx_Init(&huart1));
    assert(DebugTx_Write("partial",7U,false));
    tick = UINT32_MAX - 50U; DebugTx_Task();
    tick += DEBUG_TX_TIMEOUT_MS;
    tx_abort_result = HAL_ERROR;
    DebugTx_Task();
    assert(tx_length == 7U && !memcmp(tx_data,"partial",7U));
    assert(!DebugTx_Init(&huart1));
    assert(tx_length == 7U && !memcmp(tx_data,"partial",7U));
    assert(DebugTx_Write("next\r\n",6U,false));
    tx_abort_result = HAL_OK;
    tick += DEBUG_TX_RETRY_MS; DebugTx_Task();
    assert(tx_length == 2U && !memcmp(tx_data,"\r\n",2U));
    tx_complete();
    assert(tx_length == 6U && !memcmp(tx_data,"next\r\n",6U));
    tx_complete();
    DebugTx_GetStats(&stats);
    assert(stats.timeouts == 1U && stats.abort_errors == 1U && stats.dropped == 1U);
    assert(DebugTx_Init(&huart1));
    tx_result = HAL_BUSY;
    assert(DebugTx_Write("retry\r\n",7U,false));
    for (int i=0;i<3;i++) { tick += DEBUG_TX_RETRY_MS; DebugTx_Task(); }
    DebugTx_GetStats(&stats);
    assert(stats.start_errors == 3U && stats.dropped == 1U && stats.queued == 0U);
    tx_result = HAL_OK;
    assert(DebugTx_Init(&huart1));
    assert(DebugTx_Write("error\r\n",7U,false));
    DebugTx_Task(); DebugTx_OnError(&huart1); DebugTx_Task();
    assert(tx_length == 2U);
    tx_complete();
    assert(DebugTx_Init(&huart1));
    puts("PASS debug TX: copied buffers, bounded queues, alert priority, retry, timeout, wrap and resync");
}

static void test_command_discard(void)
{
    Voice_HandleTypeDef voice;
    Voice_FrameTypeDef frame;
    ESP32Cam_HandleTypeDef esp;
    uint8_t remote[ESP32_CAM_REMOTE_FRAME_SIZE];
    uint16_t length;
    Voice_Init(&voice, &huart2, NULL);
    voice.queue_head = 3U; voice.queue_tail = 0U; voice.rx_index = 4U;
    voice.urgent_pending = 1U;
    voice.urgent_frame.type = VOICE_UART_CMD_TYPE;
    voice.urgent_frame.id = VOICE_CMD_ALARM;
    test_primask = 1U;
    Voice_UART_DiscardQueuedCommands(&voice);
    assert(test_primask == 1U && voice.rx_index == 0U);
    assert(Voice_UART_GetLatestFrame(&voice, &frame) == VOICE_OK && frame.id == VOICE_CMD_ALARM);
    assert(Voice_UART_GetLatestFrame(&voice, &frame) == VOICE_ERROR_NO_FRAME);
    assert(ESP32Cam_Init(&esp, &huart3) == ESP32_CAM_OK);
    esp.queue_head = 3U; esp.queue_tail = 0U; esp.stop_pending = 1U;
    esp.remote_receiving = 1U; esp.remote_frame_length = 6U;
    esp.configure_ack = 1U; esp.tx_busy = 1U;
    ESP32Cam_DiscardQueuedCommands(&esp);
    assert(test_primask == 1U && !esp.remote_receiving && esp.remote_frame_length == 0U);
    assert(esp.configure_ack == 1U && esp.tx_busy == 1U);
    assert(ESP32Cam_GetRemoteFrame(&esp, remote, sizeof(remote), &length) == ESP32_CAM_OK);
    assert(length == 4U && memcmp(remote, "STOP", 4U) == 0);
    assert(ESP32Cam_GetRemoteFrame(&esp, remote, sizeof(remote), &length) == ESP32_CAM_OK && length == 0U);
    Voice_UART_DiscardQueuedCommands(NULL); ESP32Cam_DiscardQueuedCommands(NULL);
    test_primask = 0U;
    puts("PASS command discard: normal queues cleared, urgent commands and active TX ownership retained");
}

static void test_interlock(void)
{
    const Interlock_Status *state = Interlock_GetStatus();
    simulate_arm_latch = 1U;
    BABY_CARE_INTERLOCK_PORT->ODR = 0U;
    tick = 0U; Interlock_Init(); Interlock_Poll();
    assert(state->tripped && !state->ready && !Interlock_Enable());
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_STOP_PIN;
    Interlock_Poll(); tick = 49U;
    assert(!Interlock_Reset() && !state->ready);
    tick = 50U; assert(!Interlock_Reset() && state->ready && !state->enabled);
    for (int i = 0; i < 3; i++) { assert(!Interlock_Reset() && !Interlock_Enable()); }
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    Interlock_Poll(); tick = 99U; assert(!Interlock_Reset());
    tick = 100U; assert(Interlock_Reset() && !state->enabled);
    assert(Interlock_Enable() && state->enabled);
    BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_STOP_PIN;
    Interlock_Poll();
    assert(state->tripped && state->reason == INTERLOCK_TRIP_STOP);
    Interlock_Poll();
    assert(!state->enabled && !state->ready && !state->armed);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_STOP_PIN;
    tick = 200U; Interlock_Poll(); tick = 250U;
    assert(!Interlock_Enable() && !state->ready);
    assert(!Interlock_Reset() && state->ready);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    Interlock_Poll(); tick = 300U; assert(Interlock_Reset());
    assert(Interlock_Enable());
    BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_ARMED_PIN;
    Interlock_Poll(); assert(state->reason == INTERLOCK_TRIP_FEEDBACK && !state->enabled);

    simulate_arm_latch = 0U;
    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_ARMED_PIN;
    Interlock_Init(); Interlock_Poll();
    assert(state->tripped && state->reason == INTERLOCK_TRIP_FEEDBACK);
    assert(!Interlock_Reset());
    simulate_arm_latch = 1U;
    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN;
    tick = UINT32_MAX - 25U; Interlock_Init(); Interlock_Poll();
    tick = 24U; Interlock_Poll(); assert(state->ready);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    Interlock_Poll(); tick = 74U; assert(Interlock_Enable());
    Interlock_Disable(); assert(!state->enabled && state->ready);
    BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_ARMED_PIN;
    Interlock_Poll(); assert(state->tripped && state->reason == INTERLOCK_TRIP_FEEDBACK);
    Interlock_Trip(); assert(state->tripped && !state->ready);
    puts("PASS interlock: open wires, immediate trip, manual ARM, stable recovery, stuck feedback and tick wrap");
}

static void prepare_interlock_power(void)
{
    simulate_arm_latch = 1U;
    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN;
    tick = 0U; Interlock_Init(); Interlock_Poll();
    tick = 50U; Interlock_Poll();
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    Interlock_Poll(); tick = 100U; Interlock_Poll();
    assert(Interlock_GetStatus()->qualified);
}

static void test_interlock_power(void)
{
    const Interlock_Status *state = Interlock_GetStatus();
    prepare_interlock_power();
    assert(Interlock_Enable() && state->power_pending && !state->power_good);
    tick = 299U;
    for (int i = 0; i < 10; i++) { assert(Interlock_Enable()); }
    tick = 300U; assert(!Interlock_Enable());
    assert(state->tripped && state->reason == INTERLOCK_TRIP_POWER);
    assert(!state->enabled && !state->ready && !state->power_pending);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_POWER_PIN;
    tick = 500U; Interlock_Poll();
    assert(!Interlock_Enable() && state->trip_count == 1U);
    assert(!Interlock_Reset() && state->ready);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    Interlock_Poll(); tick += BABY_CARE_INTERLOCK_STABLE_MS;
    assert(Interlock_Reset() && !state->enabled);
    assert(Interlock_Enable() && state->power_good && !state->power_pending);
    BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_POWER_PIN;
    Interlock_Poll(); assert(state->tripped && state->trip_count == 2U);

    prepare_interlock_power();
    assert(Interlock_Enable());
    tick = 299U; BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_POWER_PIN;
    Interlock_Poll(); assert(!state->power_pending && !state->tripped);
    tick = 1000U; Interlock_Poll(); assert(state->enabled);
    Interlock_Disable();
    BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_POWER_PIN;
    tick = 2000U; Interlock_Poll(); assert(!state->tripped);
    assert(Interlock_Enable() && state->power_pending);
    tick = 2200U; Interlock_Poll(); assert(state->reason == INTERLOCK_TRIP_POWER);

    prepare_interlock_power();
    tick = UINT32_MAX - 100U; assert(Interlock_Enable());
    tick = 98U; Interlock_Poll(); assert(state->enabled);
    tick = 99U; Interlock_Poll(); assert(state->reason == INTERLOCK_TRIP_POWER);
    prepare_interlock_power();
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_POWER_PIN;
    assert(Interlock_Enable());
    BABY_CARE_INTERLOCK_PORT->ODR &= ~(BABY_CARE_INTERLOCK_POWER_PIN | BABY_CARE_INTERLOCK_ARMED_PIN);
    Interlock_Poll(); assert(state->reason == INTERLOCK_TRIP_POWER && !state->enabled);
    prepare_interlock_power();
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_POWER_PIN;
    assert(Interlock_Enable());
    BABY_CARE_INTERLOCK_PORT->ODR &= ~(BABY_CARE_INTERLOCK_POWER_PIN | BABY_CARE_INTERLOCK_ARMED_PIN |
                    BABY_CARE_INTERLOCK_STOP_PIN);
    Interlock_Poll(); assert(state->reason == INTERLOCK_TRIP_STOP);
    puts("PASS power interlock: startup deadline, repeat requests, dropout, manual recovery and tick wrap");
}

static void test_heartbeat(void)
{
    uint32_t writes;
    tick = 0U;
    SystemHeartbeat_Init();
    assert(!(BABY_CARE_HEARTBEAT_PORT->ODR & BABY_CARE_HEARTBEAT_PIN));
    writes = heartbeat_writes;
    tick = 49U; SystemHeartbeat_Service();
    assert(heartbeat_writes == writes);
    tick = 50U; SystemHeartbeat_Service();
    assert((BABY_CARE_HEARTBEAT_PORT->ODR & BABY_CARE_HEARTBEAT_PIN) && heartbeat_writes == writes + 1U);
    SystemHeartbeat_Service();
    assert(heartbeat_writes == writes + 1U);
    tick = 100U; SystemHeartbeat_Service();
    assert(!(BABY_CARE_HEARTBEAT_PORT->ODR & BABY_CARE_HEARTBEAT_PIN));
    tick = 10000U; SystemHeartbeat_Service();
    assert((BABY_CARE_HEARTBEAT_PORT->ODR & BABY_CARE_HEARTBEAT_PIN) && heartbeat_writes == writes + 3U);
    SystemHeartbeat_Service();
    assert(heartbeat_writes == writes + 3U);
    tick = UINT32_MAX - 24U; SystemHeartbeat_Init();
    writes = heartbeat_writes;
    tick = 24U; SystemHeartbeat_Service();
    assert(heartbeat_writes == writes);
    tick = 25U; SystemHeartbeat_Service();
    assert((BABY_CARE_HEARTBEAT_PORT->ODR & BABY_CARE_HEARTBEAT_PIN) && heartbeat_writes == writes + 1U);
    tick = 75U; SystemHeartbeat_Service();
    assert(!(BABY_CARE_HEARTBEAT_PORT->ODR & BABY_CARE_HEARTBEAT_PIN));
    puts("PASS heartbeat: foreground toggle, no catch-up burst, reset LOW and tick wrap");
}

static void test_app_sensor_startup(void)
{
    tick = 0U;
    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
    BabyCare_Init(); BabyCare_Task();
    tick = 999U; BabyCare_Task();
    tick = BABY_CARE_SENSOR_PERIOD_MS - 1U; BabyCare_Task();
    assert(fake_dht_requests == 0U && BabyCare_GetStatus()->sensor_sample_count == 0U);
    tick++; BabyCare_Task();
    assert(fake_dht_requests == 1U && fake_dht_pending);
    assert(BabyCare_GetStatus()->sensor_sample_count == 0U);
    tick += 10U; app_remote("$STOP#"); BabyCare_Task();
    assert(htim2.enabled == 0U && htim3.enabled == 0U && fake_dht_pending);
    tick += 10U; BabyCare_Task();
    assert(BabyCare_GetStatus()->sensor_sample_count == 1U && !fake_dht_pending);
    assert(!BabyCare_GetStatus()->sensor_valid);
    fake_dht_start_busy = 1U;
    tick = BABY_CARE_SENSOR_PERIOD_MS * 2U; BabyCare_Task();
    assert(fake_dht_requests == 2U && BabyCare_GetStatus()->sensor_sample_count == 1U);
    assert(!fake_dht_pending);
    fake_dht_start_busy = 0U;
    tick += BABY_CARE_SENSOR_PERIOD_MS; BabyCare_Task();
    assert(fake_dht_requests == 3U && fake_dht_pending);
    tick += 20U; BabyCare_Task();
    assert(BabyCare_GetStatus()->sensor_sample_count == 2U && !fake_dht_pending);
    puts("PASS sensor scheduling: boot settling, 2100 ms cadence, BUSY and STOP during start pulse");
}

static void test_app_serial_faults(void)
{
    uint32_t writes;
    tick = 0U;
    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
    BabyCare_Init(); BabyCare_Task();
    tick = BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    BabyCare_Task(); tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
    app_voice(VOICE_CMD_FORWARD); app_remote("$A110#");
    BabyCare_OnUartError(&huart2); BabyCare_OnUartError(&huart3);
    rx_abort_result = HAL_ERROR;
    app_voice(VOICE_CMD_FORWARD); app_remote("$B110#");
    writes = heartbeat_writes;
    tick += 50U; BabyCare_Task();
    assert(BabyCare_GetStatus()->mode == BABY_CARE_MODE_IDLE);
    assert(htim2.enabled == 0U && htim3.enabled == 0U && heartbeat_writes > writes);
    rx_abort_result = HAL_OK; BabyCare_Task();
    app_voice(VOICE_CMD_FORWARD); BabyCare_Task();
    assert(BabyCare_GetStatus()->motion == BABY_CARE_MOTION_MOTOR);
    assert(htim2.enabled != 0U && htim3.enabled == 0U);
    app_voice(VOICE_CMD_STOP_1); BabyCare_OnUartError(&huart2);
    BabyCare_Task();
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    app_remote("$A110#"); BabyCare_Task();
    assert(htim3.enabled != 0U);
    app_remote("$STOP#"); BabyCare_OnUartError(&huart3);
    BabyCare_Task();
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    puts("PASS application serial faults: no stale motion, live heartbeat, fresh recovery and pending STOP");
}

static void test_app(void)
{
    uint32_t soothe_restart;
    tick = 0U;
    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
    BabyCare_Init();
    assert(tick == 0U && htim2.enabled == 0U && htim3.enabled == 0U);
    BabyCare_Task();
    assert(BabyCare_GetStatus()->sensor_sample_count == 0U);
    assert(tick == 0U);
    tick = 10U;
    app_remote("$STOP#"); BabyCare_Task();
    assert(BabyCare_GetStatus()->sensor_sample_count == 0U && htim2.enabled == 0U);
    tick = 20U; BabyCare_Task();
    assert(BabyCare_GetStatus()->sensor_sample_count == 0U);
    assert(!BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    tick = 50U; BabyCare_Task();
    assert(Interlock_GetStatus()->ready && htim3.enabled == 0U);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    BabyCare_Task(); tick = 100U; BabyCare_Task();
    app_remote("$A080#$B110#");
    BabyCare_Task();
    BabyCare_Task();
    assert(htim2.enabled == 0U && htim3.enabled != 0U);
    for (int i = 0; i < 20; i++) { tick += BABY_CARE_SERVO_STEP_MS; BabyCare_Task(); }
    assert(htim3.compare[TIM_CHANNEL_1] == 1611U);
    assert(htim3.compare[TIM_CHANNEL_2] == 1555U);
    app_remote("$7,0,0,0#"); BabyCare_Task();
    assert(htim3.enabled == 0U);
    BabyCare_Task();
    assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    assert(htim2.enabled != 0U && htim3.enabled != 0U);
    BabyCare_Task();
    assert(tx_data != NULL && tx_length != 0U);
    app_remote("$STOP#"); BabyCare_Task();
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    assert(tx_data != NULL && tx_length != 0U);
    BabyCare_Task();
    assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    soothe_restart = tick;
    assert(BabyCare_RequestMode(BABY_CARE_MODE_ALERT));
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    app_remote("$A120#"); BabyCare_Task();
    assert(htim3.enabled == 0U && BabyCare_GetStatus()->safety_latched);
    assert(!BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    BabyCare_Stop();
    assert(BabyCare_GetStatus()->safety_latched);
    assert(!BabyCare_ClearSafetyLatch());
    assert(!Interlock_GetStatus()->enabled && Interlock_GetStatus()->ready);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    BabyCare_Task(); tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
    assert(BabyCare_ClearSafetyLatch());
    BabyCare_Task();
    assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    soothe_restart = tick;
    app_remote("$A090x#"); BabyCare_Task();
    assert(BabyCare_GetStatus()->mode == BABY_CARE_MODE_SOOTHE);
    tick = soothe_restart + BABY_CARE_SOOTHE_MAX_MS - 1U;
    assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    tick++; BabyCare_Task();
    assert(BabyCare_GetStatus()->mode == BABY_CARE_MODE_IDLE && htim2.enabled == 0U);
    app_remote("$A110#$STOP#$A130#"); BabyCare_Task();
    assert(htim3.enabled == 0U);
    assert(debug_blocking_calls == 0U);
    tick += REPORT_PERIOD_MS; BabyCare_Task();
    assert(report_count > 0U && strstr(report,"\"cameraDetected\":null"));
    assert(strstr(report,"\"sound\":null") && strstr(report,"\"babyInBed\":null"));
    assert(strstr(report,"\"status\":\"sensor_fault\""));
    BABY_CARE_INTERLOCK_PORT->ODR &= ~BABY_CARE_INTERLOCK_STOP_PIN;
    BabyCare_Task();
    assert(BabyCare_GetStatus()->safety_latched && !Interlock_GetStatus()->enabled);
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_STOP_PIN;
    BabyCare_Task(); tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
    app_remote("$RESET#"); BabyCare_Task();
    assert(Interlock_GetStatus()->ready && !Interlock_GetStatus()->enabled);
    app_remote("$A110#"); BabyCare_Task();
    assert(htim3.enabled == 0U);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    BabyCare_Task(); tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
    app_remote("$A110#$B090#");
    assert(BabyCare_ClearSafetyLatch()); BabyCare_Task();
    BabyCare_Task(); assert(htim3.enabled == 0U);
    assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    assert(!BabyCare_GetStatus()->interlock_power_pending);
    BABY_CARE_INTERLOCK_PORT->ODR &= ~(BABY_CARE_INTERLOCK_POWER_PIN | BABY_CARE_INTERLOCK_ARMED_PIN);
    BabyCare_Task();
    assert(BabyCare_GetStatus()->interlock_trip == INTERLOCK_TRIP_POWER);
    assert(!BabyCare_GetStatus()->interlock_power_good);
    assert(BabyCare_GetStatus()->safety_latched && htim2.enabled == 0U && htim3.enabled == 0U);
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_POWER_PIN;
    BabyCare_Task();
    assert(!BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    assert(!BabyCare_ClearSafetyLatch());
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    BabyCare_Task(); tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
    assert(BabyCare_ClearSafetyLatch());
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    BabyCare_Task();
    pwm_result = HAL_ERROR;
    assert(!BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
    assert(BabyCare_GetStatus()->safety_latched && BabyCare_GetStatus()->fault_flags != 0U);
    assert(!Interlock_GetStatus()->enabled && !Interlock_GetStatus()->ready);
    assert(!BabyCare_ClearSafetyLatch());
    pwm_result = HAL_OK;
    puts("PASS application: nonblocking init, alarm latch, malformed commands, expiry, STOP and HAL faults");
}

static void test_desktop_demo_limits(void)
{
    uint32_t start;
    uint32_t bases[] = {0U, UINT32_MAX - 100U};
    pwm_result = tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    simulate_arm_latch = 1U;
    for (size_t b = 0U; b < sizeof(bases) / sizeof(bases[0]); b++) {
        tick = bases[b];
        BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
        BabyCare_Init(); BabyCare_Task();
        assert(htim2.enabled == 0U && htim3.enabled == 0U);
        assert(htim3.compare[TIM_CHANNEL_1] == 1500U && htim3.compare[TIM_CHANNEL_2] == 1500U);
        tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
        BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
        BabyCare_Task(); tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
        app_remote("$A000#$B180#"); BabyCare_Task(); BabyCare_Task();
        start = tick;
        for (unsigned i = 0U; i < 60U; i++) {
            tick += BABY_CARE_SERVO_STEP_MS; BabyCare_Task();
            assert(htim3.compare[TIM_CHANNEL_1] >= 1333U && htim3.compare[TIM_CHANNEL_1] <= 1666U);
            assert(htim3.compare[TIM_CHANNEL_2] >= 1416U && htim3.compare[TIM_CHANNEL_2] <= 1583U);
        }
        assert(htim3.compare[TIM_CHANNEL_1] == 1666U && htim3.compare[TIM_CHANNEL_2] == 1583U);
        tick = start + BABY_CARE_REMOTE_HOLD_MS; BabyCare_Task();
        assert(htim3.enabled == 0U && !Interlock_GetStatus()->enabled);
        BabyCare_Task();
        assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
        start = tick;
        for (unsigned i = 0U; i < 200U; i++) {
            tick += 20U; BabyCare_Task();
            assert(htim2.compare[TIM_CHANNEL_3] <= BABY_CARE_SOOTHE_SPEED_PERCENT);
        }
        tick = start + BABY_CARE_SOOTHE_MAX_MS - 1U;
        assert(BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
        tick++; BabyCare_Task();
        assert(BabyCare_GetStatus()->mode == BABY_CARE_MODE_IDLE);
        assert(htim2.enabled == 0U && htim3.enabled == 0U && !Interlock_GetStatus()->enabled);
    }
    puts("PASS desktop demo: SG90 pulse bounds, 20pct PWM, 10s expiry and tick wrap");
}

static void test_app_motor_init_failure(void)
{
    HAL_StatusTypeDef errors[] = {HAL_BUSY, HAL_ERROR, HAL_TIMEOUT};
    pwm_result = HAL_OK;
    tx_result = rx_result = HAL_OK;
    simulate_arm_latch = 1U;
    check_motor_gpio_init = 1U;
    for (size_t i = 0U; i < sizeof(errors) / sizeof(errors[0]); i++) {
        BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
        GPIOA->ODR = GPIO_PIN_4 | GPIO_PIN_5;
        htim2.enabled = 1U << TIM_CHANNEL_3;
        htim2.compare[TIM_CHANNEL_3] = 60U;
        motor_gpio_init_calls = 0U;
        override_motor_stop = 1U;
        motor_stop_result = errors[i];
        BabyCare_Init();
        assert(motor_gpio_init_calls == 0U);
        assert(BabyCare_GetStatus()->fault_flags == BABY_CARE_FAULT_MOTOR);
        assert(BabyCare_GetStatus()->safety_latched);
        assert(BabyCare_GetStatus()->mode == BABY_CARE_MODE_FAULT);
        assert(!Interlock_GetStatus()->enabled && !Interlock_GetStatus()->ready);
        assert((BABY_CARE_INTERLOCK_PORT->ODR & BABY_CARE_INTERLOCK_ENABLE_PIN) == 0U);
        override_motor_stop = 0U;
        tick += 100U;
        BabyCare_Task();
        assert(!BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE));
        assert(!BabyCare_ClearSafetyLatch());
        assert(BabyCare_GetStatus()->fault_flags == BABY_CARE_FAULT_MOTOR);
        assert(!Interlock_GetStatus()->enabled && !Interlock_GetStatus()->ready);
        assert(motor_gpio_init_calls == 0U);
    }
    check_motor_gpio_init = 0U;
    puts("PASS application motor init failure: motor fault retained and actuator gate never enabled");
}

static void test_esp_boot_policy(void)
{
    const uint32_t starts[] = {80000U, UINT32_MAX - 1000U};
    uint32_t previous_calls, queries;
    tx_result = HAL_OK; tx_abort_result = HAL_OK;
    rx_result = HAL_OK; rx_abort_result = HAL_OK;
    pwm_result = HAL_OK; override_motor_stop = 0U;
    test_primask = 0U;
    for (size_t round = 0U; round < sizeof(starts) / sizeof(starts[0]); round++) {
        tick = starts[round];
        BABY_CARE_INTERLOCK_PORT->ODR = 0U;
        previous_calls = esp_tx_calls;
        queries = 0U;
        BabyCare_Init();
        assert(esp_tx_calls == previous_calls);
        for (uint32_t elapsed = 0U; elapsed <= 15000U; elapsed += 50U) {
            tick = starts[round] + elapsed;
            BabyCare_Task();
            assert(htim2.enabled == 0U && htim3.enabled == 0U);
            assert(!(BABY_CARE_INTERLOCK_PORT->ODR & BABY_CARE_INTERLOCK_ENABLE_PIN));
            if (esp_tx_calls != previous_calls) {
                assert(esp_tx_calls == previous_calls + 1U);
#if BABY_CARE_ESP32_CONFIGURE_ON_BOOT
                assert(elapsed == 2000U && esp_tx_length == 11U);
                assert(!memcmp(esp_tx_data, "wifi_mode:2", 11U));
                app_remote("$STOP#");
                BabyCare_Task();
                assert(htim2.enabled == 0U && htim3.enabled == 0U);
                break;
#else
                assert(elapsed == (queries + 1U) * BABY_CARE_ESP32_QUERY_PERIOD_MS);
                assert(esp_tx_length == 5U + (queries % 2U));
                assert(!memcmp(esp_tx_data, (queries % 2U) ? "sta_ip" : "ap_ip", esp_tx_length));
                if (queries == 0U) { app_remote("ap_ip:192.168.4.1\n"); }
                queries++;
                previous_calls = esp_tx_calls;
#endif
            }
            BabyCare_OnUartTxComplete(&huart1);
            BabyCare_OnUartTxComplete(&huart2);
            BabyCare_OnUartTxComplete(&huart3);
        }
#if BABY_CARE_ESP32_CONFIGURE_ON_BOOT
        assert(esp_tx_calls == previous_calls + 1U);
#else
        assert(queries == 3U && BabyCare_GetStatus()->esp32_ap_ip_valid);
        app_remote("$A090#$STOP#"); BabyCare_Task();
        assert(htim2.enabled == 0U && htim3.enabled == 0U);
#endif
    }
#if BABY_CARE_ESP32_CONFIGURE_ON_BOOT
    puts("PASS ESP boot policy: explicit opt-in configures, STOP and disconnected interlock remain effective");
#else
    puts("PASS ESP boot policy: default preserves settings, timed AP/STA queries, reinit, tick wrap and STOP");
#endif
}

static void test_voice_v3_boot_broadcast(void)
{
    const uint32_t starts[] = {0U, UINT32_MAX - 50U};
    tx_abort_result = rx_result = rx_abort_result = pwm_result = HAL_OK;
    override_motor_stop = check_motor_gpio_init = 0U;
    test_primask = 0U;
    for (size_t i = 0U; i < sizeof(starts) / sizeof(starts[0]); i++) {
        const uint32_t before = voice_tx_calls;
        tick = starts[i];
        tx_result = HAL_OK;
        BABY_CARE_INTERLOCK_PORT->ODR = 0U;
        BabyCare_Init();
        BabyCare_Task();
        assert(voice_tx_calls == before);
        tick = starts[i] + 99U;
        BabyCare_Task();
        assert(voice_tx_calls == before);
        tx_result = HAL_BUSY;
        tick = starts[i] + 100U;
        BabyCare_Task();
        assert(voice_tx_calls == before);
        tx_result = HAL_OK;
        tick++;
        BabyCare_Task();
        assert(voice_tx_calls == before + 1U);
        assert(voice_tx_length == 5U);
        assert(memcmp(voice_tx_data, "\xAA\x55\xFF\x58\xFB", 5U) == 0);
        BabyCare_OnUartTxComplete(&huart2);
        tick += 5000U;
        BabyCare_Task();
        assert(voice_tx_calls == before + 1U);
        assert(htim2.enabled == 0U && htim3.enabled == 0U);
        assert(!(BABY_CARE_INTERLOCK_PORT->ODR & BABY_CARE_INTERLOCK_ENABLE_PIN));
    }
    puts("PASS voice V3 boot: exact 0x58 frame, delayed nonblocking retry, no duplicate, tick wrap and no actuation");
}

static int test_servo_start_clock(void)
{
    const uint32_t starts[] = {200U, UINT32_MAX - 1U, UINT32_MAX};
    unsigned cases = 0U;
    pwm_result = tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    simulate_arm_latch = 1U;
    override_motor_stop = check_motor_gpio_init = 0U;
    test_primask = 0U;
    for (unsigned base = 0U; base < 3U; ++base) {
        for (unsigned axis = TIM_CHANNEL_1; axis <= TIM_CHANNEL_2; ++axis) {
            for (unsigned delayed_axis = TIM_CHANNEL_1; delayed_axis <= TIM_CHANNEL_2; ++delayed_axis) {
                for (uint32_t delay = 0U; delay <= 2U; ++delay) {
                    uint32_t remote_started;
                    const uint32_t first_angle = axis == TIM_CHANNEL_1
                        ? 90U + BABY_CARE_SERVO_STEP_DEG : 90U - BABY_CARE_SERVO_STEP_DEG;
                    const uint32_t first_compare = BABY_CARE_SERVO_MIN_PULSE_US +
                        (BABY_CARE_SERVO_MAX_PULSE_US - BABY_CARE_SERVO_MIN_PULSE_US) * first_angle / 180U;
                    memset(servo_start_advance, 0, sizeof(servo_start_advance));
                    tick = starts[base] - 2U * BABY_CARE_INTERLOCK_STABLE_MS;
                    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
                    BabyCare_Init(); BabyCare_Task();
                    tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
                    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
                    BabyCare_Task(); tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
                    assert(tick == starts[base] && Interlock_GetStatus()->qualified);
                    servo_start_advance[delayed_axis] = delay;
                    app_remote(axis == TIM_CHANNEL_1 ? "$B110#" : "$A110#");
                    BabyCare_Task();
                    remote_started = tick;
                    if (htim3.compare[axis] != 1500U) {
                        printf("FAIL early servo step: axis=%u delayed_axis=%u delay=%lu start=%lu compare=%lu\n",
                               axis, delayed_axis, (unsigned long)delay, (unsigned long)starts[base],
                               (unsigned long)htim3.compare[axis]);
                        return 0;
                    }
                    assert(htim3.enabled != 0U && Interlock_GetStatus()->enabled);
                    tick = servo_started_at[axis] + BABY_CARE_SERVO_STEP_MS - 1U;
                    BabyCare_Task();
                    assert(htim3.compare[axis] == 1500U);
                    tick++; BabyCare_Task();
                    assert(htim3.compare[axis] == first_compare);
                    tick += BABY_CARE_SERVO_STEP_MS - 1U; BabyCare_Task();
                    assert(htim3.compare[axis] == first_compare);
                    tick++; BabyCare_Task();
                    assert(htim3.compare[axis] != first_compare);
                    tick = remote_started + BABY_CARE_REMOTE_HOLD_MS; BabyCare_Task();
                    assert(htim3.enabled == 0U && !Interlock_GetStatus()->enabled);
                    app_remote(axis == TIM_CHANNEL_1 ? "$B110#" : "$A110#"); BabyCare_Task();
                    app_remote("$STOP#"); BabyCare_Task();
                    assert(htim3.enabled == 0U && !Interlock_GetStatus()->enabled);
                    cases++;
                }
            }
        }
    }
    printf("PASS servo start clock: %u axis/start-delay/wrap cases, exact step boundaries, hold expiry and STOP\n", cases);
    return 1;
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--servo-start-clock") == 0) {
        return test_servo_start_clock() ? 0 : 1;
    }
    if (argc == 2 && strcmp(argv[1], "--snapshot") == 0) {
        system_snapshot_t snapshot = {0};
        char payload[PROTOCOL_BUFFER_SIZE];
        char line[PROTOCOL_LINE_BUFFER_SIZE];
        snapshot.timestamp_ms = UINT32_MAX;
        snapshot.status = BABY_STATUS_SENSOR_FAULT;
        snapshot.dht11_health = SENSOR_HEALTH_READ_FAILED;
        assert(device_protocol_encode_snapshot(&snapshot,payload,sizeof(payload)) > 0U);
        assert(comm_bridge_build_report_line(payload,line,sizeof(line)) > 0U);
        puts(line);
        return 0;
    }
    test_state_machine(); test_protocol(); test_motor_initialization(); test_motor_init_failures();
    test_motor_stop_patterns(); test_motor(); test_servo(); test_queues();
    test_voice_rx_recovery(); test_esp_rx_recovery(); test_esp_configuration();
    test_debug_tx(); test_command_discard(); test_interlock(); test_interlock_power();
    test_heartbeat(); test_app_sensor_startup(); test_app_serial_faults(); test_app();
    test_desktop_demo_limits(); test_app_motor_init_failure();
    test_esp_boot_policy();
    test_voice_v3_boot_broadcast();
    assert(test_servo_start_clock());
    puts("All firmware regression groups passed.");
    return 0;
}
