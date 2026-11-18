#include "baby_care_app.h"

#include "baby_cabin_bridge.h"
#include "dht11.h"
#include "dc_motor_pwm.h"
#include "esp32_camera_link.h"
#include "servo_gimbal.h"
#include "voice_interaction.h"
#include "debug_uart_tx.h"
#include "actuator_interlock.h"
#include "system_heartbeat.h"

#include <stdio.h>
#include <string.h>

#if BABY_CARE_USE_DHT11
static DHT11_HandleTypeDef s_dht11;
static DHT11_DataTypeDef s_dht11_data;
#endif

#if BABY_CARE_USE_DC_MOTOR
static DC_Motor_HandleTypeDef s_motor;
#endif

#if BABY_CARE_USE_SERVO_GIMBAL
static ServoGimbal_HandleTypeDef s_gimbal;
#endif

#if BABY_CARE_USE_VOICE_UART
static Voice_HandleTypeDef s_voice;
#endif

#if BABY_CARE_USE_ESP32_CAMERA
static ESP32Cam_HandleTypeDef s_esp32_cam;
#endif

static BabyCare_StatusTypeDef s_status;
static uint32_t s_last_sensor_tick;
static uint32_t s_last_report_tick;
static uint32_t s_last_soothe_tick;
static uint32_t s_last_esp32_query_tick;
static uint8_t s_soothe_phase;
static uint8_t s_next_esp32_query_sta;
static uint32_t s_motion_started_ms;
static uint32_t s_remote_started_ms;
static uint8_t s_remote_active;
static uint8_t s_stop_in_cycle;
#if BABY_CARE_USE_DHT11
static uint8_t s_dht_pending;
#endif
#if BABY_CARE_USE_VOICE_UART
static uint8_t s_voice_init_pending;
static uint32_t s_voice_init_ms;
#endif
#if BABY_CARE_USE_DC_MOTOR
static uint8_t s_motor_ready;
#endif
#if BABY_CARE_USE_SERVO_GIMBAL
static uint8_t s_servo_ready;
static uint8_t s_servo_enabled;
#endif

static void BabyCare_Log(const char *message)
{
#if BABY_CARE_USE_DEBUG_UART
    if (message == NULL) {
        return;
    }

    (void)DebugTx_Write(message, strlen(message), false);
#else
    (void)message;
#endif
}

static void BabyCare_StopOutputs(void)
{
    Interlock_Disable();
#if BABY_CARE_USE_DC_MOTOR
    if (s_motor_ready != 0U) {
        (void)DC_Motor_SetSpeed(&s_motor, 0);
        if (DC_Motor_Stop(&s_motor) != DC_MOTOR_OK) {
            s_status.fault_flags |= BABY_CARE_FAULT_MOTOR;
        }
    }
#endif
#if BABY_CARE_USE_SERVO_GIMBAL
    if (s_servo_ready != 0U) {
        if (ServoGimbal_Stop(&s_gimbal) != SERVO_OK) {
            s_status.fault_flags |= BABY_CARE_FAULT_SERVO;
        }
        s_servo_enabled = 0U;
    }
#endif
    s_remote_active = 0U;
    if (s_status.fault_flags != 0U) {
        Interlock_Trip();
        s_status.safety_latched = 1U;
        s_status.mode = BABY_CARE_MODE_FAULT;
    }
}

static void BabyCare_ServiceInterlock(void)
{
    const Interlock_Status *state;
    Interlock_Poll();
    state = Interlock_GetStatus();
    if (state->tripped) {
        s_status.safety_latched = 1U;
        if (s_status.mode != BABY_CARE_MODE_ALERT) { s_status.mode = BABY_CARE_MODE_FAULT; }
        BabyCare_StopOutputs();
        s_stop_in_cycle = 1U;
    }
    s_status.interlock_stop_ok = state->stop_ok;
    s_status.interlock_armed = state->armed;
    s_status.interlock_ready = state->ready;
    s_status.interlock_enabled = state->enabled;
    s_status.interlock_trip = (uint8_t)state->reason;
    s_status.interlock_power_good = state->power_good;
    s_status.interlock_power_pending = state->power_pending;
}

void BabyCare_Stop(void)
{
    s_stop_in_cycle = 1U;
    BabyCare_StopOutputs();
    if (s_status.safety_latched == 0U) {
        s_status.mode = BABY_CARE_MODE_IDLE;
    }
}

uint8_t BabyCare_ClearSafetyLatch(void)
{
    BabyCare_Stop();
    if (s_status.fault_flags != 0U) {
        return 0U;
    }
    if (!Interlock_Reset()) { BabyCare_ServiceInterlock(); return 0U; }
#if BABY_CARE_USE_VOICE_UART
    Voice_UART_DiscardQueuedCommands(&s_voice);
#endif
#if BABY_CARE_USE_ESP32_CAMERA
    ESP32Cam_DiscardQueuedCommands(&s_esp32_cam);
#endif
    s_status.safety_latched = 0U;
    s_status.mode = BABY_CARE_MODE_IDLE;
    BabyCare_ServiceInterlock();
    return 1U;
}

static uint8_t BabyCare_EnableServos(void)
{
    BabyCare_ServiceInterlock();
    if (s_status.safety_latched || !Interlock_Enable()) {
        s_status.rejected_commands++;
        return 0U;
    }
#if BABY_CARE_USE_SERVO_GIMBAL
    if (s_servo_ready == 0U ||
        (s_servo_enabled == 0U && ServoGimbal_Start(&s_gimbal) != SERVO_OK)) {
        s_status.fault_flags |= BABY_CARE_FAULT_SERVO;
        BabyCare_StopOutputs();
        return 0U;
    }
    s_servo_enabled = 1U;
#endif
    return 1U;
}

uint8_t BabyCare_RequestMode(BabyCare_ModeTypeDef mode)
{
    if ((unsigned)mode > BABY_CARE_MODE_ALERT) {
        s_status.rejected_commands++;
        return 0U;
    }
    if (mode == BABY_CARE_MODE_IDLE) {
        BabyCare_Stop();
        return 1U;
    }
    if (mode == BABY_CARE_MODE_ALERT) {
        Interlock_Trip();
        s_status.safety_latched = 1U;
        s_status.mode = mode;
        BabyCare_StopOutputs();
        return 1U;
    }
    if (s_status.safety_latched != 0U || s_stop_in_cycle != 0U) {
        s_status.rejected_commands++;
        return 0U;
    }
    if (s_status.mode == mode) {
        return 1U;
    }

    switch (mode) {
    case BABY_CARE_MODE_SLEEP:
        BabyCare_StopOutputs();
        if (s_status.fault_flags != 0U) { return 0U; }
        s_status.mode = mode;
        BabyCare_Log("mode=sleep\r\n");
        break;

    case BABY_CARE_MODE_SOOTHE:
        if (BabyCare_EnableServos() == 0U) { return 0U; }
#if BABY_CARE_USE_DC_MOTOR
        if (s_motor_ready == 0U || DC_Motor_Start(&s_motor) != DC_MOTOR_OK) {
            s_status.fault_flags |= BABY_CARE_FAULT_MOTOR;
            BabyCare_StopOutputs();
            return 0U;
        }
#endif
        s_status.mode = mode;
        s_remote_active = 0U;
        s_motion_started_ms = HAL_GetTick();
        s_soothe_phase = 0U;
        s_last_soothe_tick = HAL_GetTick();
        BabyCare_Log("mode=soothe\r\n");
        break;

    default:
        return 0U;
    }
    return 1U;
}

static void BabyCare_HandleVoiceCommand(uint8_t command_id)
{
    s_status.latest_voice_command = command_id;

    switch (command_id) {
    case VOICE_CMD_STOP_1:
    case VOICE_CMD_STOP_2:
        BabyCare_Stop();
        break;

    case VOICE_CMD_SLEEP:
        (void)BabyCare_RequestMode(BABY_CARE_MODE_SLEEP);
        break;

    case VOICE_CMD_FORWARD:
    case VOICE_CMD_BACKWARD:
    case VOICE_CMD_ROTATE_LEFT:
    case VOICE_CMD_ROTATE_RIGHT:
        (void)BabyCare_RequestMode(BABY_CARE_MODE_SOOTHE);
        break;

    case VOICE_CMD_ALARM:
        (void)BabyCare_RequestMode(BABY_CARE_MODE_ALERT);
        break;

    default:
        break;
    }
}

static void BabyCare_PollVoice(void)
{
#if BABY_CARE_USE_VOICE_UART
    Voice_FrameTypeDef frame;

    if (Voice_UART_GetLatestFrame(&s_voice, &frame) == VOICE_OK &&
        Voice_GetEventType(&frame) == VOICE_EVENT_COMMAND) {
        BabyCare_HandleVoiceCommand(frame.id);
    }
#endif
}

static void BabyCare_RecordDhtResult(DHT11_StatusTypeDef dht_status)
{
#if BABY_CARE_USE_DHT11
    s_status.sensor_sample_count++;

    s_status.sensor_error = (uint8_t)dht_status;
    s_status.sensor_pin_level =
        (uint8_t)HAL_GPIO_ReadPin(BABY_CARE_DHT11_PORT, BABY_CARE_DHT11_PIN);

    if (dht_status == DHT11_OK) {
        s_status.temperature_c = DHT11_GetTemperature(&s_dht11_data);
        s_status.humidity_percent = DHT11_GetHumidity(&s_dht11_data);
        s_status.sensor_valid = 1U;
        s_status.sensor_last_success_ms = HAL_GetTick();
    } else {
        s_status.sensor_valid = 0U;
        s_status.sensor_failures++;
    }
#endif
}

static void BabyCare_ServiceSensors(uint32_t now)
{
#if BABY_CARE_USE_DHT11
    DHT11_StatusTypeDef status;
    if (s_dht_pending == 0U && (now - s_last_sensor_tick) >= BABY_CARE_SENSOR_PERIOD_MS) {
        s_last_sensor_tick = now;
        status = DHT11_StartRead(&s_dht11);
        if (status == DHT11_OK) { s_dht_pending = 1U; }
        else if (status != DHT11_BUSY) { BabyCare_RecordDhtResult(status); }
    }
    if (s_dht_pending != 0U) {
        status = DHT11_Task(&s_dht11, &s_dht11_data);
        if (status != DHT11_BUSY) {
            s_dht_pending = 0U;
            BabyCare_RecordDhtResult(status);
        }
    }
#else
    (void)now;
#endif
}

static void BabyCare_RunSootheMotion(void)
{
    const uint32_t now = HAL_GetTick();
    if (s_remote_active != 0U && (now - s_remote_started_ms) >= BABY_CARE_REMOTE_HOLD_MS) {
        BabyCare_Stop();
    }
    if (s_status.mode != BABY_CARE_MODE_SOOTHE) {
        return;
    }
    if ((now - s_motion_started_ms) >= BABY_CARE_SOOTHE_MAX_MS) {
        BabyCare_Stop();
        return;
    }

    if ((HAL_GetTick() - s_last_soothe_tick) < BABY_CARE_SOOTHE_PERIOD_MS) {
        return;
    }

    s_last_soothe_tick = HAL_GetTick();
    s_soothe_phase ^= 1U;

#if BABY_CARE_USE_SERVO_GIMBAL
    if (s_soothe_phase != 0U) {
        (void)ServoGimbal_SetAngles(&s_gimbal, 70U, 95U);
    } else {
        (void)ServoGimbal_SetAngles(&s_gimbal, 110U, 85U);
    }
#endif

#if BABY_CARE_USE_DC_MOTOR
    (void)DC_Motor_SetSpeed(&s_motor, (s_soothe_phase != 0U)
        ? BABY_CARE_SOOTHE_SPEED_PERCENT : -BABY_CARE_SOOTHE_SPEED_PERCENT);
#endif
}

static void BabyCare_ReportStatus(void)
{
    char line[512];
    DebugTx_Stats tx;
    const char *mode_text = "idle";
    const int32_t temp_deci = (int32_t)(s_status.temperature_c * 10.0f);
    const int32_t hum_deci = (int32_t)(s_status.humidity_percent * 10.0f);
    uint32_t voice_tx_failures = 0U;
    uint32_t voice_tx_timeouts = 0U;
    uint32_t esp_tx_failures = 0U;
    uint32_t esp_tx_timeouts = 0U;
    uint8_t esp_config_failed = 0U;
    uint8_t esp_config_result = 0U;
    uint32_t esp_config_ack_timeouts = 0U;
    uint32_t esp_config_rejections = 0U;
    uint8_t dht_raw_bytes = 0U;
    uint8_t dht_raw[5] = {0};
    int written;

    switch (s_status.mode) {
    case BABY_CARE_MODE_SLEEP:
        mode_text = "sleep";
        break;
    case BABY_CARE_MODE_SOOTHE:
        mode_text = "soothe";
        break;
    case BABY_CARE_MODE_ALERT:
        mode_text = "alert";
        break;
    case BABY_CARE_MODE_FAULT:
        mode_text = "fault";
        break;
    case BABY_CARE_MODE_IDLE:
    default:
        break;
    }

    DebugTx_GetStats(&tx);
#if BABY_CARE_USE_DHT11
    dht_raw_bytes = s_dht11.raw_bytes;
    memcpy(dht_raw, s_dht11.last_raw, sizeof(dht_raw));
#endif
#if BABY_CARE_USE_VOICE_UART
    voice_tx_failures = s_voice.tx_failures;
    voice_tx_timeouts = s_voice.tx_timeouts;
#endif
#if BABY_CARE_USE_ESP32_CAMERA
    esp_tx_failures = s_esp32_cam.tx_failures;
    esp_tx_timeouts = s_esp32_cam.tx_timeouts;
    esp_config_failed = s_esp32_cam.configure_failed;
    esp_config_result = (uint8_t)s_esp32_cam.configure_result;
    esp_config_ack_timeouts = s_esp32_cam.configure_ack_timeouts;
    esp_config_rejections = s_esp32_cam.configure_rejections;
#endif
    written = snprintf(
        line,
        sizeof(line),
        "status mode=%s temp=%ld.%ldC hum=%ld.%ld%% sensor=%u dht=%u pin=%u voice=0x%02X tx_drop=%lu tx_err=%lu tx_timeout=%lu vtx_fail=%lu vtx_timeout=%lu etx_fail=%lu etx_timeout=%lu esp_cfg_fail=%u esp_cfg_result=%u esp_cfg_ack_timeout=%lu esp_cfg_reject=%lu il_stop=%u il_arm=%u il_ready=%u il_en=%u il_trip=%u il_pg=%u il_pending=%u dht_n=%u dht_raw=%02X%02X%02X%02X%02X\r\n",
        mode_text,
        (long)(temp_deci / 10),
        (long)(temp_deci % 10),
        (long)(hum_deci / 10),
        (long)(hum_deci % 10),
        s_status.sensor_valid,
        s_status.sensor_error,
        s_status.sensor_pin_level,
        s_status.latest_voice_command,
        (unsigned long)tx.dropped,
        (unsigned long)tx.start_errors,
        (unsigned long)tx.timeouts,
        (unsigned long)voice_tx_failures,
        (unsigned long)voice_tx_timeouts,
        (unsigned long)esp_tx_failures,
        (unsigned long)esp_tx_timeouts,
        esp_config_failed,
        esp_config_result,
        (unsigned long)esp_config_ack_timeouts,
        (unsigned long)esp_config_rejections,
        s_status.interlock_stop_ok, s_status.interlock_armed,
        s_status.interlock_ready, s_status.interlock_enabled, s_status.interlock_trip,
        s_status.interlock_power_good, s_status.interlock_power_pending,
        (unsigned int)dht_raw_bytes,
        (unsigned int)dht_raw[0], (unsigned int)dht_raw[1],
        (unsigned int)dht_raw[2], (unsigned int)dht_raw[3], (unsigned int)dht_raw[4]);
    if (written > 0 && (size_t)written < sizeof(line)) {
        BabyCare_Log(line);
    }
}

static void BabyCare_HandleEsp32RemoteFrame(void)
{
#if BABY_CARE_USE_ESP32_CAMERA
    uint8_t frame[ESP32_CAM_REMOTE_FRAME_SIZE];
    uint16_t len = 0U;
#if BABY_CARE_USE_SERVO_GIMBAL
    uint8_t index;
    uint8_t angle;
#endif

    if (ESP32Cam_GetRemoteFrame(&s_esp32_cam, frame, sizeof(frame), &len) != ESP32_CAM_OK ||
        len == 0U) {
        return;
    }

    if (len == 4U && memcmp(frame, "STOP", 4U) == 0) {
        BabyCare_Stop();
        return;
    }
    if (len == 5U && memcmp(frame, "RESET", 5U) == 0) {
        (void)BabyCare_ClearSafetyLatch();
        return;
    }
#if BABY_CARE_USE_SERVO_GIMBAL
    if (ServoGimbal_DecodeRemoteFrame(frame, len, &index, &angle) != SERVO_OK ||
        s_status.safety_latched != 0U || s_stop_in_cycle != 0U) {
        s_status.rejected_commands++;
        return;
    }
    /* Enter remote mode once; later axis commands must preserve the other target. */
    if (s_remote_active == 0U) {
        BabyCare_StopOutputs();
    }
    if (s_status.fault_flags != 0U || BabyCare_EnableServos() == 0U) { return; }
    s_status.mode = BABY_CARE_MODE_IDLE;
    (void)ServoGimbal_SetByIndex(&s_gimbal, index, angle);
    s_remote_started_ms = HAL_GetTick();
    s_remote_active = 1U;
#else
    s_status.rejected_commands++;
#endif
#endif
}

static void BabyCare_PollEsp32Camera(void)
{
#if BABY_CARE_USE_ESP32_CAMERA
    ESP32Cam_LinkStatusTypeDef status;
    char line[96];
    const uint32_t now = HAL_GetTick();

    BabyCare_HandleEsp32RemoteFrame();

    if (s_esp32_cam.configuring == 0U &&
        (now - s_last_esp32_query_tick) >= BABY_CARE_ESP32_QUERY_PERIOD_MS) {
        s_last_esp32_query_tick = now;

        if (s_next_esp32_query_sta != 0U) {
            (void)ESP32Cam_RequestStaIp(&s_esp32_cam);
        } else {
            (void)ESP32Cam_RequestApIp(&s_esp32_cam);
        }
        s_next_esp32_query_sta ^= 1U;
    }

    if (ESP32Cam_GetStatus(&s_esp32_cam, &status) == ESP32_CAM_OK &&
        status.updated != 0U) {
        s_status.esp32_ap_ip_valid = status.ap_ip_valid;
        s_status.esp32_sta_ip_valid = status.sta_ip_valid;
        (void)strncpy(s_status.esp32_ap_ip,
                      status.ap_ip,
                      sizeof(s_status.esp32_ap_ip) - 1U);
        s_status.esp32_ap_ip[sizeof(s_status.esp32_ap_ip) - 1U] = '\0';
        (void)strncpy(s_status.esp32_sta_ip,
                      status.sta_ip,
                      sizeof(s_status.esp32_sta_ip) - 1U);
        s_status.esp32_sta_ip[sizeof(s_status.esp32_sta_ip) - 1U] = '\0';

        (void)snprintf(line,
                       sizeof(line),
                       "esp32 ap_ip=%s sta_ip=%s\r\n",
                       (s_status.esp32_ap_ip_valid != 0U) ? s_status.esp32_ap_ip : "null",
                       (s_status.esp32_sta_ip_valid != 0U) ? s_status.esp32_sta_ip : "null");
        BabyCare_Log(line);
    }
#endif
}

void BabyCare_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.mode = BABY_CARE_MODE_IDLE;
    SystemHeartbeat_Init();
    Interlock_Init();

#if BABY_CARE_USE_DEBUG_UART
    (void)DebugTx_Init(&BABY_CARE_DEBUG_UART);
#endif
#if BABY_CARE_USE_DHT11
    DHT11_Init(&s_dht11, BABY_CARE_DHT11_PORT, BABY_CARE_DHT11_PIN);
    s_dht_pending = 0U;
#endif

#if BABY_CARE_USE_DC_MOTOR
    s_motor_ready = (DC_Motor_Init(&s_motor,
                        &BABY_CARE_MOTOR_TIM,
                        BABY_CARE_MOTOR_CHANNEL,
                        BABY_CARE_MOTOR_IN1_PORT,
                        BABY_CARE_MOTOR_IN1_PIN,
                        BABY_CARE_MOTOR_IN2_PORT,
                        BABY_CARE_MOTOR_IN2_PIN) == DC_MOTOR_OK);
    if (s_motor_ready == 0U) { s_status.fault_flags |= BABY_CARE_FAULT_MOTOR; }
#endif

#if BABY_CARE_USE_SERVO_GIMBAL
    s_servo_ready = (ServoGimbal_Init(&s_gimbal,
                           &BABY_CARE_GIMBAL_PAN_TIM,
                           BABY_CARE_GIMBAL_PAN_CH,
                           &BABY_CARE_GIMBAL_TILT_TIM,
                           BABY_CARE_GIMBAL_TILT_CH) == SERVO_OK);
    if (s_servo_ready != 0U) {
        Servo_SetPulseRange(&s_gimbal.pan, BABY_CARE_SERVO_MIN_PULSE_US,
                           BABY_CARE_SERVO_MAX_PULSE_US);
        Servo_SetPulseRange(&s_gimbal.tilt, BABY_CARE_SERVO_MIN_PULSE_US,
                           BABY_CARE_SERVO_MAX_PULSE_US);
    }
    if (s_servo_ready != 0U &&
        (ServoGimbal_SetSoftLimits(&s_gimbal,
             BABY_CARE_PAN_SAFE_MIN_DEG, BABY_CARE_PAN_SAFE_MAX_DEG,
             BABY_CARE_TILT_SAFE_MIN_DEG, BABY_CARE_TILT_SAFE_MAX_DEG) != SERVO_OK ||
         ServoGimbal_SetMotionProfile(&s_gimbal,
             BABY_CARE_SERVO_STEP_DEG, BABY_CARE_SERVO_STEP_MS) != SERVO_OK)) {
        s_servo_ready = 0U;
    }
    s_servo_enabled = 0U;
    if (s_servo_ready == 0U) { s_status.fault_flags |= BABY_CARE_FAULT_SERVO; }
#endif

#if BABY_CARE_USE_VOICE_UART
    Voice_Init(&s_voice, &BABY_CARE_VOICE_UART, NULL);
    (void)Voice_UART_StartReceiveIT(&s_voice);
    s_voice_init_pending = 1U;
    s_voice_init_ms = HAL_GetTick();
#endif

#if BABY_CARE_USE_ESP32_CAMERA
    (void)ESP32Cam_Init(&s_esp32_cam, &BABY_CARE_ESP32_UART);
    (void)ESP32Cam_StartReceiveIT(&s_esp32_cam);
#if BABY_CARE_ESP32_CONFIGURE_ON_BOOT
    (void)ESP32Cam_ConfigureDefault(&s_esp32_cam);
#endif
    s_last_esp32_query_tick = HAL_GetTick();
    s_next_esp32_query_sta = 0U;
#endif

    BabyCabinBridge_Init();
    BabyCare_StopOutputs();
    s_last_sensor_tick = HAL_GetTick();
    s_last_report_tick = HAL_GetTick();
    s_remote_active = 0U;
    s_stop_in_cycle = 0U;
    BabyCare_Log("baby care app init done\r\n");
}

void BabyCare_Task(void)
{
    const uint32_t now = HAL_GetTick();
    s_stop_in_cycle = 0U;
    BabyCare_ServiceInterlock();

#if BABY_CARE_USE_VOICE_UART
    Voice_UART_Task(&s_voice);
    if (s_voice_init_pending != 0U && (now - s_voice_init_ms) >= 100U) {
        if (Voice_UART_PlayBroadcast(&s_voice, VOICE_BROADCAST_INIT_DONE) == VOICE_OK) {
            s_voice_init_pending = 0U;
        }
    }
#endif
#if BABY_CARE_USE_ESP32_CAMERA
    ESP32Cam_Task(&s_esp32_cam);
#endif
    BabyCare_PollVoice();
    BabyCare_PollEsp32Camera();
    BabyCare_RunSootheMotion();
#if BABY_CARE_USE_DC_MOTOR
    DC_Motor_Task(&s_motor);
#endif
#if BABY_CARE_USE_SERVO_GIMBAL
    /* A command above may have started PWM after the task's entry timestamp. */
    if (s_servo_enabled != 0U && ServoGimbal_Task(&s_gimbal, HAL_GetTick()) != SERVO_OK) {
        s_status.fault_flags |= BABY_CARE_FAULT_SERVO;
        BabyCare_StopOutputs();
    }
#endif

    BabyCare_ServiceSensors(now);
    BabyCabinBridge_Task();
    BabyCare_ServiceInterlock();

    if ((now - s_last_report_tick) >= BABY_CARE_REPORT_PERIOD_MS) {
        s_last_report_tick = now;
        BabyCare_ReportStatus();
    }
    DebugTx_Task();
    SystemHeartbeat_Service();
}

void BabyCare_OnUartRxComplete(UART_HandleTypeDef *huart)
{
#if BABY_CARE_USE_VOICE_UART
    (void)Voice_UART_RxCpltCallback(&s_voice, huart);
#endif
#if BABY_CARE_USE_ESP32_CAMERA
    ESP32Cam_OnUartRxComplete(&s_esp32_cam, huart);
#endif
#if !BABY_CARE_USE_VOICE_UART && !BABY_CARE_USE_ESP32_CAMERA
    (void)huart;
#endif
}

void BabyCare_OnUartTxComplete(UART_HandleTypeDef *huart)
{
    DebugTx_OnComplete(huart);
#if BABY_CARE_USE_VOICE_UART
    Voice_UART_OnTxComplete(&s_voice, huart);
#endif
#if BABY_CARE_USE_ESP32_CAMERA
    ESP32Cam_OnUartTxComplete(&s_esp32_cam, huart);
#endif
    (void)huart;
}

void BabyCare_OnUartError(UART_HandleTypeDef *huart)
{
    DebugTx_OnError(huart);
#if BABY_CARE_USE_VOICE_UART
    Voice_UART_OnError(&s_voice, huart);
#endif
#if BABY_CARE_USE_ESP32_CAMERA
    ESP32Cam_OnUartError(&s_esp32_cam, huart);
#endif
    (void)huart;
}

const BabyCare_StatusTypeDef *BabyCare_GetStatus(void)
{
    const Interlock_Status *state = Interlock_GetStatus();
    s_status.interlock_stop_ok = state->stop_ok;
    s_status.interlock_armed = state->armed;
    s_status.interlock_ready = state->ready;
    s_status.interlock_enabled = state->enabled;
    s_status.interlock_trip = (uint8_t)state->reason;
    s_status.interlock_power_good = state->power_good;
    s_status.interlock_power_pending = state->power_pending;
    return &s_status;
}
