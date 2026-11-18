/* Integrated candidate tests retain the current servo-clock regression HAL. */
#define main prior_firmware_regression_main
#include "test_firmware.c"
#undef main

static void receive_frame(Voice_HandleTypeDef *voice, uint8_t type, uint8_t id)
{
    const uint8_t raw[] = {0xAA, 0x55, type, id, 0xFB};
    for (size_t i = 0; i < sizeof(raw); ++i) {
        voice->rx_byte = raw[i];
        (void)Voice_UART_RxCpltCallback(voice, voice->huart);
    }
}

static void expect_app_log(uint32_t now, uint32_t frames, uint32_t applied, uint32_t apply_ms, uint32_t rx_ms)
{
    char expected[512];
    int length = snprintf(expected, sizeof(expected),
        "voice_rx v=2 frames=%lu stops=%lu alarms=0 last_ms=%lu stop_ms=%lu errors=0 dropped=0 "
        "applied=%lu apply_ms=%lu now_ms=%lu\r\n", (unsigned long)frames, (unsigned long)frames,
        (unsigned long)rx_ms, (unsigned long)rx_ms,
        (unsigned long)applied, (unsigned long)apply_ms, (unsigned long)now);
    unsigned found = 0U;
    assert(length > 0 && (size_t)length < sizeof(expected));
    DebugTx_Task();
    for (unsigned i = 0; i < 32U && tx_data != NULL; ++i) {
        if (tx_length == (unsigned)length && !memcmp(tx_data, expected, tx_length)) {
            found++;
        }
        tx_complete();
    }
    assert(found == 1U);
    printf("WIRE %s", expected);
}

int main(void)
{
    Voice_HandleTypeDef voice;
    Voice_RxStatsTypeDef stats;
    Voice_FrameTypeDef frame;
    prior_firmware_regression_main(0, NULL);
    rx_result = HAL_OK;
    tick = 0U;
    test_primask = 0U;
    Voice_Init(&voice, &huart2, NULL);
    assert(Voice_UART_StartReceiveIT(&voice) == VOICE_OK);
    assert(Voice_UART_GetRxStats(NULL, &stats) == VOICE_ERROR_PARAM);
    assert(Voice_UART_GetRxStats(&voice, NULL) == VOICE_ERROR_PARAM);
    assert(Voice_UART_GetRxStats(&voice, &stats) == VOICE_OK);
    assert(stats.frames == 0U && stats.stop_frames == 0U);

    receive_frame(&voice, VOICE_UART_CMD_TYPE, VOICE_CMD_STOP_1);
    tick = 7U;
    receive_frame(&voice, VOICE_UART_CMD_TYPE, VOICE_CMD_STOP_1);
    assert(Voice_UART_GetRxStats(&voice, &stats) == VOICE_OK);
    assert(stats.frames == 2U && stats.stop_frames == 2U && stats.last_stop_ms == 7U);
    assert(Voice_UART_GetLatestFrame(&voice, &frame) == VOICE_OK);
    assert(Voice_UART_GetLatestFrame(&voice, &frame) == VOICE_ERROR_NO_FRAME);
    assert(Voice_UART_GetRxStats(&voice, &stats) == VOICE_OK && stats.frames == 2U);

    tick = UINT32_MAX;
    receive_frame(&voice, VOICE_UART_CMD_TYPE, VOICE_CMD_STOP_2);
    tick = 0U;
    receive_frame(&voice, VOICE_UART_CMD_TYPE, VOICE_CMD_ALARM);
    receive_frame(&voice, VOICE_SYS_WELCOME, VOICE_CMD_STOP_1);
    assert(Voice_UART_GetRxStats(&voice, &stats) == VOICE_OK);
    assert(stats.frames == 5U && stats.stop_frames == 3U && stats.alarm_frames == 1U);
    assert(stats.last_frame_ms == 0U && stats.last_stop_ms == UINT32_MAX);
    assert(stats.dropped_frames == 1U);
    assert(Voice_UART_ParseByte(&voice, 0x42, NULL) == VOICE_ERROR_NO_FRAME);
    const uint8_t bad[] = {0xAA, 0x55, 0, 1, 0};
    for (size_t i = 0; i < sizeof(bad); ++i) { (void)Voice_UART_ParseByte(&voice, bad[i], NULL); }
    Voice_UART_OnError(&voice, &huart3);
    Voice_UART_OnError(&voice, &huart2);
    Voice_UART_DiscardQueuedCommands(&voice);
    test_primask = 1U;
    assert(Voice_UART_GetRxStats(&voice, &stats) == VOICE_OK);
    assert(test_primask == 1U && stats.frames == 5U && stats.uart_errors == 1U);
    test_primask = 0U;
    assert(Voice_UART_GetRxStats(&voice, &stats) == VOICE_OK && test_primask == 0U);

    Voice_Init(&voice, &huart2, NULL);
    for (unsigned i = 0; i < VOICE_QUEUE_SIZE + 2U; ++i) {
        receive_frame(&voice, VOICE_UART_CMD_TYPE, VOICE_CMD_FORWARD);
    }
    assert(Voice_UART_GetRxStats(&voice, &stats) == VOICE_OK);
    assert(stats.frames == VOICE_QUEUE_SIZE + 2U && stats.dropped_frames == 3U);
    voice.rx_frames = UINT32_MAX;
    voice.rx_stop_frames = UINT32_MAX;
    tick = 42U;
    receive_frame(&voice, VOICE_UART_CMD_TYPE, VOICE_CMD_STOP_1);
    assert(Voice_UART_GetRxStats(&voice, &stats) == VOICE_OK);
    assert(stats.frames == 0U && stats.stop_frames == 0U && stats.last_stop_ms == 42U);
    Voice_Init(&voice, &huart2, NULL);
    assert(Voice_UART_GetRxStats(&voice, &stats) == VOICE_OK);
    assert(stats.frames == 0U && stats.last_stop_ms == 0U && stats.uart_errors == 0U);
    tick = 0U;
    tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    tx_data = NULL;
    BABY_CARE_INTERLOCK_PORT->ODR = 0U;
    BabyCare_Init();
    for (unsigned i = 0; i < 32U; ++i) { tx_complete(); }
    tick = 20U;
    app_voice(VOICE_CMD_STOP_1);
    app_voice(VOICE_CMD_STOP_1);
    tick = 2000U;
    BabyCare_Task();
    expect_app_log(2000U, 2U, 1U, 2000U, 20U);
    tick = 4000U;
    BabyCare_Task();
    expect_app_log(4000U, 2U, 1U, 2000U, 20U);
    assert(BabyCare_GetStatus()->voice_stop_applied == 1U);
    BabyCare_Stop();
    assert(BabyCare_GetStatus()->voice_stop_applied == 1U);
    tick = 4020U;
    app_voice(VOICE_CMD_STOP_1);
    tick = 6000U;
    BabyCare_Task();
    expect_app_log(6000U, 3U, 2U, 6000U, 4020U);
    assert(BabyCare_GetStatus()->voice_stop_applied == 2U);
    BabyCare_Init();
    assert(BabyCare_GetStatus()->voice_stop_applied == 0U);
    assert(BabyCare_GetStatus()->voice_stop_applied_ms == 0U);
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    puts("PASS integrated STOP: repeated/coalesced RX distinct from applied count; periodic reports do not create events; generic Stop excluded; init resets; host only.");
    puts("PASS voice RX observation: repeated STOP, coalescing, types, invalid frames, errors, overflow, IRQ state, tick/count wrap and reset; host only.");
    return 0;
}
