/* HAL-only simulation of remote framing; never opens a serial port. */
#define main prior_firmware_main
#include "test_firmware.c"
#undef main

static unsigned gap_cases;

static int check_gap_splits(void)
{
    const char *commands[] = {"$A110#", "$B080#", "$STOP#", "$7,0,0,0#"};
    const uint32_t bases[] = {0U, UINT32_MAX - 100U};
    const uint32_t gaps[] = {249U, 250U, 251U};
    for (unsigned base = 0; base < 2; ++base) {
        for (unsigned command = 0; command < 4; ++command) {
            const char *text = commands[command];
            size_t size = strlen(text);
            for (size_t split = 1; split < size; ++split) {
                for (unsigned gap = 0; gap < 3; ++gap) {
                    ESP32Cam_HandleTypeDef h;
                    uint8_t frame[ESP32_CAM_REMOTE_FRAME_SIZE];
                    uint16_t length;
                    char prefix[32];
                    tick = bases[base];
                    assert(ESP32Cam_Init(&h, &huart3) == ESP32_CAM_OK);
                    assert(ESP32Cam_StartReceiveIT(&h) == ESP32_CAM_OK);
                    memcpy(prefix, text, split);prefix[split] = 0;
                    esp_feed(&h, prefix);
                    tick += gaps[gap];
                    /* Deliberately no foreground Task between the two IRQ bursts. */
                    esp_feed(&h, text + split);
                    assert(ESP32Cam_GetRemoteFrame(&h, frame, sizeof(frame), &length) == ESP32_CAM_OK);
                    gap_cases++;
                    if ((length != 0U) != (gaps[gap] < 250U)) {
                        printf("FAIL stale remote candidate: base=%u command=%u split=%u gap=%lu len=%u\n",
                               base, command, (unsigned)split, (unsigned long)gaps[gap], length);
                        return 0;
                    }
                    if (gaps[gap] < 250U) {
                        const char *expected = command < 2 ? text + 1 : "STOP";
                        size_t expected_size = command < 2 ? size - 2U : 4U;
                        assert(length == expected_size && memcmp(frame, expected, expected_size) == 0);
                    }
                    esp_feed(&h, "$STOP#$A100#");
                    assert(ESP32Cam_GetRemoteFrame(&h, frame, sizeof(frame), &length) == ESP32_CAM_OK);
                    assert(length == 4U && memcmp(frame, "STOP", 4U) == 0);
                    assert(ESP32Cam_GetRemoteFrame(&h, frame, sizeof(frame), &length) == ESP32_CAM_OK && length == 0U);
                }
            }
        }
    }
    printf("PASS ESP remote interbyte gaps: %u split/boundary/wrap cases; fresh STOP remains recoverable.\n", gap_cases);
    return 1;
}

static int check_application(void)
{
    const char *prefixes[] = {"$A1", "$B0"};
    const char *suffixes[] = {"10#", "80#"};
    for (unsigned base = 0; base < 2; ++base) {
        for (unsigned command = 0; command < 2; ++command) {
            tick = base ? UINT32_MAX - 100U : 0U;
            simulate_arm_latch = 1U;
            BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
            BabyCare_Init();BabyCare_Task();
            tick += BABY_CARE_INTERLOCK_STABLE_MS;BabyCare_Task();
            BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
            BabyCare_Task();tick += BABY_CARE_INTERLOCK_STABLE_MS;BabyCare_Task();
            assert(!Interlock_GetStatus()->enabled && htim3.enabled == 0U);
            app_remote(prefixes[command]);tick += 250U;app_remote(suffixes[command]);
            BabyCare_Task();
            if (Interlock_GetStatus()->enabled || htim3.enabled) {
                puts("FAIL application accepted expired ESP prefix/suffix as new servo target");return 0;
            }
            app_remote(command ? "$B080#" : "$A110#");BabyCare_Task();
            assert(Interlock_GetStatus()->enabled && htim3.enabled != 0U);
            app_remote("$BROKEN$STOP#$A100#");BabyCare_Task();
            assert(!Interlock_GetStatus()->enabled && htim3.enabled == 0U && htim2.enabled == 0U);
        }
    }
    puts("PASS ESP application gaps: four stale candidates do not enable PWM; fresh targets and nested STOP still work.");
    return 1;
}

static int check_remote_oracle(void)
{
    ESP32Cam_HandleTypeDef h;
    uint8_t body[ESP32_CAM_REMOTE_FRAME_SIZE], output[ESP32_CAM_REMOTE_FRAME_SIZE];
    uint16_t length;
    unsigned used = 0, receiving = 0, checks = 0;
    uint32_t random = 0x743250AAU, last = 0;
    const uint8_t alphabet[] = {'$', '#', 'S', 'T', 'O', 'P', 'A', '1', '0', '7', ','};
    tick = UINT32_MAX - 1000U;
    assert(ESP32Cam_Init(&h, &huart3) == ESP32_CAM_OK);
    assert(ESP32Cam_StartReceiveIT(&h) == ESP32_CAM_OK);
    for (unsigned i = 0; i < 200000; ++i) {
        random = random * 1664525U + 1013904223U;
        tick += i % 19U == 0U ? 250U : i % 37U == 0U ? 249U : 1U;
        uint8_t byte = alphabet[(random >> 24) % sizeof(alphabet)];
        int emitted = 0;
        if (receiving && (uint32_t)(tick - last) >= 250U) { receiving = 0;used = 0; }
        last = tick;
        if (byte == '$') { receiving = 1;used = 0; }
        else if (receiving && byte == '#') { receiving = 0;emitted = used != 0; }
        else if (receiving) {
            if (used < sizeof(body)) { body[used++] = byte; }
            else { receiving = 0;used = 0; }
        }
        h.rx_byte = byte;ESP32Cam_OnUartRxComplete(&h, &huart3);
        assert(ESP32Cam_GetRemoteFrame(&h, output, sizeof(output), &length) == ESP32_CAM_OK);
        checks++;
        if (emitted && ((used == 4U && memcmp(body, "STOP", 4U) == 0) ||
                        (used == 7U && memcmp(body, "7,0,0,0", 7U) == 0))) {
            assert(length == 4U && memcmp(output, "STOP", 4U) == 0);
        } else if ((length != 0U) != emitted || (emitted && (length != used || memcmp(output, body, used)))) {
            printf("FAIL remote bounded oracle byte=%u\n", i);return 0;
        }
    }
    printf("PASS ESP remote reference: %u deterministic bytes/timed gaps agree with independent bounded oracle.\n", checks);
    return 1;
}

int main(int argc, char **argv)
{
    pwm_result = tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    if (!check_gap_splits()) { return 1; }
    if (argc == 2 && strcmp(argv[1], "--repro-only") == 0) { return 0; }
    if (!check_application() || !check_remote_oracle()) { return 1; }
    return prior_firmware_main(0, NULL);
}
