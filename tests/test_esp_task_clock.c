/* Inject an ideal UART IRQ immediately before a foreground critical section. */
#define main prior_firmware_main
#include "test_firmware.c"
#undef main

static ESP32Cam_HandleTypeDef *inject_handle;
static const char *inject_text;
static uint32_t inject_advance;
static unsigned inject_at, critical_entries, injected;

void test_clock_disable_irq(void)
{
    if (test_primask == 0U && inject_at != 0U && ++critical_entries == inject_at) {
        inject_at = 0U;
        tick += inject_advance;
        esp_feed(inject_handle, inject_text);
        injected++;
    }
    test_primask = 1U;
}

static void init_camera(ESP32Cam_HandleTypeDef *handle, uint32_t start)
{
    inject_at = 0U;
    test_primask = 0U;
    tick = start;
    assert(ESP32Cam_Init(handle, &huart3) == ESP32_CAM_OK);
    assert(ESP32Cam_StartReceiveIT(handle) == ESP32_CAM_OK);
}

static void inject_during_task(ESP32Cam_HandleTypeDef *handle, const char *text,
                               uint32_t advance, unsigned point)
{
    inject_handle = handle;
    inject_text = text;
    inject_advance = advance;
    inject_at = point;
    critical_entries = injected = 0U;
    ESP32Cam_Task(handle);
    assert(injected == 1U && inject_at == 0U && test_primask == 0U);
}

static int check_prefix(const char *command, size_t split, uint32_t start,
                        uint32_t advance, unsigned point)
{
    ESP32Cam_HandleTypeDef handle;
    char prefix[32];
    uint8_t frame[ESP32_CAM_REMOTE_FRAME_SIZE];
    uint16_t length;
    const char *expected = command[1] == 'A' || command[1] == 'B' ? command + 1 : "STOP";
    const size_t expected_size = expected == command + 1 ? strlen(command) - 2U : 4U;
    memcpy(prefix, command, split);
    prefix[split] = 0;
    init_camera(&handle, start);
    inject_during_task(&handle, prefix, advance, point);
    esp_feed(&handle, command + split);
    assert(ESP32Cam_GetRemoteFrame(&handle, frame, sizeof(frame), &length) == ESP32_CAM_OK);
    if (length != expected_size || memcmp(frame, expected, length) != 0) {
        printf("FAIL fresh command lost: command=%s split=%u start=%lu advance=%lu critical=%u length=%u\n",
               command, (unsigned)split, (unsigned long)start, (unsigned long)advance, point, length);
        return 0;
    }
    return 1;
}

static int check_ip(unsigned sta, uint32_t start, uint32_t advance, unsigned point)
{
    ESP32Cam_HandleTypeDef handle;
    ESP32Cam_LinkStatusTypeDef status;
    init_camera(&handle, start);
    inject_during_task(&handle, sta ? "sta_ip:192.168.1.7\n" : "ap_ip:192.168.4.1\n", advance, point);
    assert(ESP32Cam_GetStatus(&handle, &status) == ESP32_CAM_OK);
    if (!(sta ? status.sta_ip_valid : status.ap_ip_valid)) {
        printf("FAIL fresh %s IP expired: start=%lu advance=%lu critical=%u\n",
               sta ? "STA" : "AP", (unsigned long)start, (unsigned long)advance, point);
        return 0;
    }
    tick += 14999U;
    ESP32Cam_Task(&handle);
    assert(ESP32Cam_GetStatus(&handle, &status) == ESP32_CAM_OK);
    assert(sta ? status.sta_ip_valid : status.ap_ip_valid);
    tick++;
    ESP32Cam_Task(&handle);
    assert(ESP32Cam_GetStatus(&handle, &status) == ESP32_CAM_OK);
    assert(!(sta ? status.sta_ip_valid : status.ap_ip_valid));
    return 1;
}

static void check_expiry_and_mask(void)
{
    const uint32_t starts[] = {0U, UINT32_MAX - 100U};
    const uint32_t ages[] = {249U, 250U, 251U};
    for (unsigned base = 0; base < 2U; ++base) {
        for (unsigned age = 0; age < 3U; ++age) {
            ESP32Cam_HandleTypeDef handle;
            uint8_t frame[ESP32_CAM_REMOTE_FRAME_SIZE];
            uint16_t length;
            init_camera(&handle, starts[base]);
            esp_feed(&handle, "$ST");
            tick += ages[age];
            test_primask = 1U;
            ESP32Cam_Task(&handle);
            assert(test_primask == 1U);
            test_primask = 0U;
            esp_feed(&handle, "OP#");
            assert(ESP32Cam_GetRemoteFrame(&handle, frame, sizeof(frame), &length) == ESP32_CAM_OK);
            assert((length == 4U) == (ages[age] < 250U));
            if (length) { assert(memcmp(frame, "STOP", length) == 0); }
        }
    }
    puts("PASS existing 250ms expiry boundary, tick wrap and incoming PRIMASK preservation");
}

int main(int argc, char **argv)
{
    const char *commands[] = {"$A110#", "$B080#", "$STOP#", "$7,0,0,0#"};
    const uint32_t starts[] = {0U, 100U, UINT32_MAX};
    const uint32_t advances[] = {0U, 1U, 2U, 249U, 250U, 15000U};
    unsigned prefixes = 0U, ips = 0U;
    pwm_result = tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    if (argc == 2 && strcmp(argv[1], "--repro-stop") == 0) {
        return check_prefix("$STOP#", 1U, 100U, 1U, 2U) ? 0 : 1;
    }
    if (argc == 2 && strcmp(argv[1], "--repro-ap") == 0) { return check_ip(0U, 100U, 1U, 2U) ? 0 : 1; }
    if (argc == 2 && strcmp(argv[1], "--repro-sta") == 0) { return check_ip(1U, 100U, 1U, 2U) ? 0 : 1; }
    for (unsigned base = 0; base < 3U; ++base) {
        for (unsigned delta = 0; delta < 6U; ++delta) {
            for (unsigned point = 1U; point <= 2U; ++point) {
                for (unsigned command = 0; command < 4U; ++command) {
                    for (size_t split = 1; split < strlen(commands[command]); ++split) {
                        if (!check_prefix(commands[command], split, starts[base], advances[delta], point)) { return 1; }
                        prefixes++;
                    }
                }
                for (unsigned sta = 0; sta < 2U; ++sta) {
                    if (!check_ip(sta, starts[base], advances[delta], point)) { return 1; }
                    ips++;
                }
            }
        }
    }
    printf("PASS fresh IRQ time snapshots: %u split-command cases and %u AP/STA expiry cases\n", prefixes, ips);
    check_expiry_and_mask();
    return prior_firmware_main(0, NULL);
}
