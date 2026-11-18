/* Reuse the existing HAL doubles and application regression without editing them. */
#define main prior_firmware_regression_main
#include "test_firmware.c"
#undef main

static uint32_t resync_checks;

static int expect_frame(Voice_HandleTypeDef *v, uint8_t type, uint8_t id)
{
    Voice_FrameTypeDef f;
    resync_checks++;
    if (Voice_UART_GetLatestFrame(v, &f) != VOICE_OK || f.type != type || f.id != id) {
        printf("FAIL voice resync expected type=%u id=%u index=%u\n", type, id, v->rx_index);
        return 0;
    }
    return Voice_UART_GetLatestFrame(v, &f) == VOICE_ERROR_NO_FRAME;
}

static void feed_raw(Voice_HandleTypeDef *v, const uint8_t *bytes, size_t length, int irq)
{
    for (size_t i = 0; i < length; ++i) {
        if (irq) {
            v->rx_byte = bytes[i];
            (void)Voice_UART_RxCpltCallback(v, v->huart);
        } else {
            (void)Voice_UART_ParseByte(v, bytes[i], NULL);
        }
    }
}

static int test_truncated_prefixes(void)
{
    const uint8_t prefix[] = {0xAA, 0x55, 0, VOICE_CMD_FORWARD};
    const uint8_t urgent[] = {VOICE_CMD_STOP_1, VOICE_CMD_STOP_2, VOICE_CMD_ALARM};
    for (int irq = 0; irq < 2; ++irq) {
        for (size_t length = 1; length < sizeof(prefix) + 1; ++length) {
            for (size_t j = 0; j < sizeof(urgent); ++j) {
                Voice_HandleTypeDef v;
                Voice_Init(&v, &huart2, NULL);
                if (irq) { assert(Voice_UART_StartReceiveIT(&v) == VOICE_OK); }
                const uint8_t stop[] = {0xAA, 0x55, 0, urgent[j], 0xFB};
                feed_raw(&v, prefix, length, irq);
                feed_raw(&v, stop, sizeof(stop), irq);
                if (!expect_frame(&v, 0, urgent[j])) {
                    printf("Truncated prefix length=%u IRQ=%d\n", (unsigned)length, irq);
                    return 0;
                }
            }
        }
    }
    puts("PASS voice resync: truncated prefixes followed immediately by both STOP IDs and ALARM via parser/IRQ");
    return 1;
}

static int test_all_valid_payloads(void)
{
    Voice_HandleTypeDef v;
    Voice_Init(&v, &huart2, NULL);
    for (unsigned type = 0; type < 256; ++type) {
        for (unsigned id = 0; id < 256; ++id) {
            const uint8_t bytes[] = {0xAA, 0x55, (uint8_t)type, (uint8_t)id, 0xFB};
            feed_raw(&v, bytes, sizeof(bytes), 0);
            if (!expect_frame(&v, (uint8_t)type, (uint8_t)id)) { return 0; }
        }
    }
    puts("PASS voice resync: all 65536 type/ID payloads retained, including embedded header bytes");
    return 1;
}

static int test_gap_wrap_and_urgent_priority(void)
{
    Voice_HandleTypeDef v;
    Voice_FrameTypeDef frame;
    const uint8_t partial[] = {0xAA, 0x55, 0};
    Voice_Init(&v, &huart2, NULL);
    tick = UINT32_MAX - 10U;
    feed_raw(&v, partial, sizeof(partial), 0);
    tick += VOICE_FRAME_GAP_MS;
    voice_feed(&v, VOICE_CMD_STOP_1);
    if (!expect_frame(&v, 0, VOICE_CMD_STOP_1)) { return 0; }
    voice_feed(&v, VOICE_CMD_FORWARD);
    feed_raw(&v, partial, sizeof(partial), 0);
    voice_feed(&v, VOICE_CMD_STOP_2);
    voice_feed(&v, VOICE_CMD_FORWARD);
    voice_feed(&v, VOICE_CMD_ALARM);
    voice_feed(&v, VOICE_CMD_STOP_1);
    if (!expect_frame(&v, 0, VOICE_CMD_ALARM)) { return 0; }
    assert(Voice_UART_GetLatestFrame(&v, &frame) == VOICE_ERROR_NO_FRAME);
    puts("PASS voice resync: gap timeout, tick wrap, queued motion discarded and ALARM priority retained");
    return 1;
}

static int test_window_reference(void)
{
    Voice_HandleTypeDef v;
    uint8_t window[5];
    size_t size = 0;
    uint32_t random = 0x743055AAU;
    Voice_Init(&v, &huart2, NULL);
    for (unsigned i = 0; i < 200000; ++i) {
        Voice_FrameTypeDef actual;
        const uint8_t biased[] = {0xAA, 0x55, 0, 1, 2, 0x26, 0xFB};
        random = random * 1664525U + 1013904223U;
        uint8_t byte = (i % 7U < 5U) ? biased[(random >> 24) % sizeof(biased)] : (uint8_t)(random >> 16);
        if (size == sizeof(window)) { memmove(window, window + 1, --size); }
        window[size++] = byte;
        int expected = size == sizeof(window) && window[0] == 0xAA && window[1] == 0x55 && window[4] == 0xFB;
        (void)Voice_UART_ParseByte(&v, byte, NULL);
        Voice_StatusTypeDef status = Voice_UART_GetLatestFrame(&v, &actual);
        resync_checks++;
        if ((status == VOICE_OK) != expected || (expected && memcmp(actual.raw, window, sizeof(window)))) {
            printf("FAIL independent sliding-window reference at byte=%u\n", i);
            return 0;
        }
        if (expected) { size = 0; }
        if (i % 101U == 0U) { tick++; }
    }
    puts("PASS voice resync: 200000 deterministic noisy bytes agree with independent sliding-window oracle");
    return 1;
}

int main(int argc, char **argv)
{
    if (!test_truncated_prefixes()) { return 1; }
    if (argc == 2 && strcmp(argv[1], "--repro-only") == 0) { return 0; }
    if (!test_all_valid_payloads() || !test_gap_wrap_and_urgent_priority() || !test_window_reference()) { return 1; }
    if (prior_firmware_regression_main(0, NULL) != 0) { return 1; }
    printf("All voice resync checks passed: %u checks; host simulation, not hardware.\n", resync_checks);
    return 0;
}
