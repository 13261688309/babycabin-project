/* Actual C receiver and application, with the established firmware HAL model. */
#define main prior_firmware_regression_main
#include "test_firmware.c"
#undef main
#include "vision_uplink.h"

static uint8_t packet[64];

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_u32(uint8_t *p, uint32_t n)
{
    for (unsigned i = 0; i < 4U; ++i) { p[i] = (uint8_t)(n >> (i * 8U)); }
}

static void checksum(uint8_t *p)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < 60U; ++i) {
        crc ^= p[i];
        for (unsigned bit = 0; bit < 8U; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U);
        }
    }
    put_u32(p + 60, ~crc);
}

static void fresh_packet(uint32_t sequence, uint32_t age, uint8_t label)
{
    put_u32(packet + 8, sequence);
    put_u32(packet + 12, VisionUplink_AdvertiseTick() + 1500U - age);
    put_u32(packet + 16, age);
    packet[20] = label ? 0x84 : 0; /* 900 / 1000 */
    packet[21] = label ? 3 : 0;
    packet[22] = label;
    packet[23] = label ? 1 : 0;
    checksum(packet);
}

static VisionUplink_Snapshot snapshot(void)
{
    VisionUplink_Snapshot state;
    VisionUplink_GetSnapshot(&state);
    return state;
}

static void send_packet(void)
{
    VisionUplink_OnBytes(packet, sizeof(packet));
    VisionUplink_Task();
}

static void drain_logs(unsigned *alerts, unsigned *capabilities)
{
    DebugTx_Task();
    for (unsigned i = 0; i < 32U && tx_data != NULL; ++i) {
        if (tx_length > 12U && !memcmp(tx_data, "vision_link ", 12U)) {
            printf("WIRE %.*s", (int)tx_length, tx_data);
            (*capabilities)++;
        }
        if (tx_length > 30U && !memcmp(tx_data, "POST /api/device/alert ", 23U)) {
            char text[512];
            assert(tx_length < sizeof(text));
            memcpy(text, tx_data, tx_length);
            text[tx_length] = '\0';
            if (strstr(text, "\"alert\":\"visual_cry\"") != NULL) {
                printf("WIRE %s", text);
                (*alerts)++;
            }
        }
        tx_complete();
    }
}

static int replay_python_observations(const char *path)
{
    uint8_t row[68];
    size_t count;
    unsigned alerts = 0U, capabilities = 0U, packets = 0U;
    FILE *input = fopen(path, "rb");
    assert(input != NULL);
    assert(prior_firmware_regression_main(0, NULL) == 0);
    VisionUplink_Init(); tick = 0U;
    BABY_CARE_INTERLOCK_PORT->ODR = 0U;
    tx_data = NULL; tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    BabyCare_Init(); drain_logs(&alerts, &capabilities);
    while ((count = fread(row, 1, sizeof(row), input)) != 0U) {
        assert(count == sizeof(row));
        tick = read_u32(row);
        VisionUplink_AdvertiseTick();
        VisionUplink_OnBytes(row + 4U, 17U);
        VisionUplink_OnBytes(row + 21U, 47U);
        BabyCare_Task(); drain_logs(&alerts, &capabilities);
        packets++;
        assert(htim2.enabled == 0U && htim3.enabled == 0U);
        assert((BABY_CARE_INTERLOCK_PORT->ODR & BABY_CARE_INTERLOCK_ENABLE_PIN) == 0U);
    }
    fclose(input);
    assert(packets == 8U && snapshot().accepted == 8U && alerts == 1U && capabilities == 8U);
    puts("PASS vision Python temporal replay: eight actual Python-encoded packets reach C app, one visual reminder returns, outputs unchanged; model inference and HAL simulated.");
    return 0;
}

int main(int argc, char **argv)
{
    uint8_t original[64], bad[64];
    FILE *input;
    uint32_t seq;
    unsigned alerts = 0U, capabilities = 0U;
    if (argc == 3 && strcmp(argv[2], "replay") == 0) { return replay_python_observations(argv[1]); }
    assert(argc == 2);
    input = fopen(argv[1], "rb");
    assert(input != NULL && fread(original, 1, sizeof(original), input) == sizeof(original));
    assert(fgetc(input) == EOF);
    fclose(input);
    assert(prior_firmware_regression_main(0, NULL) == 0);
    memcpy(packet, original, sizeof(packet));
    assert(read_u32(packet + 12) == 2400U);

    for (unsigned cut = 1U; cut < 64U; ++cut) {
        VisionUplink_Init(); tick = 1000U; VisionUplink_AdvertiseTick();
        VisionUplink_OnBytes(original, cut); VisionUplink_Task();
        assert(snapshot().accepted == 0U);
        VisionUplink_OnBytes(original + cut, 64U - cut); VisionUplink_Task();
        assert(snapshot().accepted == 1U && snapshot().fresh && snapshot().label == 1U);
    }
    tick = 2399U; assert(snapshot().fresh && snapshot().age_ms == 1499U);
    tick = 2400U; assert(!snapshot().fresh);

    for (unsigned field = 0U; field < 10U; ++field) {
        VisionUplink_Init(); tick = 1000U; VisionUplink_AdvertiseTick();
        memcpy(bad, original, sizeof(bad));
        switch (field) {
        case 0: bad[24] ^= 1U; break; /* different model */
        case 1: put_u32(bad + 56, 2U); break;
        case 2: bad[22] = 5U; break;
        case 3: bad[23] = 2U; break;
        case 4: bad[20] = 0U; bad[21] = 0U; break;
        case 5: put_u32(bad + 16, 1500U); break;
        case 6: put_u32(bad + 12, 2401U); break; /* unissued anchor */
        case 7: put_u32(bad + 4, 0U); break;
        case 8: bad[22] = 0U; bad[23] = 0U; break;
        default: break;
        }
        checksum(bad);
        if (field == 9U) { bad[60] ^= 1U; }
        VisionUplink_OnBytes(bad, 64U); VisionUplink_Task();
        assert(snapshot().accepted == 0U && snapshot().rejected == 1U && !snapshot().fresh);
    }
    VisionUplink_Init(); tick = 1000U; VisionUplink_AdvertiseTick();
    VisionUplink_OnBytes(original, 64U); tick = 2500U; VisionUplink_Task();
    assert(snapshot().accepted == 0U && snapshot().rejected == 1U); /* delayed USB/foreground */

    VisionUplink_Init(); tick = 1000U; VisionUplink_AdvertiseTick();
    VisionUplink_OnBytes(original, 32U); VisionUplink_Task();
    tick = 1251U; VisionUplink_OnBytes(original + 32U, 32U); VisionUplink_Task();
    assert(snapshot().accepted == 0U);
    memcpy(packet, original, sizeof(packet)); fresh_packet(1U, 0U, 1U); send_packet();
    assert(snapshot().fresh);
    send_packet(); assert(!snapshot().fresh && snapshot().rejected == 1U); /* duplicate */
    fresh_packet(0U, 0U, 1U); send_packet(); assert(snapshot().rejected == 2U); /* older sequence */
    fresh_packet(2U, 0U, 1U); put_u32(packet + 4, 7U); checksum(packet); send_packet();
    assert(snapshot().rejected == 3U); /* another host without session reset */

    VisionUplink_OnSessionReset(); assert(!snapshot().fresh && snapshot().host == 0U);
    VisionUplink_Task(); fresh_packet(0U, 0U, 1U); send_packet(); assert(snapshot().fresh);
    for (unsigned n = 0U; n < 4U; ++n) { VisionUplink_OnBytes(packet, 64U); }
    VisionUplink_Task(); assert(snapshot().drops && !snapshot().fresh);
    fresh_packet(1U, 0U, 1U); send_packet(); assert(snapshot().fresh);

    VisionUplink_Init(); tick = UINT32_MAX - 500U;
    fresh_packet(UINT32_MAX, 0U, 1U); send_packet(); assert(snapshot().fresh);
    tick += 700U; fresh_packet(0U, 0U, 1U); send_packet(); assert(snapshot().fresh);
    test_primask = 1U; (void)snapshot(); assert(test_primask == 1U); test_primask = 0U;

    VisionUplink_Init();
    for (unsigned i = 0U; i < 6U; ++i) {
        tick = 1000U + i * 1000U; fresh_packet(i, 0U, 1U); send_packet();
        assert(VisionUplink_TakeCryAlert(&seq) == (i == 3U));
    }
    assert(snapshot().alerts == 1U);
    tick = 7000U; fresh_packet(6U, 0U, 3U); send_packet();
    for (unsigned i = 0U; i < 4U; ++i) {
        tick = 8000U + i * 1000U; fresh_packet(7U + i, 0U, 1U); send_packet();
    }
    assert(snapshot().alerts == 2U && VisionUplink_TakeCryAlert(&seq));
    assert(!VisionUplink_TakeCryAlert(&seq));
    VisionUplink_Init();
    for (unsigned i = 0U; i < 5U; ++i) {
        tick = 1000U + i * 1000U; fresh_packet(i, 1000U, 1U); send_packet();
    }
    assert(snapshot().alerts == 0U); /* source age leaves a gap between frames */

    VisionUplink_Init(); tick = 0U;
    BABY_CARE_INTERLOCK_PORT->ODR = 0U;
    tx_data = NULL; tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    BabyCare_Init();
    drain_logs(&alerts, &capabilities);
    for (unsigned i = 0U; i < 5U; ++i) {
        tick = 1000U + i * 1000U;
        fresh_packet(i, 0U, 1U);
        VisionUplink_OnBytes(packet, 64U);
        BabyCare_Task();
        drain_logs(&alerts, &capabilities);
    }
    assert(alerts == 1U && capabilities > 0U);
    const uint8_t noise[] = "$STOP#$RESET#$A090#";
    VisionUplink_OnBytes(noise, sizeof(noise) - 1U); BabyCare_Task();
    assert(BabyCare_GetStatus()->mode == BABY_CARE_MODE_FAULT);
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    assert((BABY_CARE_INTERLOCK_PORT->ODR & BABY_CARE_INTERLOCK_ENABLE_PIN) == 0U);
    puts("PASS vision uplink: Python fixture, all fragment cuts, schema/model/CRC/anchor/age rejection, replay, host/session, overflow, wrap, sustained visual reminders, real app alert and outputs unchanged; HAL only.");
    return 0;
}
