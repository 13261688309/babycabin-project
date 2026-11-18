/* Reuse the existing HAL doubles; this executable never accesses hardware. */
#define main prior_firmware_regression_main
#include "test_firmware.c"
#undef main

int main(int argc, char **argv)
{
    Voice_HandleTypeDef voice;
    unsigned long received_ms;
    unsigned byte;
    unsigned count = 0;
    int scanned;
    if (argc == 2 && strcmp(argv[1], "--self-test") == 0) {
        return prior_firmware_regression_main(0, NULL);
    }
    if (argc != 1) { return 2; }
    Voice_Init(&voice, &huart2, NULL);
    while ((scanned = scanf("%lu %x", &received_ms, &byte)) == 2) {
        Voice_FrameTypeDef frame;
        if (received_ms > 300000UL || byte > 255U || ++count > 65536U) { return 2; }
        tick = (uint32_t)received_ms;
        if (Voice_UART_ParseByte(&voice, (uint8_t)byte, &frame) == VOICE_OK) {
            printf("{\"receivedOffsetMs\":%lu,\"rawHex\":\"%02x%02x%02x%02x%02x\",\"type\":%u,\"id\":%u}\n",
                   received_ms, (unsigned)frame.raw[0], (unsigned)frame.raw[1],
                   (unsigned)frame.raw[2], (unsigned)frame.raw[3], (unsigned)frame.raw[4],
                   (unsigned)frame.type, (unsigned)frame.id);
        }
    }
    return scanned == EOF ? 0 : 2;
}
