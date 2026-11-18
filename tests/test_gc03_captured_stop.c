#include <test_gimbal_control.c>

int main(void)
{
    unsigned bytes[5];
    uint8_t queued[32];
    unsigned i;
    uint32_t applied, epoch;

    for (i = 0; i < 5; ++i) {
        G(scanf("%x", &bytes[i]) == 1);
    }
    G(bytes[0] == 0xAA && bytes[1] == 0x55 && bytes[2] == 0 &&
      bytes[3] == VOICE_CMD_STOP_2 && bytes[4] == 0xFB);
    independent_regression_main();
    start_test(1000U);
    applied = BabyCare_GetStatus()->voice_stop_applied;
    epoch = GimbalControl_Epoch();
    packet(queued, 2, 100, 85, 0, 2);
    GimbalControl_OnBytes(queued, sizeof queued);
    for (i = 0; i < 5; ++i) {
        G(huart2.rx != NULL);
        *huart2.rx = (uint8_t)bytes[i];
        BabyCare_OnUartRxComplete(&huart2);
    }
    BabyCare_Task();
    G(stopped() && GimbalControl_Lease() == 0);
    G(BabyCare_GetStatus()->voice_stop_applied == applied + 1);
    G(GimbalControl_Epoch() != epoch);
    send(queued);
    G(stopped() && BabyCare_GetStatus()->voice_stop_applied == applied + 1);
    printf("PASS captured STOP 02 application and stale motion barrier\n");
    return 0;
}
