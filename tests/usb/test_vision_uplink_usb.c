#define main prior_firmware_regression_main
#include "test_firmware.c"
#undef main
#include "usb_observer_tx.h"
#include "usbd_cdc_if.h"
#include "vision_uplink.h"

USBD_HandleTypeDef hUsbDeviceFS;
static USBD_CDC_HandleTypeDef usb_class;
static unsigned receives;

uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *d, uint8_t *data, uint32_t length)
{
    USBD_CDC_HandleTypeDef *cdc = d->pClassData;
    cdc->TxBuffer = data; cdc->TxLength = length; return USBD_OK;
}
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *d)
{
    ((USBD_CDC_HandleTypeDef *)d->pClassData)->TxState = 1U; return USBD_OK;
}
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *d, uint8_t *data)
{
    (void)d; (void)data; return USBD_OK;
}
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *d)
{
    (void)d; receives++; return USBD_OK;
}

int main(int argc, char **argv)
{
    FILE *input;
    uint8_t packet[64];
    uint32_t length;
    VisionUplink_Snapshot state;
    USBD_SetupReqTypedef request = {0};
    assert(argc == 2);
    input = fopen(argv[1], "rb");
    assert(input && fread(packet, 1, 64, input) == 64); fclose(input);
    tick = 1000U; test_primask = 0U;
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceFS.pClassData = &usb_class;
    UsbObserver_Init(); VisionUplink_Init();
    assert(USBD_Interface_fops_FS.Init() == USBD_OK);
    VisionUplink_Task(); VisionUplink_AdvertiseTick();
    length = 17U;
    assert(USBD_Interface_fops_FS.Receive(packet, &length) == USBD_OK);
    VisionUplink_Task(); VisionUplink_GetSnapshot(&state); assert(!state.fresh);
    length = 47U;
    assert(USBD_Interface_fops_FS.Receive(packet + 17U, &length) == USBD_OK);
    VisionUplink_Task(); VisionUplink_GetSnapshot(&state);
    assert(state.fresh && state.accepted == 1U && receives == 2U);
    request.wValue = 0U;
    assert(USBD_Interface_fops_FS.Control(CDC_SET_CONTROL_LINE_STATE, (uint8_t *)&request, 0) == USBD_OK);
    VisionUplink_GetSnapshot(&state); assert(!state.fresh && state.host == 0U);
    VisionUplink_Task(); VisionUplink_AdvertiseTick();
    length = 64U; assert(USBD_Interface_fops_FS.Receive(packet, &length) == USBD_OK);
    VisionUplink_Task(); VisionUplink_GetSnapshot(&state); assert(state.fresh);
    assert(USBD_Interface_fops_FS.DeInit() == USBD_OK);
    VisionUplink_GetSnapshot(&state); assert(!state.fresh);
    puts("PASS vision USB callbacks: fragmented OUT reaches bounded receiver, DTR/session/deinit invalidate immediately; USB LL mocked, no hardware.");
    return 0;
}
