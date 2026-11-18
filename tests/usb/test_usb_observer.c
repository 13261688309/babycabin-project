#define main prior_firmware_regression_main
#include "test_firmware.c"
#undef main
#include "usb_observer_tx.h"
#include "usbd_cdc_if.h"

USBD_HandleTypeDef hUsbDeviceFS;
static USBD_CDC_HandleTypeDef usb_class;
static uint8_t usb_set_result, usb_tx_result;
static uint8_t fail_but_reserve;
static unsigned usb_submits, usb_receives;
static unsigned usb_groups;

uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *device, uint8_t *data, uint32_t length)
{
    USBD_CDC_HandleTypeDef *cdc = device->pClassData;
    if (usb_set_result != USBD_OK || cdc == NULL) { return USBD_FAIL; }
    cdc->TxBuffer = data;
    cdc->TxLength = length;
    return USBD_OK;
}

uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *device)
{
    USBD_CDC_HandleTypeDef *cdc = device->pClassData;
    assert(cdc != NULL);
    usb_submits++;
    if (usb_tx_result == USBD_OK || fail_but_reserve) { cdc->TxState = 1U; }
    return usb_tx_result;
}

uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *device, uint8_t *data)
{
    (void)device; (void)data;
    return USBD_OK;
}

uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *device)
{
    (void)device;
    usb_receives++;
    return USBD_OK;
}

static void reset_usb(void)
{
    memset(&usb_class, 0, sizeof(usb_class));
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceFS.pClassData = &usb_class;
    usb_set_result = usb_tx_result = USBD_OK;
    fail_but_reserve = 0U;
    usb_submits = usb_receives = 0U;
    test_primask = tick = 0U;
    UsbObserver_Init();
    UsbObserver_OnSessionReset();
}

static void test_readiness_and_bounds(void)
{
    char data[USB_OBSERVER_FRAME_SIZE + 1U];
    reset_usb();
    assert(!UsbObserver_Write("x", 1));
    UsbObserver_OnControlLines(1);
    hUsbDeviceFS.dev_state = 0;
    assert(!UsbObserver_Write("x", 1));
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceFS.pClassData = NULL;
    assert(!UsbObserver_Write("x", 1));
    assert(CDC_Transmit_FS((uint8_t *)data, 1) == USBD_FAIL);
    hUsbDeviceFS.pClassData = &usb_class;
    assert(!UsbObserver_Write(NULL, 1));
    assert(!UsbObserver_Write(data, 0));
    assert(!UsbObserver_Write(data, sizeof(data)));
    memset(data, 'Q', sizeof(data));
    assert(UsbObserver_Write(data, USB_OBSERVER_FRAME_SIZE));
    assert(usb_class.TxLength == USB_OBSERVER_FRAME_SIZE);
    assert(memcmp(usb_class.TxBuffer, data, USB_OBSERVER_FRAME_SIZE) == 0);
    usb_groups++;
}

static void test_immutable_buffer_and_wrap_stall(void)
{
    char data[] = "original\r\n";
    uint8_t copy[sizeof(data)];
    UsbObserver_Stats stats;
    reset_usb();
    UsbObserver_OnControlLines(1);
    tick = UINT32_MAX - 200U;
    assert(UsbObserver_Write(data, sizeof(data)));
    memcpy(copy, usb_class.TxBuffer, sizeof(copy));
    memset(data, 'X', sizeof(data));
    assert(memcmp(copy, usb_class.TxBuffer, sizeof(copy)) == 0);
    tick += USB_OBSERVER_STALL_MS;
    UsbObserver_Task();
    UsbObserver_Task();
    assert(!UsbObserver_Write("replacement", 11));
    assert(memcmp(copy, usb_class.TxBuffer, sizeof(copy)) == 0);
    UsbObserver_GetStats(&stats);
    assert(stats.submitted == 1 && stats.stalls == 1 && stats.busy == 1);
    usb_class.TxState = 0;
    UsbObserver_Task();
    UsbObserver_GetStats(&stats);
    assert(stats.completed == 1);
    assert(UsbObserver_Write("fresh", 5));
    assert(memcmp(usb_class.TxBuffer, "fresh", 5) == 0);
    usb_groups++;
}

static void test_session_changes_and_critical_state(void)
{
    UsbObserver_Stats stats;
    reset_usb();
    UsbObserver_OnControlLines(1);
    test_primask = 1;
    assert(UsbObserver_Write("first", 5));
    assert(test_primask == 1);
    UsbObserver_OnSessionReset();
    UsbObserver_Task();
    UsbObserver_GetStats(&stats);
    assert(test_primask == 1 && stats.dropped == 1);
    assert(!UsbObserver_Write("stale", 5));
    usb_class.TxState = 0;
    UsbObserver_OnControlLines(1);
    assert(UsbObserver_Write("next", 4));
    assert(test_primask == 1);
    UsbObserver_OnControlLines(0);
    assert(!UsbObserver_Write("closed", 6));
    assert(memcmp(usb_class.TxBuffer, "next", 4) == 0);
    test_primask = 0;
    usb_groups++;
}

static void test_submit_failures_keep_owned_buffer(void)
{
    UsbObserver_Stats stats;
    reset_usb();
    UsbObserver_OnControlLines(1);
    usb_set_result = USBD_FAIL;
    assert(!UsbObserver_Write("first", 5));
    assert(usb_submits == 0);
    usb_set_result = USBD_OK;
    usb_tx_result = USBD_FAIL;
    fail_but_reserve = 1;
    assert(!UsbObserver_Write("reserved", 8));
    assert(!UsbObserver_Write("overwrite", 9));
    assert(memcmp(usb_class.TxBuffer, "reserved", 8) == 0);
    UsbObserver_GetStats(&stats);
    assert(stats.errors == 2 && stats.busy == 1);
    usb_groups++;
}

static void test_control_requests_and_discarded_out(void)
{
    USBD_SetupReqTypedef request = {0};
    uint8_t coding[14], receive[] = {0xAA, 0x55, 0, 4, 0xFB};
    uint32_t length = sizeof(receive);
    UsbObserver_Stats stats;
    reset_usb();
    assert(USBD_Interface_fops_FS.Init() == USBD_OK);
    request.wValue = 1;
    assert(USBD_Interface_fops_FS.Control(CDC_SET_CONTROL_LINE_STATE, (uint8_t *)&request, 0) == USBD_OK);
    assert(UsbObserver_Write("ready", 5));
    memset(coding, 0xA5, sizeof(coding));
    assert(USBD_Interface_fops_FS.Control(CDC_GET_LINE_CODING, coding, UINT16_MAX) == USBD_OK);
    for (unsigned i = 7; i < sizeof(coding); i++) { assert(coding[i] == 0xA5); }
    coding[0] = 0x80; coding[1] = 0x25;
    assert(USBD_Interface_fops_FS.Control(CDC_SET_LINE_CODING, coding, 7) == USBD_OK);
    memset(coding, 0, 7);
    assert(USBD_Interface_fops_FS.Control(CDC_GET_LINE_CODING, coding, 7) == USBD_OK);
    assert(coding[0] == 0x80 && coding[1] == 0x25);
    assert(USBD_Interface_fops_FS.Control(CDC_SET_LINE_CODING, coding, 1) == USBD_FAIL);
    assert(USBD_Interface_fops_FS.Control(CDC_SET_CONTROL_LINE_STATE, NULL, 0) == USBD_FAIL);
    assert(USBD_Interface_fops_FS.Receive(receive, &length) == USBD_OK);
    assert(usb_receives == 1 && receive[3] == 4);
    UsbObserver_GetStats(&stats);
    assert(stats.discarded_rx_bytes == sizeof(receive));
    assert(USBD_Interface_fops_FS.Receive(NULL, &length) == USBD_FAIL);
    assert(USBD_Interface_fops_FS.DeInit() == USBD_OK);
    assert(!UsbObserver_Write("closed", 6));
    usb_groups++;
}

int main(void)
{
    test_readiness_and_bounds();
    test_immutable_buffer_and_wrap_stall();
    test_session_changes_and_critical_state();
    test_submit_failures_keep_owned_buffer();
    test_control_requests_and_discarded_out();
    reset_usb();
    UsbObserver_OnControlLines(1);
    if (prior_firmware_regression_main(0, NULL) != 0) { return 1; }
    assert(usb_submits > 0);
    printf("USB observer: %u group checks plus existing production-application regression passed; host simulation only.\n", usb_groups);
    return 0;
}
