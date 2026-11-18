#define main prior_firmware_regression_main
#include "test_firmware.c"
#undef main
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usb_observer_tx.h"

USBD_HandleTypeDef hUsbDeviceFS;
static PCD_HandleTypeDef host_pcd;
static struct {
    uint32_t before[8];
    USBD_CDC_HandleTypeDef cdc;
    uint32_t after[8];
} allocation;
static struct {
    uint8_t *rx[16], *tx[16];
    uint32_t rx_size[16], tx_size[16], received[16];
    unsigned transmits[16], prepares[16], closes[16], stalls[2][16];
    uint8_t opened_in[16], opened_out[16], address;
    uint8_t control_copy[512];
    uint32_t control_length;
    USBD_StatusTypeDef transmit_result;
} port;
static unsigned groups, fuzz_cases;

void *USBD_static_malloc(uint32_t size)
{
    assert(size == sizeof(allocation.cdc));
    for (unsigned i = 0; i < 8; i++) {
        allocation.before[i] = 0x13579BDFU;
        allocation.after[i] = 0x2468ACE0U;
    }
    return &allocation.cdc;
}

void USBD_static_free(void *memory) { assert(memory == &allocation.cdc); }

static void check_guards(void)
{
    for (unsigned i = 0; i < 8; i++) {
        assert(allocation.before[i] == 0x13579BDFU);
        assert(allocation.after[i] == 0x2468ACE0U);
    }
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *device)
{
    device->pData = &host_pcd;
    return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *device) { (void)device; return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *device) { (void)device; return USBD_OK; }
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *device) { (void)device; return USBD_OK; }

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *device, uint8_t address, uint8_t type, uint16_t size)
{
    unsigned ep = address & 0xFU;
    (void)device; (void)type;
    if (address & 0x80U) {
        port.opened_in[ep] = 1U;
        host_pcd.IN_ep[ep].maxpacket = size;
    } else {
        port.opened_out[ep] = 1U;
    }
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *device, uint8_t address)
{
    unsigned ep = address & 0xFU;
    (void)device;
    port.closes[ep]++;
    if (address & 0x80U) {
        port.opened_in[ep] = 0U;
        port.tx[ep] = NULL;
        port.tx_size[ep] = 0;
    } else {
        port.opened_out[ep] = 0U;
        port.rx[ep] = NULL;
        port.rx_size[ep] = 0;
    }
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *device, uint8_t address)
{ (void)device; (void)address; return USBD_OK; }
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *device, uint8_t address)
{ (void)device; port.stalls[(address >> 7) & 1][address & 0xFU]++; return USBD_OK; }
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *device, uint8_t address)
{ (void)device; port.stalls[(address >> 7) & 1][address & 0xFU] = 0; return USBD_OK; }
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *device, uint8_t address)
{ (void)device; return port.stalls[(address >> 7) & 1][address & 0xFU] != 0; }
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *device, uint8_t address)
{ (void)device; port.address = address; return USBD_OK; }

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *device, uint8_t address, uint8_t *data, uint32_t length)
{
    unsigned ep = address & 0xFU;
    (void)device;
    port.tx[ep] = data;
    port.tx_size[ep] = length;
    port.transmits[ep]++;
    if (ep == 0) {
        assert(length <= sizeof(port.control_copy));
        if (length) { assert(data != NULL); memcpy(port.control_copy, data, length); }
        port.control_length = length;
    }
    return port.transmit_result;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *device, uint8_t address, uint8_t *data, uint32_t length)
{
    unsigned ep = address & 0xFU;
    (void)device;
    port.rx[ep] = data;
    port.rx_size[ep] = length;
    port.prepares[ep]++;
    return USBD_OK;
}
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *device, uint8_t address)
{ (void)device; return port.received[address & 0xFU]; }
void USBD_LL_Delay(uint32_t delay) { HAL_Delay(delay); }

static USBD_StatusTypeDef setup(uint8_t direction_type, uint8_t request, uint16_t value,
                               uint16_t index, uint16_t length)
{
    uint8_t bytes[8] = {direction_type, request, (uint8_t)value, (uint8_t)(value >> 8),
                        (uint8_t)index, (uint8_t)(index >> 8), (uint8_t)length, (uint8_t)(length >> 8)};
    return USBD_LL_SetupStage(&hUsbDeviceFS, bytes);
}

static void configure(void)
{
    assert(setup(0, USB_REQ_SET_ADDRESS, 5, 0, 0) == USBD_OK);
    assert(port.address == 5 && hUsbDeviceFS.dev_state == USBD_STATE_ADDRESSED);
    assert(setup(0, USB_REQ_SET_CONFIGURATION, 1, 0, 0) == USBD_OK);
    assert(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
    assert(hUsbDeviceFS.pClassData == &allocation.cdc);
    assert(hUsbDeviceFS.pClassDataCmsit[0] == &allocation.cdc);
    assert(port.opened_in[1] && port.opened_out[1] && port.opened_in[2]);
    assert(port.rx_size[1] == 64 && port.rx[1] != NULL);
    assert(!UsbObserver_Write("before DTR", 10));
    assert(setup(0x21, CDC_SET_CONTROL_LINE_STATE, 1, 0, 0) == USBD_OK);
}

static void start(void)
{
    memset(&hUsbDeviceFS, 0, sizeof(hUsbDeviceFS));
    memset(&host_pcd, 0, sizeof(host_pcd));
    memset(&port, 0, sizeof(port));
    port.transmit_result = USBD_OK;
    tick = test_primask = 0;
    UsbObserver_Init();
    assert(USBD_Init(&hUsbDeviceFS, NULL, DEVICE_FS) == USBD_OK);
    assert(USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) == USBD_OK);
    assert(USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) == USBD_OK);
    assert(USBD_Start(&hUsbDeviceFS) == USBD_OK);
    assert(USBD_LL_SetSpeed(&hUsbDeviceFS, USBD_SPEED_FULL) == USBD_OK);
    assert(USBD_LL_Reset(&hUsbDeviceFS) == USBD_OK);
    configure();
    check_guards();
}

static void test_descriptors_and_configuration(void)
{
    uint16_t length;
    unsigned endpoints = 0, interfaces = 0;
    uint8_t *data;
    start();
    data = USBD_CDC.GetFSConfigDescriptor(&length);
    assert(length == 67 && data[4] == 2);
    for (unsigned offset = 0; offset < length; offset += data[offset]) {
        assert(data[offset] >= 2 && offset + data[offset] <= length);
        if (data[offset + 1] == USB_DESC_TYPE_INTERFACE) { interfaces++; }
        if (data[offset + 1] == USB_DESC_TYPE_ENDPOINT) {
            endpoints++;
            if (data[offset + 2] == CDC_CMD_EP) { assert(data[offset + 4] == 8); }
            else { assert(data[offset + 4] == 64); }
        }
    }
    assert(endpoints == 3 && interfaces == 2);
    assert(setup(0x80, USB_REQ_GET_DESCRIPTOR, 0x0200, 0, 67) == USBD_OK);
    assert(port.control_length == 67 && port.control_copy[4] == 2);
    assert(setup(0, USB_REQ_SET_CONFIGURATION, 0, 0, 0) == USBD_OK);
    assert(hUsbDeviceFS.pClassData == NULL && !UsbObserver_Write("x", 1));
    assert(setup(0, USB_REQ_SET_CONFIGURATION, 1, 0, 0) == USBD_OK);
    assert(!UsbObserver_Write("x", 1));
    check_guards();
    groups++;
}

static void finish_bulk_in(void)
{
    assert(USBD_LL_DataInStage(&hUsbDeviceFS, 1, NULL) == USBD_OK);
}

static void test_bulk_lengths_zlp_and_ownership(void)
{
    const unsigned lengths[] = {1, 63, 64, 65, 128, 1088};
    uint8_t payload[USB_OBSERVER_FRAME_SIZE], copy[USB_OBSERVER_FRAME_SIZE];
    for (unsigned i = 0; i < sizeof(lengths) / sizeof(lengths[0]); i++) {
        UsbObserver_Stats stats;
        unsigned count, length = lengths[i];
        start();
        memset(payload, (int)(0x30 + i), sizeof(payload));
        memcpy(copy, payload, length);
        count = port.transmits[1];
        assert(UsbObserver_Write(payload, length));
        assert(port.transmits[1] == count + 1 && port.tx_size[1] == length);
        assert(allocation.cdc.TxState == 1);
        memset(payload, 0xEE, sizeof(payload));
        assert(memcmp(port.tx[1], copy, length) == 0);
        assert(!UsbObserver_Write("must not overwrite", 18));
        finish_bulk_in();
        if (length % 64 == 0) {
            assert(allocation.cdc.TxState == 1 && port.tx_size[1] == 0);
            assert(port.transmits[1] == count + 2);
            UsbObserver_Task();
            UsbObserver_GetStats(&stats);
            assert(stats.completed == 0);
            assert(!UsbObserver_Write("still owned", 11));
            assert(memcmp(allocation.cdc.TxBuffer, copy, length) == 0);
            finish_bulk_in();
        }
        assert(allocation.cdc.TxState == 0);
        UsbObserver_Task();
        UsbObserver_GetStats(&stats);
        assert(stats.submitted == 1 && stats.completed == 1);
        assert(UsbObserver_Write("next", 4));
        check_guards();
    }
    groups++;
}

static void test_stall_suspend_reset_disconnect(void)
{
    uint8_t saved[8];
    UsbObserver_Stats stats;
    start();
    tick = UINT32_MAX - 100;
    assert(UsbObserver_Write("pending", 7));
    memcpy(saved, allocation.cdc.TxBuffer, 7);
    assert(USBD_LL_Suspend(&hUsbDeviceFS) == USBD_OK);
    tick += 500;
    UsbObserver_Task();
    assert(!UsbObserver_Write("no", 2));
    assert(memcmp(allocation.cdc.TxBuffer, saved, 7) == 0);
    assert(USBD_LL_Resume(&hUsbDeviceFS) == USBD_OK);
    assert(!UsbObserver_Write("no", 2));
    assert(USBD_LL_Reset(&hUsbDeviceFS) == USBD_OK);
    assert(hUsbDeviceFS.pClassData == NULL && port.tx[1] == NULL);
    UsbObserver_Task();
    UsbObserver_GetStats(&stats);
    assert(stats.stalls == 1 && stats.completed == 0);
    configure();
    assert(UsbObserver_Write("new", 3));
    assert(USBD_LL_DevDisconnected(&hUsbDeviceFS) == USBD_OK);
    assert(hUsbDeviceFS.pClassData == NULL && !port.opened_in[1]);
    UsbObserver_Task();
    assert(!UsbObserver_Write("closed", 6));
    assert(USBD_LL_Reset(&hUsbDeviceFS) == USBD_OK);
    configure();
    assert(UsbObserver_Write("restored", 8));
    check_guards();
    groups++;
}

static void receive_control_data(void)
{
    uint32_t length = port.rx_size[0];
    assert(length <= 64 && port.rx[0] != NULL);
    memset(port.rx[0], 0xA5, length);
    port.received[0] = length;
    assert(USBD_LL_DataOutStage(&hUsbDeviceFS, 0, port.rx[0]) == USBD_OK);
}

static void test_control_boundaries_and_discarded_commands(void)
{
    const uint16_t lengths[] = {0, 1, 6, 7, 8, 63, 64, 65, 255, 256, UINT16_MAX};
    uint8_t commands[] = {0xAA, 0x55, 0, 4, 0xFB, '$', 'A', '1', '2', '0', '#'};
    BabyCare_StatusTypeDef before, after;
    UsbObserver_Stats stats;
    unsigned prepared;
    start();
    assert(setup(0x21, CDC_SET_LINE_CODING, 0, 0, 7) == USBD_OK);
    receive_control_data();
    assert(setup(0xA1, CDC_GET_LINE_CODING, 0, 0, UINT16_MAX) == USBD_OK);
    assert(port.control_length == 7 && port.control_copy[0] == 0xA5);
    for (unsigned request = 0; request < 256; request++) {
        for (unsigned i = 0; i < sizeof(lengths) / sizeof(lengths[0]); i++) {
            for (unsigned input = 0; input < 2; input++) {
                (void)setup(input ? 0xA1 : 0x21, (uint8_t)request, 1, 0, lengths[i]);
                if (!input && lengths[i]) { receive_control_data(); }
                if (input && lengths[i]) { assert(port.control_length <= 7); }
                check_guards();
                fuzz_cases++;
            }
        }
    }
    tick = 0;
    BABY_CARE_INTERLOCK_PORT->ODR = BABY_CARE_INTERLOCK_STOP_PIN | BABY_CARE_INTERLOCK_POWER_PIN;
    BabyCare_Init(); BabyCare_Task();
    tick = BABY_CARE_INTERLOCK_STABLE_MS;
    BabyCare_Task();
    BABY_CARE_INTERLOCK_PORT->ODR |= BABY_CARE_INTERLOCK_ARMED_PIN;
    BabyCare_Task(); tick += BABY_CARE_INTERLOCK_STABLE_MS; BabyCare_Task();
    before = *BabyCare_GetStatus();
    assert(before.interlock_ready && before.interlock_armed);
    assert(htim2.enabled == 0 && htim3.enabled == 0);
    prepared = port.prepares[1];
    assert(port.rx[1] != NULL && port.rx_size[1] >= sizeof(commands));
    memcpy(port.rx[1], commands, sizeof(commands));
    port.received[1] = sizeof(commands);
    assert(USBD_LL_DataOutStage(&hUsbDeviceFS, 1, port.rx[1]) == USBD_OK);
    BabyCare_Task();
    after = *BabyCare_GetStatus();
    assert(after.mode == before.mode && after.latest_voice_command == before.latest_voice_command);
    assert(!after.interlock_enabled && htim2.enabled == 0 && htim3.enabled == 0);
    assert(port.prepares[1] == prepared + 1);
    UsbObserver_GetStats(&stats);
    assert(stats.discarded_rx_bytes == sizeof(commands));
    check_guards();
    groups++;
}

static void test_underlying_transmit_failure_is_not_completion(void)
{
    uint8_t saved[6];
    UsbObserver_Stats stats;
    start();
    port.transmit_result = USBD_FAIL;
    /* ST CDC ignores LL_Transmit's return: OK here means submission, not delivery. */
    assert(UsbObserver_Write("failed", 6));
    memcpy(saved, allocation.cdc.TxBuffer, sizeof(saved));
    tick = USB_OBSERVER_STALL_MS;
    UsbObserver_Task();
    assert(!UsbObserver_Write("overwrite", 9));
    assert(memcmp(saved, allocation.cdc.TxBuffer, sizeof(saved)) == 0);
    UsbObserver_GetStats(&stats);
    assert(stats.submitted == 1 && stats.completed == 0 && stats.stalls == 1);
    port.transmit_result = USBD_OK;
    assert(USBD_LL_Reset(&hUsbDeviceFS) == USBD_OK);
    UsbObserver_Task();
    configure();
    assert(UsbObserver_Write("recover", 7));
    finish_bulk_in();
    UsbObserver_Task();
    UsbObserver_GetStats(&stats);
    assert(stats.completed == 1);
    check_guards();
    groups++;
}

#ifndef USB_STACK_TEST_MAIN
#define USB_STACK_TEST_MAIN main
#endif
int USB_STACK_TEST_MAIN(void)
{
    test_descriptors_and_configuration();
    test_bulk_lengths_zlp_and_ownership();
    test_stall_suspend_reset_disconnect();
    test_control_boundaries_and_discarded_commands();
    test_underlying_transmit_failure_is_not_completion();
    start();
    assert(prior_firmware_regression_main(0, NULL) == 0);
    check_guards();
    printf("Actual ST USB core/CDC: %u groups, %u control cases, prior application regression passed; simulated LL port only.\n", groups, fuzz_cases);
    return 0;
}
