/* Reuse the actual ST core/CDC harness, retaining its standalone legacy entry. */
#define USB_STACK_TEST_MAIN legacy_usb_stack_checks
#define bsp_send_packet capture_only_bsp_send_packet
#define bsp_send_alert_packet capture_only_bsp_send_alert_packet
#include "test_usb_stack.c"
#undef bsp_send_packet
#undef bsp_send_alert_packet
#include "drivers/comm_bridge.h"

/* Use production framing here instead of the base HAL model's capture-only BSP. */
static void queue_protocol(const char *json, bool alert)
{
    char line[DEBUG_TX_FRAME_SIZE];
    size_t length = alert ? comm_bridge_build_alert_line(json, line, sizeof(line))
                          : comm_bridge_build_report_line(json, line, sizeof(line));
    assert(length && length + 2U < sizeof(line));
    memcpy(line + length, "\r\n", 2U);
    (void)DebugTx_Write(line, length + 2U, alert);
}
void bsp_send_packet(const char *json) { queue_protocol(json, false); }
void bsp_send_alert_packet(const char *json) { queue_protocol(json, true); }

static char delivered[128][USB_OBSERVER_FRAME_SIZE + 1U];
static unsigned delivered_count;

static void remember_active(void)
{
    const uint32_t length = allocation.cdc.TxLength;
    assert(length <= USB_OBSERVER_FRAME_SIZE && delivered_count < 128U);
    memcpy(delivered[delivered_count], allocation.cdc.TxBuffer, length);
    delivered[delivered_count][length] = '\0';
    delivered_count++;
}

static void complete_active(void)
{
    unsigned count = 0U;
    while (allocation.cdc.TxState) {
        assert(++count <= 2U);
        finish_bulk_in();
    }
}

static void drain_usb(void)
{
    for (unsigned i = 0; i < 32U; ++i) {
        UsbObserver_Task();
        if (!allocation.cdc.TxState) { return; }
        remember_active();
        complete_active();
    }
    assert(!"USB queue did not drain within its bounded capacity");
}

static unsigned count_prefix(const char *prefix)
{
    unsigned found = 0U;
    for (unsigned i = 0; i < delivered_count; ++i) {
        if (!strncmp(delivered[i], prefix, strlen(prefix))) { found++; }
    }
    return found;
}

static int test_information_burst(void)
{
    start();
    delivered_count = 0U;
    assert(DebugTx_Init(&huart1));
    const char *lines[] = {"vision_link v=1\r\n", "POST /api/device/report {}\r\n",
        "status mode=fault\r\n", "voice_rx v=2\r\n", "POST /api/device/alert {}\r\n"};
    for (unsigned i = 0U; i < 5U; ++i) { assert(DebugTx_Write(lines[i], strlen(lines[i]), i == 4U)); }
    drain_usb();
    if (count_prefix("vision_link ") != 1U || count_prefix("POST /api/device/report ") != 1U ||
        count_prefix("status ") != 1U || count_prefix("voice_rx ") != 1U ||
        count_prefix("POST /api/device/alert ") != 1U) {
        printf("FAIL USB burst: %u/5 log topics delivered; vision/status/report/voice/alert must coexist.\n", delivered_count);
        return 1;
    }
    assert(!strncmp(delivered[1], "POST /api/device/alert ", 23U));
    check_guards();
    puts("PASS USB mux burst: all five topics delivered; alert first after the active frame; actual ST CDC, simulated LL.");
    return 0;
}

#ifdef USB_OBSERVER_LOG_MUX
static void test_coalescing_and_reserved_capacity(void)
{
    const char *topics[] = {"status old\r\n", "POST /api/device/report old\r\n", "voice_rx old\r\n", "vision_link old\r\n"};
    const char *fresh[] = {"status new\r\n", "POST /api/device/report new\r\n", "voice_rx new\r\n", "vision_link new\r\n"};
    UsbObserver_Stats stats;
    uint8_t saved[1088];
    start(); delivered_count = 0U;
    memset(saved, 0x5A, sizeof(saved));
    assert(UsbObserver_Write(saved, sizeof(saved)));
    for (unsigned i = 0; i < 4U; ++i) { assert(UsbObserver_Write(topics[i], strlen(topics[i]))); }
    assert(UsbObserver_Write("general1", 8));
    assert(UsbObserver_Write("general2", 8));
    assert(!UsbObserver_Write("general3", 8));
    assert(UsbObserver_WritePriority("alert1", 6, true));
    assert(UsbObserver_WritePriority("alert2", 6, true));
    assert(!UsbObserver_WritePriority("alert3", 6, true));
    for (unsigned i = 0; i < 4U; ++i) { assert(UsbObserver_Write(fresh[i], strlen(fresh[i]))); }
    assert(!memcmp(allocation.cdc.TxBuffer, saved, sizeof(saved)));
    UsbObserver_GetStats(&stats);
    assert(stats.queued == 8U && stats.coalesced == 4U && stats.queue_drops == 2U);
    drain_usb();
    assert(delivered_count == 9U);
    for (unsigned i = 0; i < delivered_count; ++i) { assert(strstr(delivered[i], "old") == NULL); }
    for (unsigned i = 0; i < 4U; ++i) { assert(count_prefix(fresh[i]) == 1U); }
    UsbObserver_GetStats(&stats); assert(stats.queued == 0U);
    check_guards();
}

static void test_alert_fairness_and_uart_failure_independence(void)
{
    start(); delivered_count = 0U;
    assert(UsbObserver_Write("held", 4));
    assert(UsbObserver_Write("status waiting", 14));
    assert(UsbObserver_WritePriority("urgent1", 7, true));
    assert(UsbObserver_WritePriority("urgent2", 7, true));
    for (unsigned i = 0; i < 4U; ++i) {
        remember_active(); complete_active(); UsbObserver_Task();
        if (i < 3U) { (void)UsbObserver_WritePriority("urgent_more", 11, true); }
    }
    assert(!strncmp(delivered[3], "status ", 7));
    start(); delivered_count = 0U;
    assert(DebugTx_Init(&huart1)); tx_result = HAL_BUSY;
    for (unsigned i = 0; i < 8U; ++i) {
        assert(UsbObserver_Write("vision_link latest", 18));
        (void)DebugTx_Write("status alive", 12, false);
        DebugTx_Task();
        drain_usb();
        tick += 20U;
    }
    assert(count_prefix("status ") == 8U);
    tx_result = HAL_OK;
    check_guards();
}

static void test_stall_session_and_buffer_ownership(void)
{
    uint8_t saved[64];
    UsbObserver_Stats stats;
    start(); delivered_count = 0U;
    memset(saved, 0x55, sizeof(saved));
    assert(UsbObserver_Write(saved, sizeof(saved)));
    assert(UsbObserver_Write("status stale", 12));
    tick = 501U; UsbObserver_Task();
    assert(UsbObserver_Write("status fresh", 12));
    assert(!memcmp(allocation.cdc.TxBuffer, saved, sizeof(saved)));
    UsbObserver_GetStats(&stats); assert(stats.stalls == 1U);
    UsbObserver_OnControlLines(0U); UsbObserver_OnControlLines(1U);
    UsbObserver_Task();
    assert(!memcmp(allocation.cdc.TxBuffer, saved, sizeof(saved)));
    assert(UsbObserver_Write("new_session", 11));
    complete_active(); drain_usb();
    assert(delivered_count == 1U && !strcmp(delivered[0], "new_session"));
    assert(UsbObserver_Write("pending_active", 14));
    assert(UsbObserver_Write("status old_session", 18));
    assert(USBD_LL_Reset(&hUsbDeviceFS) == USBD_OK);
    UsbObserver_Task(); configure();
    delivered_count = 0U;
    assert(UsbObserver_Write("after_reset", 11)); drain_usb();
    assert(delivered_count == 1U && !strcmp(delivered[0], "after_reset"));
    check_guards();
}

static void test_actual_application_topics(void)
{
    start(); delivered_count = 0U;
    BABY_CARE_INTERLOCK_PORT->ODR = 0U;
    tx_result = tx_abort_result = rx_result = rx_abort_result = HAL_OK;
    BabyCare_Init();
    drain_usb(); delivered_count = 0U;
    for (uint32_t now = 0U; now <= 6000U; now += 20U) {
        tick = now;
        if (tx_data != NULL) { tx_complete(); }
        BabyCare_Task();
        if (allocation.cdc.TxState) { remember_active(); complete_active(); }
        UsbObserver_Task();
    }
    drain_usb();
    assert(count_prefix("vision_link ") >= 8U);
    assert(count_prefix("POST /api/device/report ") >= 3U);
    assert(count_prefix("status ") >= 2U && count_prefix("voice_rx ") >= 2U);
    assert(count_prefix("POST /api/device/alert ") >= 1U);
    for (unsigned i = 0U; i < delivered_count; ++i) {
        const char *line = delivered[i];
        if (!strncmp(line, "vision_link ", 12) || !strncmp(line, "POST /api/device/", 17) ||
            !strncmp(line, "status ", 7) || !strncmp(line, "voice_rx ", 9)) {
            printf("WIRE %s", line);
        }
    }
    assert(htim2.enabled == 0U && htim3.enabled == 0U);
    assert((BABY_CARE_INTERLOCK_PORT->ODR & BABY_CARE_INTERLOCK_ENABLE_PIN) == 0U);
    check_guards();
    puts("PASS actual app USB stream: vision/report/status/voice/alert all delivered under endpoint delay; outputs unchanged in HAL model.");
}
#endif

int main(void)
{
    if (test_information_burst()) { return 1; }
#ifdef USB_OBSERVER_LOG_MUX
    test_coalescing_and_reserved_capacity();
    test_alert_fairness_and_uart_failure_independence();
    test_stall_session_and_buffer_ownership();
    test_actual_application_topics();
    puts("PASS USB mux: bounded topic/general/alert queues, coalescing, fairness, UART isolation, ZLP ownership, stall and session purge; actual ST core/CDC, simulated LL.");
#endif
    return 0;
}
