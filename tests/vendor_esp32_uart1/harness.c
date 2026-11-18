#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "my_usart.h"
#include "my_usart1_user.h"

uint8_t sta_ip_connect[4] = {192, 0, 2, 42};
uint16_t wifi_Mode = 2;
bool register_mode = false;
static unsigned char reply[128];
static size_t reply_length;
static int reply_port = -1;
static int configured_port = -1;
static int configured_baud;
static int configured_tx;
static int configured_rx;

int uart_write_bytes(int port, const void *data, size_t length)
{
    assert(length < sizeof(reply));
    assert(reply_length == 0);
    memcpy(reply, data, length);
    reply_length = length;
    reply_port = port;
    return (int)length;
}

/* Same forwarding contract as the inspected vendor my_usart.c UART0 wrapper. */
int Uart_Send_Data(uint8_t *data, uint16_t len)
{
    return uart_write_bytes(UART_MUN, data, len);
}

int uart_read_bytes(int port, void *data, size_t length, unsigned int timeout) { return 0; }
int uart_flush(int port) { return 0; }
int uart_driver_install(int port, int rx, int tx, int qs, void *q, int flags) { return 0; }
int uart_param_config(int port, const uart_config_t *config)
{
    configured_port = port;
    configured_baud = config->baud_rate;
    return 0;
}
int uart_set_pin(int port, int tx, int rx, int rts, int cts)
{
    configured_tx = tx;
    configured_rx = rx;
    return 0;
}
void vTaskDelay(unsigned int ticks) {}
void vTaskDelete(void *task) {}
int xTaskCreate(void (*task)(void *), const char *name, unsigned int stack, void *arg,
                unsigned int priority, void *handle) { return 1; }
int xQueueSend(QueueHandle_t queue, const void *item, unsigned int timeout) { return 1; }

static void run_case(const char *name, const char *command, const char *expected, int port)
{
    reply_length = 0;
    reply_port = -1;
    memset(reply, 0, sizeof(reply));
    Deal_uart1_massage((uint8_t *)command, (uint16_t)strlen(command));
    assert(reply_port == port);
    assert(reply_length == strlen(expected));
    assert(memcmp(reply, expected, reply_length) == 0);
    printf("{\"case\":\"%s\",\"requestUart\":1,\"replyUart\":%d,\"sameUart\":%s,\"replyBytes\":%u}\n",
           name, reply_port, reply_port == 1 ? "true" : "false", (unsigned)reply_length);
}

int main(void)
{
    My_Uart1_user_Init(NULL);
    assert(configured_port == 1 && configured_baud == 115200);
    assert(configured_tx == 36 && configured_rx == 35);
    run_case("ap_available", "ap_ip", "ap_ip:192.168.4.1\r\n", 0);
    wifi_Mode = 1;
    run_case("ap_disabled", "AP_IP", "ap_ip:null\r\n", 0);
    run_case("sta_available", "sta_ip", "sta_ip:192.0.2.42\r\n", 0);
    sta_ip_connect[0] = 0;
    run_case("sta_unavailable", "STA_IP", "sta_ip:null\r\n", 0);
    run_case("unsupported_factory_command", "wifi_mode:2", "fail,command no found!\r\n", 1);
    run_case("virtual_key", "KEY_MENU", "KEY_MENU OK\r\n", 1);
    return 0;
}
