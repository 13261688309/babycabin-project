#ifndef VENDOR_UART_STUB_H
#define VENDOR_UART_STUB_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>

typedef void *QueueHandle_t;
typedef struct {
    int baud_rate;
    int data_bits;
    int parity;
    int stop_bits;
    int flow_ctrl;
} uart_config_t;

#define UART_NUM_0 0
#define UART_NUM_1 1
#define GPIO_NUM_35 35
#define GPIO_NUM_36 36
#define GPIO_NUM_43 43
#define GPIO_NUM_44 44
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_PIN_NO_CHANGE (-1)
#define configMAX_PRIORITIES 25
#define portMAX_DELAY 0xffffffffU
#define pdMS_TO_TICKS(value) (value)

int uart_read_bytes(int port, void *buffer, size_t length, unsigned int timeout);
int uart_write_bytes(int port, const void *buffer, size_t length);
int uart_flush(int port);
int uart_driver_install(int port, int rx, int tx, int queue_size, void *queue, int flags);
int uart_param_config(int port, const uart_config_t *config);
int uart_set_pin(int port, int tx, int rx, int rts, int cts);
void vTaskDelay(unsigned int ticks);
void vTaskDelete(void *task);
int xTaskCreate(void (*task)(void *), const char *name, unsigned int stack, void *arg,
                unsigned int priority, void *handle);
int xQueueSend(QueueHandle_t queue, const void *item, unsigned int timeout);

#endif
