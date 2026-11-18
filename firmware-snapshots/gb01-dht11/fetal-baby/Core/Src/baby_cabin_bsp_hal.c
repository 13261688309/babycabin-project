#include "bsp/bsp.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "baby_care_config.h"
#include "drivers/comm_bridge.h"
#include "main.h"
#include "inc/app_config.h"
#include "baby_care_hal.h"
#include "debug_uart_tx.h"

#define BABY_CABIN_UART_TIMEOUT_MS 100U
#define BABY_CABIN_LINE_BUFFER_SIZE PROTOCOL_LINE_BUFFER_SIZE
#if DEBUG_TX_FRAME_SIZE < PROTOCOL_LINE_BUFFER_SIZE
#error Debug UART frame storage is smaller than the protocol envelope
#endif

static char s_line_buffer[BABY_CABIN_LINE_BUFFER_SIZE];

static void bridge_dht11_config_pin(uint32_t mode, uint32_t pull)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = BABY_CARE_DHT11_PIN;
    gpio.Mode = mode;
    gpio.Pull = pull;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BABY_CARE_DHT11_PORT, &gpio);
}

bool bsp_hal_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return true;
}

uint32_t bsp_hal_millis(void)
{
    return HAL_GetTick();
}

void bsp_hal_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void bsp_hal_delay_us(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t cycles = (SystemCoreClock / 1000000U) * us;

    while ((DWT->CYCCNT - start) < cycles)
    {
    }
}

static bool bridge_queue_packet(const char *packet, size_t len, bool alert)
{
    size_t line_len;

    if (packet == NULL || len == 0U)
    {
        return false;
    }

    line_len = alert ? comm_bridge_build_alert_line(packet, s_line_buffer, sizeof(s_line_buffer))
                     : comm_bridge_build_report_line(packet, s_line_buffer, sizeof(s_line_buffer));
    if (line_len == 0U || line_len + 3U > sizeof(s_line_buffer))
    {
        return false;
    }

    memcpy(s_line_buffer + line_len, "\r\n", 3U);
    return DebugTx_Write(s_line_buffer, line_len + 2U, alert);
}

bool bsp_hal_send_packet(const char *packet, size_t len)
{
    return bridge_queue_packet(packet, len, false);
}

bool bsp_hal_send_alert_packet(const char *packet, size_t len)
{
    return bridge_queue_packet(packet, len, true);
}

bool bsp_hal_poll_command_packet(char *buffer, size_t buffer_size)
{
    if (buffer != NULL && buffer_size > 0U)
    {
        buffer[0] = '\0';
    }

    return false;
}

void bsp_hal_log(const char *message)
{
    const size_t len = (message == NULL) ? 0U : strlen(message);

    if (len == 0U)
    {
        return;
    }

    if (len + 2U > sizeof(s_line_buffer)) { return; }
    memcpy(s_line_buffer, message, len);
    memcpy(s_line_buffer + len, "\r\n", 2U);
    (void)DebugTx_Write(s_line_buffer, len + 2U, false);
}

bool bsp_hal_adc_read(bsp_adc_channel_t channel, uint16_t *raw)
{
    (void)channel;

    if (raw != NULL)
    {
        *raw = 0U;
    }

    return false;
}

bool bsp_hal_gpio_read(bsp_gpio_input_t input, bool *active)
{
    (void)input;

    if (active != NULL)
    {
        *active = false;
    }

    return false;
}

bool bsp_hal_i2c_mem_read(bsp_i2c_device_t device, uint8_t reg, uint8_t *data, size_t len)
{
    (void)device;
    (void)reg;
    (void)data;
    (void)len;
    return false;
}

bool bsp_hal_i2c_mem_write(bsp_i2c_device_t device, uint8_t reg, const uint8_t *data, size_t len)
{
    (void)device;
    (void)reg;
    (void)data;
    (void)len;
    return false;
}

void bsp_hal_dht11_drive_low(void)
{
    bridge_dht11_config_pin(GPIO_MODE_OUTPUT_OD, GPIO_PULLUP);
    HAL_GPIO_WritePin(BABY_CARE_DHT11_PORT, BABY_CARE_DHT11_PIN, GPIO_PIN_RESET);
}

void bsp_hal_dht11_release(void)
{
    bridge_dht11_config_pin(GPIO_MODE_INPUT, GPIO_PULLUP);
}

bool bsp_hal_dht11_read(bool *level)
{
    if (level == NULL)
    {
        return false;
    }

    *level = HAL_GPIO_ReadPin(BABY_CARE_DHT11_PORT, BABY_CARE_DHT11_PIN) == GPIO_PIN_SET;
    return true;
}

bool bsp_hal_voice_send(const uint8_t *data, size_t len)
{
#if BABY_CARE_USE_VOICE_UART
    if (data == NULL || len == 0U)
    {
        return false;
    }

    return HAL_UART_Transmit(&BABY_CARE_VOICE_UART,
                             (uint8_t *)data,
                             (uint16_t)len,
                             BABY_CABIN_UART_TIMEOUT_MS) == HAL_OK;
#else
    (void)data;
    (void)len;
    return false;
#endif
}

bool bsp_hal_voice_receive(uint8_t *data, size_t max_len, size_t *received_len)
{
    (void)data;
    (void)max_len;

    if (received_len != NULL)
    {
        *received_len = 0U;
    }

    return false;
}

void bsp_hal_set_output(bsp_output_t output, bool enabled)
{
    (void)output;
    (void)enabled;
}
