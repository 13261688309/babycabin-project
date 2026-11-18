#include "bsp.h"

#include <stddef.h>

#if defined(__GNUC__)
#define BSP_WEAK __attribute__((weak))
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define BSP_WEAK __weak
#else
#define BSP_WEAK
#endif

BSP_WEAK bool bsp_hal_init(void)
{
    return true;
}

BSP_WEAK uint32_t bsp_hal_millis(void)
{
    return 0U;
}

BSP_WEAK void bsp_hal_delay_ms(uint32_t ms)
{
    (void)ms;
}

BSP_WEAK void bsp_hal_delay_us(uint32_t us)
{
    (void)us;
}

BSP_WEAK bool bsp_hal_send_packet(const char *packet, size_t len)
{
    (void)packet;
    (void)len;
    return false;
}

BSP_WEAK bool bsp_hal_send_alert_packet(const char *packet, size_t len)
{
    return bsp_hal_send_packet(packet, len);
}

BSP_WEAK bool bsp_hal_poll_command_packet(char *buffer, size_t buffer_size)
{
    (void)buffer;
    (void)buffer_size;
    return false;
}

BSP_WEAK void bsp_hal_log(const char *message)
{
    (void)message;
}

BSP_WEAK bool bsp_hal_adc_read(bsp_adc_channel_t channel, uint16_t *raw)
{
    (void)channel;
    (void)raw;
    return false;
}

BSP_WEAK bool bsp_hal_adc_read_window(bsp_adc_channel_t channel, uint16_t *samples, size_t sample_count)
{
    size_t index;

    if (samples == NULL || sample_count == 0U)
    {
        return false;
    }

    for (index = 0U; index < sample_count; index++)
    {
        if (!bsp_hal_adc_read(channel, &samples[index]))
        {
            return false;
        }
    }

    return true;
}

BSP_WEAK bool bsp_hal_gpio_read(bsp_gpio_input_t input, bool *active)
{
    (void)input;
    (void)active;
    return false;
}

BSP_WEAK bool bsp_hal_i2c_mem_read(bsp_i2c_device_t device, uint8_t reg, uint8_t *data, size_t len)
{
    (void)device;
    (void)reg;
    (void)data;
    (void)len;
    return false;
}

BSP_WEAK bool bsp_hal_i2c_mem_write(bsp_i2c_device_t device, uint8_t reg, const uint8_t *data, size_t len)
{
    (void)device;
    (void)reg;
    (void)data;
    (void)len;
    return false;
}

BSP_WEAK void bsp_hal_dht11_drive_low(void)
{
}

BSP_WEAK void bsp_hal_dht11_release(void)
{
}

BSP_WEAK bool bsp_hal_dht11_read(bool *level)
{
    (void)level;
    return false;
}

BSP_WEAK bool bsp_hal_voice_send(const uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
    return false;
}

BSP_WEAK bool bsp_hal_voice_receive(uint8_t *data, size_t max_len, size_t *received_len)
{
    (void)data;
    (void)max_len;
    if (received_len != NULL)
    {
        *received_len = 0U;
    }
    return false;
}

BSP_WEAK void bsp_hal_set_output(bsp_output_t output, bool enabled)
{
    (void)output;
    (void)enabled;
}

void bsp_init(void)
{
    (void)bsp_hal_init();
}

uint32_t bsp_millis(void)
{
    return bsp_hal_millis();
}

void bsp_delay_ms(uint32_t ms)
{
    bsp_hal_delay_ms(ms);
}

void bsp_delay_us(uint32_t us)
{
    bsp_hal_delay_us(us);
}

void bsp_send_packet(const char *packet)
{
    size_t len = 0U;

    if (packet == NULL)
    {
        return;
    }

    while (packet[len] != '\0')
    {
        len++;
    }

    (void)bsp_hal_send_packet(packet, len);
}

void bsp_send_alert_packet(const char *packet)
{
    size_t len = 0U;

    if (packet == NULL)
    {
        return;
    }

    while (packet[len] != '\0')
    {
        len++;
    }

    (void)bsp_hal_send_alert_packet(packet, len);
}

bool bsp_poll_command_packet(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0U)
    {
        return false;
    }

    buffer[0] = '\0';
    return bsp_hal_poll_command_packet(buffer, buffer_size);
}

void bsp_log(const char *message)
{
    if (message != NULL)
    {
        bsp_hal_log(message);
    }
}

bool bsp_adc_read(bsp_adc_channel_t channel, uint16_t *raw)
{
    if (raw == NULL)
    {
        return false;
    }

    *raw = 0U;
    return bsp_hal_adc_read(channel, raw);
}

bool bsp_adc_read_window(bsp_adc_channel_t channel, uint16_t *samples, size_t sample_count)
{
    if (samples == NULL || sample_count == 0U)
    {
        return false;
    }

    return bsp_hal_adc_read_window(channel, samples, sample_count);
}

bool bsp_gpio_read(bsp_gpio_input_t input, bool *active)
{
    if (active == NULL)
    {
        return false;
    }

    *active = false;
    return bsp_hal_gpio_read(input, active);
}

bool bsp_i2c_mem_read(bsp_i2c_device_t device, uint8_t reg, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0U)
    {
        return false;
    }

    return bsp_hal_i2c_mem_read(device, reg, data, len);
}

bool bsp_i2c_mem_write(bsp_i2c_device_t device, uint8_t reg, const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0U)
    {
        return false;
    }

    return bsp_hal_i2c_mem_write(device, reg, data, len);
}

void bsp_dht11_drive_low(void)
{
    bsp_hal_dht11_drive_low();
}

void bsp_dht11_release(void)
{
    bsp_hal_dht11_release();
}

bool bsp_dht11_read(bool *level)
{
    if (level == NULL)
    {
        return false;
    }

    *level = false;
    return bsp_hal_dht11_read(level);
}

bool bsp_voice_send(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0U)
    {
        return false;
    }

    return bsp_hal_voice_send(data, len);
}

bool bsp_voice_receive(uint8_t *data, size_t max_len, size_t *received_len)
{
    if (data == NULL || max_len == 0U || received_len == NULL)
    {
        return false;
    }

    *received_len = 0U;
    return bsp_hal_voice_receive(data, max_len, received_len);
}

void bsp_set_output(bsp_output_t output, bool enabled)
{
    bsp_hal_set_output(output, enabled);
}
