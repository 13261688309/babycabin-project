#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    BSP_ADC_MICROPHONE = 0,
    BSP_ADC_WET_PROBE,
    BSP_ADC_BATTERY
} bsp_adc_channel_t;

typedef enum
{
    BSP_I2C_DEVICE_MPU6050 = 0,
    BSP_I2C_DEVICE_ESP32_CAMERA,
    BSP_I2C_DEVICE_BH1750,
    BSP_I2C_DEVICE_SGP30,
    BSP_I2C_DEVICE_MLX90614
} bsp_i2c_device_t;

typedef enum
{
    BSP_GPIO_WET_DIGITAL = 0,
    BSP_GPIO_PRESENCE_DET
} bsp_gpio_input_t;

typedef enum
{
    BSP_OUTPUT_WHITE_NOISE = 0,
    BSP_OUTPUT_MOTHER_VOICE,
    BSP_OUTPUT_ROCKING,
    BSP_OUTPUT_VIBRATION,
    BSP_OUTPUT_COUNT
} bsp_output_t;

void bsp_init(void);
uint32_t bsp_millis(void);
void bsp_delay_ms(uint32_t ms);
void bsp_delay_us(uint32_t us);
void bsp_send_packet(const char *packet);
void bsp_send_alert_packet(const char *packet);
bool bsp_poll_command_packet(char *buffer, size_t buffer_size);
void bsp_log(const char *message);
bool bsp_adc_read(bsp_adc_channel_t channel, uint16_t *raw);
bool bsp_adc_read_window(bsp_adc_channel_t channel, uint16_t *samples, size_t sample_count);
bool bsp_gpio_read(bsp_gpio_input_t input, bool *active);
bool bsp_i2c_mem_read(bsp_i2c_device_t device, uint8_t reg, uint8_t *data, size_t len);
bool bsp_i2c_mem_write(bsp_i2c_device_t device, uint8_t reg, const uint8_t *data, size_t len);
void bsp_dht11_drive_low(void);
void bsp_dht11_release(void);
bool bsp_dht11_read(bool *level);
bool bsp_voice_send(const uint8_t *data, size_t len);
bool bsp_voice_receive(uint8_t *data, size_t max_len, size_t *received_len);
void bsp_set_output(bsp_output_t output, bool enabled);
