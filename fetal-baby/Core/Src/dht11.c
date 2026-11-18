#include "dht11.h"
#include <string.h>

#ifndef DHT11_DEFAULT_TIMEOUT_US
#define DHT11_DEFAULT_TIMEOUT_US 1000U
#endif

#define DHT11_START_LOW_MS 20U
#define DHT11_START_MAX_LOW_MS 30U
#define DHT11_RELEASE_US 15U
#define DHT11_BIT_SAMPLE_US 40U

static void dht11_enable_gpio_clock(GPIO_TypeDef *port)
{
    if (port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (port == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    } else if (port == GPIOE) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    } else if (port == GPIOF) {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    } else if (port == GPIOG) {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    } else if (port == GPIOH) {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
#ifdef GPIOI
    else if (port == GPIOI) {
        __HAL_RCC_GPIOI_CLK_ENABLE();
    }
#endif
#ifdef GPIOJ
    else if (port == GPIOJ) {
        __HAL_RCC_GPIOJ_CLK_ENABLE();
    }
#endif
#ifdef GPIOK
    else if (port == GPIOK) {
        __HAL_RCC_GPIOK_CLK_ENABLE();
    }
#endif
}

static void dht11_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
    }
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void dht11_delay_us(uint32_t us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = (SystemCoreClock / 1000000U) * us;

    while ((DWT->CYCCNT - start) < ticks) {
    }
}

static void dht11_gpio_input(DHT11_HandleTypeDef *hdht)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = hdht->pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(hdht->port, &gpio);
}

static void dht11_gpio_output(DHT11_HandleTypeDef *hdht)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = hdht->pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(hdht->port, &gpio);
}

static void dht11_write_pin(DHT11_HandleTypeDef *hdht, GPIO_PinState state)
{
    HAL_GPIO_WritePin(hdht->port, hdht->pin, state);
}

static GPIO_PinState dht11_read_pin(DHT11_HandleTypeDef *hdht)
{
    return HAL_GPIO_ReadPin(hdht->port, hdht->pin);
}

static DHT11_StatusTypeDef dht11_wait_for_level(DHT11_HandleTypeDef *hdht,
                                                GPIO_PinState level,
                                                uint32_t timeout_us)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t timeout_ticks = (SystemCoreClock / 1000000U) * timeout_us;

    while (dht11_read_pin(hdht) != level) {
        if ((DWT->CYCCNT - start) > timeout_ticks) {
            return DHT11_ERROR_TIMEOUT;
        }
    }

    return DHT11_OK;
}

static DHT11_StatusTypeDef dht11_finish_start(DHT11_HandleTypeDef *hdht)
{
    dht11_write_pin(hdht, GPIO_PIN_SET);
    dht11_delay_us(DHT11_RELEASE_US);
    dht11_gpio_input(hdht);

    if (dht11_wait_for_level(hdht, GPIO_PIN_RESET, hdht->timeout_us) != DHT11_OK) {
        return DHT11_ERROR_TIMEOUT;
    }
    if (dht11_wait_for_level(hdht, GPIO_PIN_SET, hdht->timeout_us) != DHT11_OK) {
        return DHT11_ERROR_TIMEOUT;
    }
    if (dht11_wait_for_level(hdht, GPIO_PIN_RESET, hdht->timeout_us) != DHT11_OK) {
        return DHT11_ERROR_TIMEOUT;
    }

    return DHT11_OK;
}

static DHT11_StatusTypeDef dht11_read_byte(DHT11_HandleTypeDef *hdht, uint8_t *value)
{
    uint8_t data = 0U;

    if (value == NULL) {
        return DHT11_ERROR_PARAM;
    }

    for (uint8_t i = 0U; i < 8U; i++) {
        if (dht11_wait_for_level(hdht, GPIO_PIN_SET, hdht->timeout_us) != DHT11_OK) {
            return DHT11_ERROR_TIMEOUT;
        }

        dht11_delay_us(DHT11_BIT_SAMPLE_US);
        data <<= 1U;

        if (dht11_read_pin(hdht) == GPIO_PIN_SET) {
            data |= 1U;
            if (dht11_wait_for_level(hdht, GPIO_PIN_RESET, hdht->timeout_us) != DHT11_OK) {
                return DHT11_ERROR_TIMEOUT;
            }
        }
    }

    *value = data;
    return DHT11_OK;
}

void DHT11_Init(DHT11_HandleTypeDef *hdht, GPIO_TypeDef *port, uint16_t pin)
{
    if (hdht == NULL || port == NULL || pin == 0U) {
        return;
    }

    hdht->port = port;
    hdht->pin = pin;
    hdht->timeout_us = DHT11_DEFAULT_TIMEOUT_US;
    hdht->start_tick_ms = 0U;
    hdht->init_tick_ms = HAL_GetTick();
    hdht->last_start_tick_ms = 0U;
    hdht->read_pending = 0U;
    hdht->has_started = 0U;
    memset(hdht->last_raw, 0, sizeof(hdht->last_raw));
    hdht->raw_bytes = 0U;

    dht11_dwt_init();
    dht11_enable_gpio_clock(port);
    dht11_gpio_input(hdht);
}

DHT11_StatusTypeDef DHT11_StartRead(DHT11_HandleTypeDef *hdht)
{
    uint32_t now;

    if (hdht == NULL || hdht->port == NULL || hdht->pin == 0U || hdht->read_pending != 0U) {
        return DHT11_ERROR_PARAM;
    }
    now = HAL_GetTick();
    if ((now - hdht->init_tick_ms) < DHT11_POWER_ON_WAIT_MS ||
        (hdht->has_started != 0U &&
         (now - hdht->last_start_tick_ms) < DHT11_MIN_READ_INTERVAL_MS)) {
        return DHT11_BUSY;
    }
    memset(hdht->last_raw, 0, sizeof(hdht->last_raw));
    hdht->raw_bytes = 0U;
    /* Set ODR before switching the pin to output to avoid a high pulse. */
    dht11_write_pin(hdht, GPIO_PIN_RESET);
    dht11_gpio_output(hdht);
    hdht->start_tick_ms = now;
    hdht->last_start_tick_ms = now;
    hdht->read_pending = 1U;
    hdht->has_started = 1U;
    return DHT11_OK;
}

DHT11_StatusTypeDef DHT11_Task(DHT11_HandleTypeDef *hdht, DHT11_DataTypeDef *data)
{
    uint8_t raw[5] = {0};
    uint8_t temperature_fraction;
    uint16_t temperature_tenths;
    uint32_t elapsed_ms;
    DHT11_StatusTypeDef status;

    if (hdht == NULL || hdht->port == NULL || hdht->pin == 0U) {
        return DHT11_ERROR_PARAM;
    }
    if (data == NULL) {
        hdht->read_pending = 0U;
        dht11_gpio_input(hdht);
        return DHT11_ERROR_PARAM;
    }

    if (hdht->read_pending == 0U) { return DHT11_ERROR_PARAM; }
    elapsed_ms = HAL_GetTick() - hdht->start_tick_ms;
    if (elapsed_ms < DHT11_START_LOW_MS) { return DHT11_BUSY; }
    hdht->read_pending = 0U;
    /* Abort at 30 ms to leave margin for the millisecond tick's quantization. */
    if (elapsed_ms >= DHT11_START_MAX_LOW_MS) {
        dht11_gpio_input(hdht);
        return DHT11_ERROR_TIMEOUT;
    }
    status = dht11_finish_start(hdht);
    if (status != DHT11_OK) {
        dht11_gpio_input(hdht);
        return status;
    }

    for (uint8_t i = 0U; i < 5U; i++) {
        status = dht11_read_byte(hdht, &raw[i]);
        if (status != DHT11_OK) {
            dht11_gpio_input(hdht);
            return status;
        }
        hdht->last_raw[i] = raw[i];
        hdht->raw_bytes = (uint8_t)(i + 1U);
    }

    if (((uint8_t)(raw[0] + raw[1] + raw[2] + raw[3])) != raw[4]) {
        dht11_gpio_input(hdht);
        return DHT11_ERROR_CHECKSUM;
    }

    temperature_fraction = raw[3] & 0x7FU;
    temperature_tenths = (uint16_t)raw[2] * 10U + temperature_fraction;
    if (raw[0] < 5U || raw[0] > 95U || raw[1] != 0U || temperature_fraction > 9U ||
        temperature_tenths > ((raw[3] & 0x80U) != 0U ? 200U : 600U)) {
        dht11_gpio_input(hdht);
        return DHT11_ERROR_DATA;
    }

    data->humidity_int = raw[0];
    data->humidity_dec = raw[1];
    data->temperature_int = raw[2];
    data->temperature_dec = raw[3];
    dht11_gpio_input(hdht);
    return DHT11_OK;
}

DHT11_StatusTypeDef DHT11_Read(DHT11_HandleTypeDef *hdht, DHT11_DataTypeDef *data)
{
    DHT11_StatusTypeDef status = DHT11_StartRead(hdht);
    if (status != DHT11_OK) { return status; }
    HAL_Delay(DHT11_START_LOW_MS);
    return DHT11_Task(hdht, data);
}

float DHT11_GetTemperature(const DHT11_DataTypeDef *data)
{
    float magnitude;

    if (data == NULL) {
        return 0.0f;
    }

    magnitude = (float)data->temperature_int + ((float)(data->temperature_dec & 0x7FU) / 10.0f);
    return (data->temperature_dec & 0x80U) != 0U ? -magnitude : magnitude;
}

float DHT11_GetHumidity(const DHT11_DataTypeDef *data)
{
    if (data == NULL) {
        return 0.0f;
    }

    return (float)data->humidity_int + ((float)data->humidity_dec / 10.0f);
}
