#ifndef DHT11_H7_H
#define DHT11_H7_H

#include "baby_care_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DHT11_OK = 0,
    DHT11_ERROR_TIMEOUT,
    DHT11_ERROR_CHECKSUM,
    DHT11_ERROR_PARAM,
    DHT11_BUSY,
    DHT11_ERROR_DATA
} DHT11_StatusTypeDef;

/* Aosong DHT11 V1.3: power-on settling and more than 2 s between requests. */
#define DHT11_POWER_ON_WAIT_MS 1000U
#define DHT11_MIN_READ_INTERVAL_MS 2100U

typedef struct
{
    uint8_t humidity_int;
    uint8_t humidity_dec;
    uint8_t temperature_int;
    uint8_t temperature_dec;
} DHT11_DataTypeDef;

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t timeout_us;
    uint32_t start_tick_ms;
    uint32_t init_tick_ms;
    uint32_t last_start_tick_ms;
    uint8_t read_pending;
    uint8_t has_started;
    /* Latest transaction bytes are diagnostic only, not validated readings. */
    uint8_t last_raw[5];
    uint8_t raw_bytes;
} DHT11_HandleTypeDef;

void DHT11_Init(DHT11_HandleTypeDef *hdht, GPIO_TypeDef *port, uint16_t pin);
DHT11_StatusTypeDef DHT11_StartRead(DHT11_HandleTypeDef *hdht);
/* Service at least once between 20 and 29 ms after StartRead; late starts abort. */
DHT11_StatusTypeDef DHT11_Task(DHT11_HandleTypeDef *hdht, DHT11_DataTypeDef *data);
DHT11_StatusTypeDef DHT11_Read(DHT11_HandleTypeDef *hdht, DHT11_DataTypeDef *data);
float DHT11_GetTemperature(const DHT11_DataTypeDef *data);
float DHT11_GetHumidity(const DHT11_DataTypeDef *data);

#ifdef __cplusplus
}
#endif

#endif
