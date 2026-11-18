/* Executes the production driver against ideal digital pulses, not analogue hardware. */
#include "dht11.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef test_gpio[8];
TestCoreDebug test_core_debug;
static TestDwt cycles;
uint32_t SystemCoreClock = 1000000U;
static uint32_t tick, mode, release_cycle, start_tick, input_switch_delay_us;
static unsigned writes, reads, releases;
static int released, bus_kind;
static uint8_t packet[5];
static uint32_t zero_high_us = 24U, one_high_us = 71U;
static unsigned packet_bits = 40U;

TestDwt *test_dwt_access(void)
{
    if ((cycles.CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) { cycles.CYCCNT++; }
    return &cycles;
}

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }

static void release_bus(void)
{
    release_cycle = cycles.CYCCNT;
    released = 1;
    releases++;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    assert(port == GPIOB && gpio->Pin == GPIO_PIN_0);
    assert(gpio->Mode == GPIO_MODE_INPUT || gpio->Mode == GPIO_MODE_OUTPUT_OD);
    assert(gpio->Pull == GPIO_PULLUP);
    if (gpio->Mode == GPIO_MODE_OUTPUT_OD) {
        assert((port->ODR & GPIO_PIN_0) == 0U);
        start_tick = tick;
    } else if (mode == GPIO_MODE_OUTPUT_OD && (port->ODR & GPIO_PIN_0) == 0U) {
        release_bus();
    }
    if (gpio->Mode == GPIO_MODE_INPUT && mode == GPIO_MODE_OUTPUT_OD && released) {
        input_switch_delay_us = cycles.CYCCNT - release_cycle;
    }
    mode = gpio->Mode;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port == GPIOB && pin == GPIO_PIN_0);
    writes++;
    if (state == GPIO_PIN_SET) {
        if (mode == GPIO_MODE_OUTPUT_OD && (port->ODR & pin) == 0U) { release_bus(); }
        port->ODR |= pin;
    } else { port->ODR &= ~pin; }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    uint32_t elapsed, edge = 183U;
    assert(port == GPIOB && pin == GPIO_PIN_0);
    reads++;
    if (mode == GPIO_MODE_OUTPUT_OD && (port->ODR & pin) == 0U) { return GPIO_PIN_RESET; }
    if (bus_kind == 1 || !released) { return GPIO_PIN_SET; }
    if (bus_kind == 2) { return GPIO_PIN_RESET; }
    elapsed = cycles.CYCCNT - release_cycle;
    if (elapsed < 13U) { return GPIO_PIN_SET; }
    if (elapsed < 96U) { return GPIO_PIN_RESET; }
    if (elapsed < edge) { return GPIO_PIN_SET; }
    for (unsigned bit = 0U; bit < packet_bits; bit++) {
        uint32_t high = ((packet[bit / 8U] >> (7U - bit % 8U)) & 1U) ? one_high_us : zero_high_us;
        if (elapsed < edge + 54U) { return GPIO_PIN_RESET; }
        edge += 54U;
        if (elapsed < edge + high) { return GPIO_PIN_SET; }
        edge += high;
    }
    return elapsed < edge + 54U ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static void set_packet(uint8_t humidity, uint8_t humidity_dec, uint8_t temperature, uint8_t temperature_dec)
{
    packet[0] = humidity; packet[1] = humidity_dec;
    packet[2] = temperature; packet[3] = temperature_dec;
    packet[4] = (uint8_t)(humidity + humidity_dec + temperature + temperature_dec);
}

static void reset(DHT11_HandleTypeDef *handle, uint32_t now)
{
    memset(test_gpio, 0, sizeof(test_gpio));
    memset(handle, 0, sizeof(*handle));
    memset(&cycles, 0, sizeof(cycles));
    tick = now; mode = GPIO_MODE_INPUT; input_switch_delay_us = UINT32_MAX;
    writes = reads = releases = 0U; released = bus_kind = 0;
    zero_high_us = 24U; one_high_us = 71U;
    packet_bits = 40U;
    set_packet(53U, 0U, 24U, 4U);
    DHT11_Init(handle, GPIOB, GPIO_PIN_0);
}

static DHT11_StatusTypeDef transaction(DHT11_HandleTypeDef *handle, DHT11_DataTypeDef *data)
{
    tick = handle->init_tick_ms + DHT11_POWER_ON_WAIT_MS;
    assert(DHT11_StartRead(handle) == DHT11_OK);
    tick += 20U;
    return DHT11_Task(handle, data);
}

static void expect_unchanged(DHT11_HandleTypeDef *handle, DHT11_StatusTypeDef status)
{
    DHT11_DataTypeDef data = {11U, 22U, 33U, 44U};
    const DHT11_DataTypeDef original = data;
    assert(transaction(handle, &data) == status);
    assert(memcmp(&data, &original, sizeof(data)) == 0);
    assert(mode == GPIO_MODE_INPUT && handle->read_pending == 0U);
}

static void positive(void)
{
    DHT11_HandleTypeDef handle; DHT11_DataTypeDef data;
    reset(&handle, 0U);
    assert(transaction(&handle, &data) == DHT11_OK);
    assert(data.humidity_int == 53U && data.temperature_dec == 4U);
    assert(DHT11_GetTemperature(&data) > 24.39f && DHT11_GetTemperature(&data) < 24.41f);
    assert(DHT11_GetHumidity(&data) == 53.0f);
    assert(tick - start_tick == 20U && mode == GPIO_MODE_INPUT && releases == 1U);
    assert(input_switch_delay_us >= 15U && input_switch_delay_us <= 20U);
    assert(handle.read_pending == 0U);
    assert(DHT11_Task(&handle, &data) == DHT11_ERROR_PARAM);
}

static void negative(void)
{
    DHT11_HandleTypeDef handle; DHT11_DataTypeDef data;
    reset(&handle, 0U); set_packet(53U, 0U, 10U, 0x81U);
    assert(transaction(&handle, &data) == DHT11_OK);
    assert(DHT11_GetTemperature(&data) < -10.09f && DHT11_GetTemperature(&data) > -10.11f);
    reset(&handle, 0U); set_packet(5U, 0U, 20U, 0x80U);
    assert(transaction(&handle, &data) == DHT11_OK && DHT11_GetTemperature(&data) == -20.0f);
    reset(&handle, 0U); set_packet(95U, 0U, 60U, 0U);
    assert(transaction(&handle, &data) == DHT11_OK && DHT11_GetTemperature(&data) == 60.0f);
    reset(&handle, 0U); set_packet(53U, 0U, 0U, 0x81U);
    assert(transaction(&handle, &data) == DHT11_OK && DHT11_GetTemperature(&data) < -0.09f);
}

static void malformed(void)
{
    const uint8_t frames[][4] = {{4,0,24,0}, {96,0,24,0}, {53,1,24,0},
        {53,0,24,10}, {53,0,60,1}, {53,0,20,0x81}, {53,0,255,0}, {53,0,10,0xFF}};
    DHT11_HandleTypeDef handle;
    for (unsigned i = 0U; i < sizeof(frames) / sizeof(frames[0]); i++) {
        reset(&handle, 0U);
        set_packet(frames[i][0], frames[i][1], frames[i][2], frames[i][3]);
        expect_unchanged(&handle, DHT11_ERROR_DATA);
    }
}

static void checksum(void)
{
    DHT11_HandleTypeDef handle;
    reset(&handle, 0U); packet[4] ^= 1U;
    expect_unchanged(&handle, DHT11_ERROR_CHECKSUM);
}

static void raw_trace(void)
{
    DHT11_HandleTypeDef handle; DHT11_DataTypeDef data;
    reset(&handle, 0U);
    assert(handle.raw_bytes == 0U);
    assert(transaction(&handle, &data) == DHT11_OK);
    assert(handle.raw_bytes == 5U && memcmp(handle.last_raw, packet, 5U) == 0);
    reset(&handle, 0U); packet[4] ^= 1U;
    expect_unchanged(&handle, DHT11_ERROR_CHECKSUM);
    assert(handle.raw_bytes == 5U && memcmp(handle.last_raw, packet, 5U) == 0);
    reset(&handle, 0U); set_packet(53U, 1U, 24U, 0U);
    expect_unchanged(&handle, DHT11_ERROR_DATA);
    assert(handle.raw_bytes == 5U && memcmp(handle.last_raw, packet, 5U) == 0);
    tick = handle.last_start_tick_ms + DHT11_MIN_READ_INTERVAL_MS;
    bus_kind = 1;
    assert(DHT11_StartRead(&handle) == DHT11_OK);
    assert(handle.raw_bytes == 0U);
    for (unsigned i = 0U; i < 5U; i++) { assert(handle.last_raw[i] == 0U); }
    tick += 20U;
    assert(DHT11_Task(&handle, &data) == DHT11_ERROR_TIMEOUT);
    assert(handle.raw_bytes == 0U);
    DHT11_Init(&handle, GPIOB, GPIO_PIN_0);
    assert(handle.raw_bytes == 0U);
}

static void partial_trace(void)
{
    DHT11_HandleTypeDef handle;
    reset(&handle, 0U); packet_bits = 16U;
    expect_unchanged(&handle, DHT11_ERROR_TIMEOUT);
    assert(handle.raw_bytes == 2U);
    assert(handle.last_raw[0] == packet[0] && handle.last_raw[1] == packet[1]);
    for (unsigned i = 2U; i < 5U; i++) { assert(handle.last_raw[i] == 0U); }
}

static void warmup(void)
{
    DHT11_HandleTypeDef handle;
    reset(&handle, 123U);
    tick = 1122U;
    assert(DHT11_StartRead(&handle) == DHT11_BUSY);
    assert(writes == 0U && mode == GPIO_MODE_INPUT && !handle.read_pending && !handle.has_started);
    tick = 1123U;
    assert(DHT11_StartRead(&handle) == DHT11_OK && mode == GPIO_MODE_OUTPUT_OD);
    assert(handle.last_start_tick_ms == tick);
    assert(DHT11_StartRead(&handle) == DHT11_ERROR_PARAM);
}

static void cooldown(void)
{
    DHT11_HandleTypeDef handle; DHT11_DataTypeDef data;
    reset(&handle, 0U); assert(transaction(&handle, &data) == DHT11_OK);
    tick = 3000U; assert(DHT11_StartRead(&handle) == DHT11_BUSY);
    tick = 3099U; assert(DHT11_StartRead(&handle) == DHT11_BUSY);
    tick = 3100U; assert(DHT11_StartRead(&handle) == DHT11_OK);
    tick += 30U; assert(DHT11_Task(&handle, &data) == DHT11_ERROR_TIMEOUT);
    tick = 5199U; assert(DHT11_StartRead(&handle) == DHT11_BUSY);
    tick = 5200U; assert(DHT11_StartRead(&handle) == DHT11_OK);
}

static void late_start(void)
{
    DHT11_HandleTypeDef handle; DHT11_DataTypeDef data;
    reset(&handle, 0U); tick = 1000U;
    assert(DHT11_StartRead(&handle) == DHT11_OK);
    tick = 1019U;
    assert(DHT11_Task(&handle, &data) == DHT11_BUSY && handle.read_pending);
    assert(mode == GPIO_MODE_OUTPUT_OD && reads == 0U && releases == 0U);
    tick = 1029U;
    assert(DHT11_Task(&handle, &data) == DHT11_OK);
    reset(&handle, 0U); tick = 1000U;
    assert(DHT11_StartRead(&handle) == DHT11_OK);
    tick = 1030U;
    data.humidity_int = 77U;
    assert(DHT11_Task(&handle, &data) == DHT11_ERROR_TIMEOUT);
    assert(data.humidity_int == 77U && mode == GPIO_MODE_INPUT && !handle.read_pending);
    assert(reads == 0U && releases == 1U);
}

static void missing_bus(void)
{
    DHT11_HandleTypeDef handle;
    for (int kind = 1; kind <= 2; kind++) {
        reset(&handle, 0U); bus_kind = kind;
        expect_unchanged(&handle, DHT11_ERROR_TIMEOUT);
        assert(cycles.CYCCNT - release_cycle < 2200U);
    }
}

static void wrap(void)
{
    DHT11_HandleTypeDef handle; DHT11_DataTypeDef data;
    reset(&handle, UINT32_MAX - 500U);
    tick = 498U; assert(DHT11_StartRead(&handle) == DHT11_BUSY);
    tick = 499U; assert(DHT11_StartRead(&handle) == DHT11_OK);
    cycles.CYCCNT = UINT32_MAX - 5U;
    tick += 20U;
    assert(DHT11_Task(&handle, &data) == DHT11_OK);
    assert(cycles.CYCCNT < 10000U && mode == GPIO_MODE_INPUT);
    reset(&handle, UINT32_MAX - 1500U);
    tick = UINT32_MAX - 500U; assert(DHT11_StartRead(&handle) == DHT11_OK);
    tick += 20U; assert(DHT11_Task(&handle, &data) == DHT11_OK);
    tick = 1598U; assert(DHT11_StartRead(&handle) == DHT11_BUSY);
    tick = 1599U; assert(DHT11_StartRead(&handle) == DHT11_OK);
}

static void ownership(void)
{
    DHT11_HandleTypeDef handle; DHT11_DataTypeDef data;
    reset(&handle, 0U);
    cycles.CYCCNT = 12345U;
    DHT11_Init(&handle, GPIOB, GPIO_PIN_0);
    assert(cycles.CYCCNT >= 12345U);
    tick = 1000U; assert(DHT11_StartRead(&handle) == DHT11_OK);
    assert(DHT11_Task(&handle, NULL) == DHT11_ERROR_PARAM);
    assert(mode == GPIO_MODE_INPUT && !handle.read_pending);
    assert(DHT11_StartRead(NULL) == DHT11_ERROR_PARAM);
    assert(DHT11_Task(NULL, &data) == DHT11_ERROR_PARAM);
    reset(&handle, 0U); tick = 1000U;
    assert(DHT11_Read(&handle, &data) == DHT11_OK);
    for (unsigned z = 23U; z <= 27U; z += 4U) {
        reset(&handle, 0U); zero_high_us = z; one_high_us = z == 23U ? 68U : 74U;
        assert(transaction(&handle, &data) == DHT11_OK);
    }
}

static const struct { const char *name; void (*run)(void); } scenarios[] = {
    {"positive", positive}, {"negative", negative}, {"malformed", malformed}, {"checksum", checksum},
    {"warmup", warmup}, {"cooldown", cooldown}, {"late-start", late_start}, {"missing-bus", missing_bus},
    {"wrap", wrap}, {"ownership", ownership}, {"raw-trace", raw_trace},
    {"partial-trace", partial_trace}};

int main(int argc, char **argv)
{
    unsigned count = 0U;
    for (unsigned i = 0U; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
        if (argc == 1 || (argc == 2 && strcmp(argv[1], scenarios[i].name) == 0)) {
            scenarios[i].run(); printf("PASS DHT11 %s\n", scenarios[i].name); count++;
        }
    }
    return count == 0U ? 2 : 0;
}
