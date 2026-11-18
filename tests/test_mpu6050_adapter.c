#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "inc/app_config.h"

/* The harness supplies a generated shadow config; production stays disabled. */
#if !SENSOR_ENABLE_MPU6050
#error MPU adapter test requires its isolated enabled-branch config
#endif
#ifndef SENSOR_ADAPTERS_SOURCE
#define SENSOR_ADAPTERS_SOURCE "../fetal-baby/BabyCabin/drivers/sensor_adapters.c"
#endif
#include SENSOR_ADAPTERS_SOURCE

static int16_t sample[3];
static unsigned fail_write, writes, reads;
static int fail_read;

bool bsp_i2c_mem_write(bsp_i2c_device_t device, uint8_t reg, const uint8_t *data, size_t len)
{
    assert(device == BSP_I2C_DEVICE_MPU6050 && len == 1U && *data == 0U);
    assert(reg == 0x6BU || reg == 0x1CU);
    writes++;
    return writes != fail_write;
}

bool bsp_i2c_mem_read(bsp_i2c_device_t device, uint8_t reg, uint8_t *data, size_t len)
{
    assert(device == BSP_I2C_DEVICE_MPU6050 && reg == 0x3BU && len == 6U);
    reads++;
    if (fail_read) { return false; }
    for (unsigned axis = 0; axis < 3; ++axis) {
        data[axis * 2] = (uint8_t)((uint16_t)sample[axis] >> 8);
        data[axis * 2 + 1] = (uint8_t)sample[axis];
    }
    return true;
}

bool ambient_light_read(uint16_t *lux, sensor_health_status_t *health)
{ (void)lux; (void)health; return false; }
bool air_quality_read(uint16_t *co2, uint16_t *voc, sensor_health_status_t *health)
{ (void)co2; (void)voc; (void)health; return false; }
bool presence_detector_poll(bool *present, uint8_t *distance, sensor_health_status_t *health)
{ (void)present; (void)distance; (void)health; return false; }
bool bsp_gpio_read(bsp_gpio_input_t input, bool *active)
{ (void)input; (void)active; return false; }

static int numeric_range(void)
{
    const int16_t previous[] = {INT16_MIN, 0, INT16_MAX};
    unsigned checks = 0;
    for (unsigned p = 0; p < 3; ++p) {
        for (int32_t current = INT16_MIN; current <= INT16_MAX; ++current) {
            int32_t signed_delta = current - previous[p];
            uint32_t expected = (uint32_t)(signed_delta < 0 ? -signed_delta : signed_delta);
            uint32_t actual = (uint32_t)abs_i16_delta((int16_t)current, previous[p]);
            if (actual != expected) {
                printf("FAIL MPU delta current=%ld previous=%d expected=%lu got=%lu\n",
                       (long)current, previous[p], (unsigned long)expected, (unsigned long)actual);
                return 0;
            }
            checks++;
        }
    }
    printf("PASS MPU signed delta: %u exact comparisons spanning all 65536 sample values and three references.\n", checks);
    return 1;
}

static int axis_vectors(void)
{
    const int16_t previous[] = {INT16_MIN, 0, INT16_MAX};
    mpu6050_reading_t reading;
    unsigned cases = 0;
    for (unsigned axis = 0; axis < 3; ++axis) {
        for (unsigned p = 0; p < 3; ++p) {
            for (int32_t current = INT16_MIN; current <= INT16_MAX; ++current) {
                sample[0] = sample[1] = sample[2] = 0;
                sample[axis] = previous[p];
                assert(sensor_adapters_read_mpu6050(&reading) && reading.valid);
                sample[axis] = (int16_t)current;
                assert(sensor_adapters_read_mpu6050(&reading) && reading.valid);
                int32_t delta = current - previous[p];
                if (delta < 0) { delta = -delta; }
                if (reading.motion_active != (delta > MOTION_ACCEL_DELTA_THRESHOLD) ||
                    reading.ax != sample[0] || reading.ay != sample[1] || reading.az != sample[2]) {
                    printf("FAIL MPU axis=%u current=%ld previous=%d\n", axis, (long)current, previous[p]);
                    return 0;
                }
                cases++;
            }
        }
    }
    printf("PASS MPU production adapter: %u big-endian axis vectors, including exact threshold boundaries.\n", cases);
    return 1;
}

static int lifecycle(void)
{
    mpu6050_reading_t reading;
    unsigned before;
    for (unsigned failure = 1; failure <= 2; ++failure) {
        writes = 0; fail_write = failure;
        if (sensor_adapters_init()) { puts("FAIL MPU initialization reported success after I2C write failure"); return 0; }
        before = reads;
        assert(!sensor_adapters_read_mpu6050(&reading) && !reading.valid && !reading.motion_active);
        assert(reads == before && writes == failure);
    }
    writes = 0; fail_write = 0;
    assert(sensor_adapters_init() && writes == 2U);
    sample[0] = INT16_MAX; sample[1] = sample[2] = 0;
    assert(sensor_adapters_read_mpu6050(&reading) && !reading.motion_active);
    sample[0] = INT16_MIN;
    assert(sensor_adapters_read_mpu6050(&reading) && reading.motion_active);
    fail_read = 1;
    memset(&reading, 0xFF, sizeof(reading));
    assert(!sensor_adapters_read_mpu6050(&reading) && !reading.valid && !reading.motion_active);
    assert(reading.ax == 0 && reading.ay == 0 && reading.az == 0);
    fail_read = 0;sample[0] = INT16_MAX;
    assert(sensor_adapters_read_mpu6050(&reading) && !reading.motion_active);
    sample[0] = INT16_MIN;
    assert(sensor_adapters_init());
    assert(sensor_adapters_read_mpu6050(&reading) && !reading.motion_active);
    before = reads;assert(!sensor_adapters_read_mpu6050(NULL) && reads == before);
    puts("PASS MPU lifecycle: both setup failures, no pre-ready reads, invalid failure outputs, fresh baseline after read failure/reinit, NULL argument.");
    return 1;
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--lifecycle-repro") == 0) { return lifecycle() ? 0 : 1; }
    if (!numeric_range()) { return 1; }
    if (argc == 2 && strcmp(argv[1], "--repro-only") == 0) { return 0; }
    {
        mpu6050_reading_t reading;
        unsigned before = reads;
        assert(!sensor_adapters_read_mpu6050(&reading) && !reading.valid && reads == before);
    }
    assert(sensor_adapters_init());
    if (!axis_vectors() || !lifecycle()) { return 1; }
    puts("All optional MPU adapter tests passed; synthetic I2C only, no hardware enablement.");
    return 0;
}
