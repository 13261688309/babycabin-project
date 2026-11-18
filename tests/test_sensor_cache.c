#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "inc/app_config.h"
#include "protocol/device_protocol.h"

#ifndef SENSOR_CACHE_SOURCE
#define SENSOR_CACHE_SOURCE "../fetal-baby/BabyCabin/drivers/sensors.c"
#endif
#include SENSOR_CACHE_SOURCE

static uint32_t test_now;
static unsigned calls;
static int transport_ok;
static dht11_reading_t reply;
uint32_t bsp_millis(void) { return test_now; }
bool sensor_adapters_init(void) { return true; }
bool camera_module_init(void) { return false; }
bool ambient_light_init(void) { return false; }
bool air_quality_init(void) { return false; }
bool presence_detector_init(void) { return false; }
bool sensor_adapters_read_dht11(dht11_reading_t *r)
{ calls++; *r = reply; return transport_ok != 0; }
bool sensor_adapters_read_mpu6050(mpu6050_reading_t *r)
{ memset(r,0,sizeof(*r));return false; }
bool sensor_adapters_read_microphone(microphone_reading_t *r)
{ memset(r,0,sizeof(*r));return false; }
bool sensor_adapters_read_wet_probe(wet_probe_reading_t *r)
{ memset(r,0,sizeof(*r));return false; }
bool sensor_adapters_read_battery(battery_reading_t *r)
{ memset(r,0,sizeof(*r));return false; }
bool camera_module_read_detection(camera_detection_t *r)
{ memset(r,0,sizeof(*r));return false; }
bool sensor_adapters_read_ambient_light(uint16_t *lux,sensor_health_status_t *health)
{ *lux=0;*health=SENSOR_HEALTH_DISABLED;return false; }
bool sensor_adapters_read_air_quality(uint16_t *co2,uint16_t *voc,sensor_health_status_t *health)
{ *co2=*voc=0;*health=SENSOR_HEALTH_DISABLED;return false; }
bool sensor_adapters_read_presence(bool *present,uint8_t *distance,sensor_health_status_t *health)
{ *present=false;*distance=0;*health=SENSOR_HEALTH_DISABLED;return false; }

static int expect_null(system_snapshot_t *snapshot)
{
    char payload[PROTOCOL_BUFFER_SIZE];
    assert(device_protocol_encode_snapshot(snapshot,payload,sizeof(payload))>0);
    return strstr(payload,"\"temp\":null") && strstr(payload,"\"hum\":null");
}

static int startup(void)
{
    system_snapshot_t s;
    test_now=1234U;calls=0;transport_ok=0;
    assert(sensors_init() && sensors_read(&s));
    if (!expect_null(&s)) { puts("FAIL sensor startup encoded fabricated healthy temperature/humidity");return 0; }
    assert(s.timestamp_ms==test_now && calls==0);
#if SENSOR_ENABLE_DHT11
    assert(s.dht11_health==SENSOR_HEALTH_STALE);
#else
    assert(s.dht11_health==SENSOR_HEALTH_DISABLED);
#endif
    puts("PASS sensor startup: no fabricated observation, actual aggregation tick, no early read.");
    return 1;
}

static int failed_cadence(void)
{
    system_snapshot_t s;
    test_now=0;calls=0;transport_ok=0;reply.valid=false;
    assert(sensors_init());test_now=2000;
    assert(sensors_read(&s));
#if SENSOR_ENABLE_DHT11
    if (calls!=1) { puts("FAIL no first due read");return 0; }
    for (unsigned i=1;i<2000;i++) { test_now=2000+i;assert(sensors_read(&s)); }
    if (calls!=1) { puts("FAIL unsuccessful DHT attempts bypassed the original 2000ms interval");return 0; }
    assert(s.dht11_health==SENSOR_HEALTH_READ_FAILED && expect_null(&s));
    test_now=4000;assert(sensors_read(&s) && calls==2);
#else
    test_now=100000;assert(sensors_read(&s) && calls==0 && expect_null(&s));
#endif
    puts("PASS sensor retry cadence: failures remain invalid and cannot trigger sub-period retries.");
    return 1;
}

static int lifecycle(void)
{
    system_snapshot_t s;
    const uint32_t bases[]={0,UINT32_MAX-1000U};
    for (unsigned n=0;n<2;n++) {
        test_now=bases[n];calls=0;transport_ok=1;reply=(dht11_reading_t){23.5f,48.0f,true};
        assert(sensors_init());test_now+=1999;assert(sensors_read(&s) && calls==0 && expect_null(&s));
        test_now++;assert(sensors_read(&s));
#if SENSOR_ENABLE_DHT11
        assert(calls==1 && s.dht11_health==SENSOR_HEALTH_OK && s.temperature_c==23.5f && s.humidity_rh==48.0f);
        test_now++;assert(sensors_read(&s) && calls==1 && s.dht11_health==SENSOR_HEALTH_OK);
        transport_ok=0;reply=(dht11_reading_t){99,99,false};test_now+=1999;
        assert(sensors_read(&s) && calls==2 && s.dht11_health==SENSOR_HEALTH_READ_FAILED && expect_null(&s));
        test_now++;assert(sensors_read(&s) && calls==2 && expect_null(&s));
        transport_ok=1;reply=(dht11_reading_t){21,55,true};test_now+=1999;
        assert(sensors_read(&s) && calls==3 && s.dht11_health==SENSOR_HEALTH_OK && s.temperature_c==21);
#else
        assert(calls==0 && s.dht11_health==SENSOR_HEALTH_DISABLED && expect_null(&s));
#endif
        assert(sensors_init() && sensors_read(&s) && expect_null(&s));
    }
    puts("PASS sensor lifecycle: cache only after real success, failure hidden from JSON, recovery/reinit and tick wrap.");
    return 1;
}

static int malformed_samples(void)
{
#if SENSOR_ENABLE_DHT11
    const dht11_reading_t invalid[]={{20,40,false},{NAN,40,true},{20,NAN,true},{INFINITY,40,true},
                                    {20,INFINITY,true},{20,-1,true},{20,101,true}};
    for (unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        system_snapshot_t s;
        test_now=0;calls=0;transport_ok=1;reply=invalid[i];assert(sensors_init());
        test_now=2000;assert(sensors_read(&s));
        assert(calls==1 && s.dht11_health==SENSOR_HEALTH_READ_FAILED && expect_null(&s));
        assert(s.timestamp_ms==test_now);
    }
#endif
    assert(!sensors_read(NULL));
    puts("PASS sensor malformed samples: validity, finite observations, humidity bounds and NULL argument.");
    return 1;
}

int main(int argc,char **argv)
{
    if (argc==2 && strcmp(argv[1],"--cadence-repro")==0) { return failed_cadence()?0:1; }
    if (!startup()) { return 1; }
    if (argc==2 && strcmp(argv[1],"--repro-only")==0) { return 0; }
    if (!failed_cadence() || !lifecycle() || !malformed_samples()) { return 1; }
    puts("All optional sensor-cache tests passed; adapter replies are synthetic, no hardware.");
    return 0;
}
