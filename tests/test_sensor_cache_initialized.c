/* Reuse only the original simulated adapters; never call its uninitialized fixtures. */
#define main historical_cache_fixture_main
#include "test_sensor_cache.c"
#undef main

static int clean_read(system_snapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    return sensors_read(snapshot);
}

static int clean_startup(void)
{
    system_snapshot_t s = {0};
    test_now = 1234;calls = 0;transport_ok = 0;
    assert(sensors_init() && clean_read(&s));
    if (!expect_null(&s)) { puts("FAIL initialized snapshot still encoded fabricated boot observations");return 0; }
    assert(s.timestamp_ms == test_now && calls == 0);
    return 1;
}

static int clean_cadence(void)
{
    system_snapshot_t s = {0};
    test_now = 0;calls = 0;transport_ok = 0;reply.valid = false;
    assert(sensors_init());test_now = 2000;assert(clean_read(&s));
#if SENSOR_ENABLE_DHT11
    assert(calls == 1);
    for (unsigned i = 1;i < 2000;i++) { test_now = 2000+i;assert(clean_read(&s)); }
    if (calls != 1) { puts("FAIL initialized failure cadence bypassed 2000ms");return 0; }
    assert(s.dht11_health == SENSOR_HEALTH_READ_FAILED && expect_null(&s));
    test_now = 4000;assert(clean_read(&s) && calls == 2);
#else
    test_now = 100000;assert(clean_read(&s) && calls == 0 && expect_null(&s));
#endif
    return 1;
}

int main(int argc,char **argv)
{
    system_snapshot_t s = {0};
    if (argc == 2 && strcmp(argv[1],"--cadence-repro") == 0) { return clean_cadence()?0:1; }
    if (!clean_startup()) { return 1; }
    if (argc == 2 && strcmp(argv[1],"--repro-only") == 0) { return 0; }
    if (!clean_cadence()) { return 1; }
    const uint32_t starts[] = {0,UINT32_MAX-1000U};
    for (unsigned i = 0;i < 2;i++) {
        test_now = starts[i];calls = 0;transport_ok = 1;reply = (dht11_reading_t){23.5f,48,true};
        assert(sensors_init());test_now += 1999;assert(clean_read(&s) && calls == 0 && expect_null(&s));
        test_now++;assert(clean_read(&s));
#if SENSOR_ENABLE_DHT11
        assert(calls == 1 && s.dht11_health == SENSOR_HEALTH_OK && s.temperature_c == 23.5f && s.humidity_rh == 48);
        char json[PROTOCOL_BUFFER_SIZE];assert(device_protocol_encode_snapshot(&s,json,sizeof(json)) > 0);
        assert(strstr(json,"\"temp\":23.50") && strstr(json,"\"hum\":48.00"));
        test_now++;assert(clean_read(&s) && calls == 1 && s.dht11_health == SENSOR_HEALTH_OK);
        transport_ok = 0;reply = (dht11_reading_t){99,99,false};test_now += 1999;
        assert(clean_read(&s) && calls == 2 && s.dht11_health == SENSOR_HEALTH_READ_FAILED && expect_null(&s));
        test_now++;assert(clean_read(&s) && calls == 2 && expect_null(&s));
        transport_ok = 1;reply = (dht11_reading_t){21,55,true};test_now += 1999;
        assert(clean_read(&s) && calls == 3 && s.dht11_health == SENSOR_HEALTH_OK && s.temperature_c == 21);
#else
        assert(calls == 0 && s.dht11_health == SENSOR_HEALTH_DISABLED && expect_null(&s));
#endif
        assert(sensors_init() && clean_read(&s) && expect_null(&s));
    }
#if SENSOR_ENABLE_DHT11
    const dht11_reading_t bad[] = {{20,40,false},{NAN,40,true},{20,NAN,true},{INFINITY,40,true},
                                  {20,INFINITY,true},{20,-1,true},{20,101,true}};
    for (unsigned i = 0;i < sizeof(bad)/sizeof(bad[0]);i++) {
        test_now = 0;calls = 0;transport_ok = 1;reply = bad[i];assert(sensors_init());
        test_now = 2000;assert(clean_read(&s) && calls == 1 && s.dht11_health == SENSOR_HEALTH_READ_FAILED && expect_null(&s));
    }
#endif
    assert(!sensors_read(NULL));
    puts("PASS fully initialized snapshots: startup, failure cadence, valid/cache/null protocol, recovery/reinit, wrap and malformed observations.");
    puts("Synthetic optional aggregation only; active Core DHT and hardware remain unchanged.");
    return 0;
}
