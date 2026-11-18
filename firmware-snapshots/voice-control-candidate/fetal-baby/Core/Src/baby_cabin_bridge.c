#include "baby_cabin_bridge.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "baby_care_app.h"
#include "bsp/bsp.h"
#include "inc/app_config.h"
#include "inc/app_types.h"
#include "protocol/device_protocol.h"
#include "services/alert_manager.h"
#include "services/power_manager.h"
#include "services/state_machine.h"

static state_machine_t s_machine;
static system_snapshot_t s_snapshot;
static uint32_t s_last_report_ms;
static char s_report_buffer[PROTOCOL_BUFFER_SIZE];

static void bridge_fill_snapshot(system_snapshot_t *snapshot)
{
    const BabyCare_StatusTypeDef *care = BabyCare_GetStatus();

    (void)memset(snapshot, 0, sizeof(*snapshot));

    snapshot->timestamp_ms = bsp_millis();
    snapshot->dht11_health = SENSOR_HEALTH_DISABLED;
    snapshot->mpu6050_health = SENSOR_HEALTH_DISABLED;
    snapshot->microphone_health = SENSOR_HEALTH_DISABLED;
    snapshot->wet_probe_health = SENSOR_HEALTH_DISABLED;
    snapshot->battery_health = SENSOR_HEALTH_DISABLED;
    snapshot->camera_health = SENSOR_HEALTH_DISABLED;
    snapshot->ambient_light_health = SENSOR_HEALTH_DISABLED;
    snapshot->air_quality_health = SENSOR_HEALTH_DISABLED;
    snapshot->presence_health = SENSOR_HEALTH_DISABLED;
    snapshot->ir_temp_health = SENSOR_HEALTH_DISABLED;
    snapshot->load_cell_health = SENSOR_HEALTH_DISABLED;

    if (care == NULL)
    {
        return;
    }

    snapshot->temperature_c = care->temperature_c;
    snapshot->humidity_rh = care->humidity_percent;
#if BABY_CARE_USE_DHT11
    snapshot->dht11_health = (care->sensor_valid != 0U)
        ? SENSOR_HEALTH_OK
        : SENSOR_HEALTH_READ_FAILED;
    if (care->sensor_sample_count == 0U)
    {
        snapshot->dht11_health = SENSOR_HEALTH_STALE;
    }
    else if (care->sensor_valid != 0U &&
             (snapshot->timestamp_ms - care->sensor_last_success_ms) > BABY_CARE_SENSOR_STALE_MS)
    {
        snapshot->dht11_health = SENSOR_HEALTH_STALE;
    }
#endif
    snapshot->esp32_link_online = (care->esp32_ap_ip_valid != 0U || care->esp32_sta_ip_valid != 0U);
}

void BabyCabinBridge_Init(void)
{
    bsp_init();
    alert_manager_init();
    power_manager_init();
    state_machine_init(&s_machine);
    s_last_report_ms = 0U;
    (void)memset(&s_snapshot, 0, sizeof(s_snapshot));
}

void BabyCabinBridge_Task(void)
{
    baby_status_t status;
    size_t report_len;

    bridge_fill_snapshot(&s_snapshot);
    status = state_machine_update(&s_machine, &s_snapshot);
    s_snapshot.status = status;
    (void)alert_manager_handle_status(status, &s_snapshot);

    if ((s_snapshot.timestamp_ms - s_last_report_ms) < REPORT_PERIOD_MS)
    {
        return;
    }

    s_last_report_ms = s_snapshot.timestamp_ms;
    report_len = device_protocol_encode_snapshot(&s_snapshot, s_report_buffer, sizeof(s_report_buffer));
    if (report_len > 0U && report_len < sizeof(s_report_buffer))
    {
        bsp_send_packet(s_report_buffer);
        s_last_report_ms = s_snapshot.timestamp_ms;
    }
}
