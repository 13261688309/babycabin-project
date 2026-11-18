#include "alert_manager.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp/bsp.h"
#include "inc/app_config.h"
#include "protocol/device_protocol.h"

static uint32_t g_last_alert_ms;
static uint32_t g_last_status_alert_ms[BABY_STATUS_COUNT];
static baby_status_t g_last_alerted_status;
static char g_alert_buffer[PROTOCOL_BUFFER_SIZE];

void alert_manager_init(void)
{
    size_t index;

    g_last_alert_ms = 0U;
    g_last_alerted_status = BABY_STATUS_NORMAL;

    for (index = 0U; index < BABY_STATUS_COUNT; index++)
    {
        g_last_status_alert_ms[index] = 0U;
    }
}

static uint32_t reminder_interval_for(baby_status_t status)
{
    switch (status)
    {
    case BABY_STATUS_WET:
        return WET_REMINDER_MS;
    case BABY_STATUS_TEMP_ALERT:
        return TEMP_REMINDER_MS;
    case BABY_STATUS_ABNORMAL_CRYING:
        return ALERT_COOLDOWN_MS;
    case BABY_STATUS_ENV_WARNING:
        return TEMP_REMINDER_MS;
    case BABY_STATUS_PRESENCE_ALERT:
        return ALERT_COOLDOWN_MS;
    default:
        return ALERT_COOLDOWN_MS;
    }
}

bool alert_manager_handle_status(baby_status_t status, const system_snapshot_t *snapshot)
{
    size_t len;
    bool is_new_status;

    if (snapshot == NULL || (unsigned)status >= BABY_STATUS_COUNT)
    {
        return false;
    }

    if (status == BABY_STATUS_NORMAL || status == BABY_STATUS_SLEEPY ||
        status == BABY_STATUS_DATA_UNAVAILABLE)
    {
        g_last_alerted_status = status;
        return false;
    }

    is_new_status = (status != g_last_alerted_status);
    g_last_alerted_status = status;

    if (!is_new_status)
    {
        uint32_t interval = reminder_interval_for(status);

        if ((snapshot->timestamp_ms - g_last_status_alert_ms[status]) < interval)
        {
            return false;
        }
    }

    len = device_protocol_encode_alert(status, snapshot, g_alert_buffer, sizeof(g_alert_buffer));
    if (len == 0U || len >= sizeof(g_alert_buffer))
    {
        return false;
    }

    bsp_send_alert_packet(g_alert_buffer);
    g_last_alert_ms = snapshot->timestamp_ms;
    g_last_status_alert_ms[status] = snapshot->timestamp_ms;
    return true;
}
