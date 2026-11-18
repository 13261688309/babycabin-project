#include "app.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp/bsp.h"
#include "drivers/actuators.h"
#include "drivers/sensors.h"
#include "drivers/voice_module.h"
#include "inc/app_config.h"
#include "protocol/device_protocol.h"
#include "services/alert_manager.h"
#include "services/power_manager.h"
#include "services/state_machine.h"

static state_machine_t g_state_machine;
static system_snapshot_t g_snapshot;
static uint32_t g_last_report_ms;
static uint32_t g_last_command_poll_ms;
static bool g_auto_soothe_active;
static char g_report_buffer[PROTOCOL_BUFFER_SIZE];
static char g_command_buffer[COMMAND_BUFFER_SIZE];
static device_command_t g_polled_commands[MAX_COMMANDS_PER_POLL];
static uint32_t g_polled_command_durations_ms[MAX_COMMANDS_PER_POLL];

static void app_poll_voice(void)
{
    voice_event_t event;
    device_command_t command;
    uint32_t duration_ms;

    if (!voice_module_poll_event(&event))
    {
        return;
    }

    if (voice_module_event_to_command(&event, &command, &duration_ms))
    {
        actuators_execute(command, duration_ms);
    }
}

static void app_poll_commands(uint32_t now_ms)
{
    size_t command_count;
    size_t index;

    if ((now_ms - g_last_command_poll_ms) < COMMAND_POLL_PERIOD_MS)
    {
        return;
    }

    g_last_command_poll_ms = now_ms;

    if (!bsp_poll_command_packet(g_command_buffer, sizeof(g_command_buffer)))
    {
        return;
    }

    command_count = device_protocol_parse_commands(
        g_command_buffer,
        g_polled_commands,
        g_polled_command_durations_ms,
        MAX_COMMANDS_PER_POLL);

    for (index = 0U; index < command_count; index++)
    {
        actuators_execute(g_polled_commands[index], g_polled_command_durations_ms[index]);
    }
}

void app_init(void)
{
    (void)sensors_init();
    (void)actuators_init();
    (void)voice_module_init();
    alert_manager_init();
    power_manager_init();
    state_machine_init(&g_state_machine);
    g_last_report_ms = 0U;
    g_last_command_poll_ms = 0U;
    g_auto_soothe_active = false;
}

void app_task(void)
{
    baby_status_t status;
    size_t report_len;
    uint32_t now_ms = bsp_millis();

    app_poll_voice();
    app_poll_commands(now_ms);

    if (!sensors_read(&g_snapshot))
    {
        actuators_task();
        return;
    }

    g_snapshot.timestamp_ms = now_ms;
    status = state_machine_update(&g_state_machine, &g_snapshot);

    if (status == BABY_STATUS_ABNORMAL_CRYING && !g_auto_soothe_active)
    {
        actuators_execute(DEVICE_CMD_START_WHITE_NOISE, AUTO_SOOTHE_WHITE_NOISE_MS);
        g_auto_soothe_active = true;
    }
    else if (status != BABY_STATUS_ABNORMAL_CRYING && g_auto_soothe_active)
    {
        actuators_execute(DEVICE_CMD_STOP_WHITE_NOISE, 0U);
        g_auto_soothe_active = false;
    }

    (void)alert_manager_handle_status(status, &g_snapshot);

    if ((g_snapshot.timestamp_ms - g_last_report_ms) >= REPORT_PERIOD_MS)
    {
        report_len = device_protocol_encode_snapshot(&g_snapshot, g_report_buffer, sizeof(g_report_buffer));
        if (report_len > 0U && report_len < sizeof(g_report_buffer))
        {
            bsp_send_packet(g_report_buffer);
            g_last_report_ms = g_snapshot.timestamp_ms;
        }
    }

    if (power_manager_should_sleep(&g_snapshot))
    {
        actuators_stop_all();
    }

    actuators_task();
}

bool app_handle_command_packet(const char *payload)
{
    device_command_t command;
    uint32_t duration_ms;

    if (!device_protocol_parse_command(payload, &command, &duration_ms))
    {
        return false;
    }

    actuators_execute(command, duration_ms);
    return true;
}
