#include "voice_module.h"

#include <stddef.h>

#include "bsp/bsp.h"
#include "inc/app_config.h"

#if SENSOR_ENABLE_CI1302_VOICE
static uint8_t g_voice_rx_buffer[CI1302_RX_BUFFER_SIZE];
#endif

static bool ci1302_send_command(uint8_t command_id)
{
#if SENSOR_ENABLE_CI1302_VOICE
    uint8_t frame[5];

    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    frame[2] = 0xFFU;
    frame[3] = command_id;
    frame[4] = 0xFBU;
    return bsp_voice_send(frame, sizeof(frame));
#else
    (void)command_id;
    return false;
#endif
}

bool voice_module_init(void)
{
    return true;
}

bool voice_module_play_command(device_command_t command)
{
    switch (command)
    {
    case DEVICE_CMD_START_WHITE_NOISE:
        return ci1302_send_command(CI1302_CMD_WHITE_NOISE);
    case DEVICE_CMD_PLAY_MOTHER_VOICE:
        return ci1302_send_command(CI1302_CMD_MOTHER_VOICE);
    default:
        return false;
    }
}

bool voice_module_poll_event(voice_event_t *event)
{
#if SENSOR_ENABLE_CI1302_VOICE
    size_t received_len = 0U;
#endif

    if (event == NULL)
    {
        return false;
    }

    event->type = VOICE_EVENT_NONE;
    event->command_id = 0U;
    event->valid = false;

#if SENSOR_ENABLE_CI1302_VOICE
    if (!bsp_voice_receive(g_voice_rx_buffer, sizeof(g_voice_rx_buffer), &received_len) || received_len == 0U)
    {
        return false;
    }

    event->type = VOICE_EVENT_UNKNOWN;
    event->command_id = g_voice_rx_buffer[received_len - 1U];
    event->valid = true;
    return true;
#else
    return false;
#endif
}

bool voice_module_event_to_command(const voice_event_t *event, device_command_t *command, uint32_t *duration_ms)
{
    if (event == NULL || command == NULL || duration_ms == NULL || !event->valid)
    {
        return false;
    }

    *duration_ms = CI1302_VOICE_COMMAND_DURATION_MS;

    switch (event->command_id)
    {
    case CI1302_EVENT_WHITE_NOISE:
        *command = DEVICE_CMD_START_WHITE_NOISE;
        return true;
    case CI1302_EVENT_MOTHER_VOICE:
        *command = DEVICE_CMD_PLAY_MOTHER_VOICE;
        return true;
    case CI1302_EVENT_ROCKING:
        *command = DEVICE_CMD_START_ROCKING;
        return true;
    case CI1302_EVENT_VIBRATION:
        *command = DEVICE_CMD_START_VIBRATION;
        return true;
    default:
        *command = DEVICE_CMD_NONE;
        *duration_ms = 0U;
        return false;
    }
}
