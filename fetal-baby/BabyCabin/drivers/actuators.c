#include "actuators.h"

#include "bsp/bsp.h"
#include "drivers/voice_module.h"
#include "inc/app_config.h"

static uint32_t g_output_deadlines_ms[BSP_OUTPUT_COUNT];

static void set_output_with_deadline(bsp_output_t output, bool enabled, uint32_t duration_ms)
{
    bsp_set_output(output, enabled);

    if (!enabled || duration_ms == 0U)
    {
        g_output_deadlines_ms[output] = 0U;
        return;
    }

    g_output_deadlines_ms[output] = bsp_millis() + duration_ms;
}

static bool deadline_expired(uint32_t now_ms, uint32_t deadline_ms)
{
    return deadline_ms != 0U && ((int32_t)(now_ms - deadline_ms) >= 0);
}

bool actuators_init(void)
{
    actuators_stop_all();
    return true;
}

void actuators_stop_all(void)
{
    set_output_with_deadline(BSP_OUTPUT_WHITE_NOISE, false, 0U);
    set_output_with_deadline(BSP_OUTPUT_MOTHER_VOICE, false, 0U);
    set_output_with_deadline(BSP_OUTPUT_ROCKING, false, 0U);
    set_output_with_deadline(BSP_OUTPUT_VIBRATION, false, 0U);
}

void actuators_execute(device_command_t command, uint32_t duration_ms)
{
    switch (command)
    {
    case DEVICE_CMD_START_WHITE_NOISE:
#if SENSOR_ENABLE_CI1302_VOICE
        if (voice_module_play_command(command))
        {
            break;
        }
#endif
        set_output_with_deadline(BSP_OUTPUT_WHITE_NOISE, true, duration_ms);
        break;
    case DEVICE_CMD_STOP_WHITE_NOISE:
        set_output_with_deadline(BSP_OUTPUT_WHITE_NOISE, false, 0U);
        break;
    case DEVICE_CMD_START_ROCKING:
        set_output_with_deadline(BSP_OUTPUT_ROCKING, true, duration_ms);
        break;
    case DEVICE_CMD_STOP_ROCKING:
        set_output_with_deadline(BSP_OUTPUT_ROCKING, false, 0U);
        break;
    case DEVICE_CMD_START_VIBRATION:
        set_output_with_deadline(BSP_OUTPUT_VIBRATION, true, duration_ms);
        break;
    case DEVICE_CMD_STOP_VIBRATION:
        set_output_with_deadline(BSP_OUTPUT_VIBRATION, false, 0U);
        break;
    case DEVICE_CMD_PLAY_MOTHER_VOICE:
#if SENSOR_ENABLE_CI1302_VOICE
        if (voice_module_play_command(command))
        {
            break;
        }
#endif
        set_output_with_deadline(BSP_OUTPUT_MOTHER_VOICE, true, duration_ms);
        break;
    case DEVICE_CMD_NONE:
    default:
        break;
    }
}

void actuators_task(void)
{
    const uint32_t now_ms = bsp_millis();

    if (deadline_expired(now_ms, g_output_deadlines_ms[BSP_OUTPUT_WHITE_NOISE]))
    {
        set_output_with_deadline(BSP_OUTPUT_WHITE_NOISE, false, 0U);
    }

    if (deadline_expired(now_ms, g_output_deadlines_ms[BSP_OUTPUT_MOTHER_VOICE]))
    {
        set_output_with_deadline(BSP_OUTPUT_MOTHER_VOICE, false, 0U);
    }

    if (deadline_expired(now_ms, g_output_deadlines_ms[BSP_OUTPUT_ROCKING]))
    {
        set_output_with_deadline(BSP_OUTPUT_ROCKING, false, 0U);
    }

    if (deadline_expired(now_ms, g_output_deadlines_ms[BSP_OUTPUT_VIBRATION]))
    {
        set_output_with_deadline(BSP_OUTPUT_VIBRATION, false, 0U);
    }
}
