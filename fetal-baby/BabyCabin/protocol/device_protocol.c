#include "device_protocol.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define DEVICE_COMMAND_MAX_DURATION_MS 300000U
#define JSON_MAX_DEPTH 8U

static const char *status_to_text(baby_status_t status)
{
    switch (status)
    {
    case BABY_STATUS_SLEEPY:
        return "sleepy";
    case BABY_STATUS_HUNGRY:
        return "hungry";
    case BABY_STATUS_WET:
        return "wet";
    case BABY_STATUS_UNCOMFORTABLE:
        return "uncomfortable";
    case BABY_STATUS_ABNORMAL_CRYING:
        return "abnormal_crying";
    case BABY_STATUS_TEMP_ALERT:
        return "temperature_alert";
    case BABY_STATUS_ENV_WARNING:
        return "env_warning";
    case BABY_STATUS_PRESENCE_ALERT:
        return "presence_alert";
    case BABY_STATUS_DATA_UNAVAILABLE:
        return "data_unavailable";
    case BABY_STATUS_SENSOR_FAULT:
        return "sensor_fault";
    case BABY_STATUS_COUNT:
        return "invalid";
    case BABY_STATUS_NORMAL:
    default:
        return "normal";
    }
}

static const char *suggestion_for_status(baby_status_t status)
{
    switch (status)
    {
    case BABY_STATUS_WET:
        return "check_diaper";
    case BABY_STATUS_ABNORMAL_CRYING:
        return "start_white_noise";
    case BABY_STATUS_TEMP_ALERT:
        return "notify_parent";
    case BABY_STATUS_UNCOMFORTABLE:
        return "check_environment";
    case BABY_STATUS_ENV_WARNING:
        return "ventilate_or_adjust_light";
    case BABY_STATUS_PRESENCE_ALERT:
        return "check_baby_in_bed";
    case BABY_STATUS_SLEEPY:
        return "none";
    case BABY_STATUS_HUNGRY:
        return "feed";
    case BABY_STATUS_SENSOR_FAULT:
        return "check_sensors";
    default:
        return "none";
    }
}

static const char *sensor_health_to_text(sensor_health_status_t health)
{
    switch (health)
    {
    case SENSOR_HEALTH_OK:
        return "ok";
    case SENSOR_HEALTH_STALE:
        return "stale";
    case SENSOR_HEALTH_READ_FAILED:
        return "read_failed";
    case SENSOR_HEALTH_DISABLED:
    default:
        return "disabled";
    }
}

typedef struct
{
    const char *cursor;
    const char *end;
} json_reader_t;

static void json_skip_space(json_reader_t *reader)
{
    while (reader->cursor < reader->end &&
           (*reader->cursor == ' ' || *reader->cursor == '\t' ||
            *reader->cursor == '\r' || *reader->cursor == '\n'))
    {
        reader->cursor++;
    }
}

static bool json_parse_string(json_reader_t *reader, const char **text, size_t *length)
{
    const char *start;

    if (reader->cursor >= reader->end || *reader->cursor != '"')
    {
        return false;
    }

    reader->cursor++;
    start = reader->cursor;
    while (reader->cursor < reader->end)
    {
        const unsigned char ch = (unsigned char)*reader->cursor;
        if (ch == '"')
        {
            if (text != NULL) { *text = start; }
            if (length != NULL) { *length = (size_t)(reader->cursor - start); }
            reader->cursor++;
            return true;
        }
        if (ch < 0x20U)
        {
            return false;
        }
        if (ch == '\\')
        {
            reader->cursor++;
            if (reader->cursor >= reader->end ||
                strchr("\"\\/bfnrtu", *reader->cursor) == NULL)
            {
                return false;
            }
            if (*reader->cursor == 'u')
            {
                size_t digit;
                for (digit = 0U; digit < 4U; digit++)
                {
                    reader->cursor++;
                    if (reader->cursor >= reader->end ||
                        !( (*reader->cursor >= '0' && *reader->cursor <= '9') ||
                           (*reader->cursor >= 'a' && *reader->cursor <= 'f') ||
                           (*reader->cursor >= 'A' && *reader->cursor <= 'F') ))
                    {
                        return false;
                    }
                }
            }
        }
        reader->cursor++;
    }
    return false;
}

static bool json_text_equals(const char *text, size_t length, const char *expected)
{
    const size_t expected_length = strlen(expected);
    return length == expected_length && memcmp(text, expected, length) == 0;
}

static bool json_parse_uint(json_reader_t *reader, uint32_t *value)
{
    uint32_t parsed = 0U;
    const char *start = reader->cursor;

    if (start >= reader->end || *start < '0' || *start > '9')
    {
        return false;
    }
    if (*start == '0' && start + 1 < reader->end && start[1] >= '0' && start[1] <= '9')
    {
        return false;
    }
    while (reader->cursor < reader->end &&
           *reader->cursor >= '0' && *reader->cursor <= '9')
    {
        const uint32_t digit = (uint32_t)(*reader->cursor - '0');
        if (parsed > (UINT32_MAX - digit) / 10U)
        {
            return false;
        }
        parsed = parsed * 10U + digit;
        reader->cursor++;
    }
    *value = parsed;
    return true;
}

static bool json_skip_value(json_reader_t *reader, unsigned depth);

static bool json_skip_compound(json_reader_t *reader, char open, char close, unsigned depth)
{
    if (depth >= JSON_MAX_DEPTH || reader->cursor >= reader->end || *reader->cursor != open)
    {
        return false;
    }
    reader->cursor++;
    json_skip_space(reader);
    if (reader->cursor < reader->end && *reader->cursor == close)
    {
        reader->cursor++;
        return true;
    }

    while (reader->cursor < reader->end)
    {
        if (open == '{')
        {
            if (!json_parse_string(reader, NULL, NULL)) { return false; }
            json_skip_space(reader);
            if (reader->cursor >= reader->end || *reader->cursor++ != ':') { return false; }
            json_skip_space(reader);
        }
        if (!json_skip_value(reader, depth + 1U)) { return false; }
        json_skip_space(reader);
        if (reader->cursor < reader->end && *reader->cursor == close)
        {
            reader->cursor++;
            return true;
        }
        if (reader->cursor >= reader->end || *reader->cursor++ != ',') { return false; }
        json_skip_space(reader);
    }
    return false;
}

static bool json_skip_value(json_reader_t *reader, unsigned depth)
{
    const char *literal;
    size_t literal_length;

    json_skip_space(reader);
    if (reader->cursor >= reader->end) { return false; }
    if (*reader->cursor == '"') { return json_parse_string(reader, NULL, NULL); }
    if (*reader->cursor == '{') { return json_skip_compound(reader, '{', '}', depth); }
    if (*reader->cursor == '[') { return json_skip_compound(reader, '[', ']', depth); }

    if (*reader->cursor == 't') { literal = "true"; literal_length = 4U; }
    else if (*reader->cursor == 'f') { literal = "false"; literal_length = 5U; }
    else if (*reader->cursor == 'n') { literal = "null"; literal_length = 4U; }
    else
    {
        const char *start = reader->cursor;
        if (*reader->cursor == '-') { reader->cursor++; }
        if (reader->cursor >= reader->end) { return false; }
        if (*reader->cursor == '0') { reader->cursor++; }
        else
        {
            if (*reader->cursor < '1' || *reader->cursor > '9') { return false; }
            while (reader->cursor < reader->end &&
                   *reader->cursor >= '0' && *reader->cursor <= '9') { reader->cursor++; }
        }
        if (reader->cursor < reader->end && *reader->cursor == '.')
        {
            reader->cursor++;
            if (reader->cursor >= reader->end || *reader->cursor < '0' || *reader->cursor > '9') { return false; }
            while (reader->cursor < reader->end &&
                   *reader->cursor >= '0' && *reader->cursor <= '9') { reader->cursor++; }
        }
        if (reader->cursor < reader->end && (*reader->cursor == 'e' || *reader->cursor == 'E'))
        {
            reader->cursor++;
            if (reader->cursor < reader->end && (*reader->cursor == '+' || *reader->cursor == '-')) { reader->cursor++; }
            if (reader->cursor >= reader->end || *reader->cursor < '0' || *reader->cursor > '9') { return false; }
            while (reader->cursor < reader->end &&
                   *reader->cursor >= '0' && *reader->cursor <= '9') { reader->cursor++; }
        }
        return reader->cursor > start;
    }

    if ((size_t)(reader->end - reader->cursor) < literal_length ||
        memcmp(reader->cursor, literal, literal_length) != 0)
    {
        return false;
    }
    reader->cursor += literal_length;
    return true;
}

static bool command_from_text(const char *text, size_t length,
                              device_command_t *command, uint32_t *default_duration_ms)
{
    struct command_name
    {
        const char *name;
        device_command_t command;
        uint32_t default_duration_ms;
    };
    static const struct command_name names[] = {
        {"start_white_noise", DEVICE_CMD_START_WHITE_NOISE, 180000U},
        {"stop_white_noise", DEVICE_CMD_STOP_WHITE_NOISE, 0U},
        {"start_rocking", DEVICE_CMD_START_ROCKING, 180000U},
        {"stop_rocking", DEVICE_CMD_STOP_ROCKING, 0U},
        {"start_vibration", DEVICE_CMD_START_VIBRATION, 60000U},
        {"stop_vibration", DEVICE_CMD_STOP_VIBRATION, 0U},
        {"play_mother_voice", DEVICE_CMD_PLAY_MOTHER_VOICE, 60000U}
    };
    size_t index;

    for (index = 0U; index < sizeof(names) / sizeof(names[0]); index++)
    {
        if (json_text_equals(text, length, names[index].name))
        {
            *command = names[index].command;
            *default_duration_ms = names[index].default_duration_ms;
            return true;
        }
    }
    return false;
}

static bool collect_command_value(json_reader_t *reader,
                                  device_command_t *commands,
                                  uint32_t *durations_ms,
                                  size_t max_commands,
                                  size_t *count,
                                  unsigned depth);

static bool collect_command_object(json_reader_t *reader,
                                   device_command_t *commands,
                                   uint32_t *durations_ms,
                                   size_t max_commands,
                                   size_t *count,
                                   unsigned depth)
{
    const char *command_text = NULL;
    size_t command_length = 0U;
    uint32_t duration_ms = 0U;
    uint32_t duration_sec = 0U;
    bool command_seen = false;
    bool duration_ms_seen = false;
    bool duration_sec_seen = false;
    bool nested_commands_seen = false;

    if (depth >= JSON_MAX_DEPTH || reader->cursor >= reader->end || *reader->cursor++ != '{')
    {
        return false;
    }
    json_skip_space(reader);
    if (reader->cursor < reader->end && *reader->cursor == '}')
    {
        reader->cursor++;
        return true;
    }

    while (reader->cursor < reader->end)
    {
        const char *key;
        size_t key_length;
        if (!json_parse_string(reader, &key, &key_length)) { return false; }
        json_skip_space(reader);
        if (reader->cursor >= reader->end || *reader->cursor++ != ':') { return false; }
        json_skip_space(reader);

        if (json_text_equals(key, key_length, "command"))
        {
            if (command_seen || !json_parse_string(reader, &command_text, &command_length)) { return false; }
            command_seen = true;
        }
        else if (json_text_equals(key, key_length, "durationMs"))
        {
            if (duration_ms_seen || !json_parse_uint(reader, &duration_ms)) { return false; }
            duration_ms_seen = true;
        }
        else if (json_text_equals(key, key_length, "durationSec"))
        {
            if (duration_sec_seen || !json_parse_uint(reader, &duration_sec)) { return false; }
            duration_sec_seen = true;
        }
        else if (json_text_equals(key, key_length, "commands"))
        {
            if (nested_commands_seen || command_seen ||
                !collect_command_value(reader, commands, durations_ms, max_commands, count, depth + 1U))
            {
                return false;
            }
            nested_commands_seen = true;
        }
        else if (!json_skip_value(reader, depth + 1U))
        {
            return false;
        }

        json_skip_space(reader);
        if (reader->cursor < reader->end && *reader->cursor == '}')
        {
            reader->cursor++;
            break;
        }
        if (reader->cursor >= reader->end || *reader->cursor++ != ',') { return false; }
        json_skip_space(reader);
    }

    if (!command_seen)
    {
        return nested_commands_seen && !duration_ms_seen && !duration_sec_seen;
    }
    if (nested_commands_seen || (duration_ms_seen && duration_sec_seen)) { return false; }

    {
        device_command_t command;
        uint32_t default_duration;
        uint32_t selected_duration;
        if (!command_from_text(command_text, command_length, &command, &default_duration)) { return false; }
        if (default_duration == 0U)
        {
            if (duration_ms_seen || duration_sec_seen) { return false; }
            selected_duration = 0U;
        }
        else if (duration_sec_seen)
        {
            if (duration_sec == 0U || duration_sec > DEVICE_COMMAND_MAX_DURATION_MS / 1000U) { return false; }
            selected_duration = duration_sec * 1000U;
        }
        else
        {
            selected_duration = duration_ms_seen ? duration_ms : default_duration;
            if (selected_duration == 0U || selected_duration > DEVICE_COMMAND_MAX_DURATION_MS) { return false; }
        }
        if (*count >= max_commands) { return false; }
        commands[*count] = command;
        durations_ms[*count] = selected_duration;
        (*count)++;
    }
    return true;
}

static bool collect_command_value(json_reader_t *reader,
                                  device_command_t *commands,
                                  uint32_t *durations_ms,
                                  size_t max_commands,
                                  size_t *count,
                                  unsigned depth)
{
    json_skip_space(reader);
    if (reader->cursor >= reader->end || depth >= JSON_MAX_DEPTH) { return false; }
    if (*reader->cursor == '{')
    {
        return collect_command_object(reader, commands, durations_ms, max_commands, count, depth);
    }
    if (*reader->cursor != '[') { return false; }

    reader->cursor++;
    json_skip_space(reader);
    if (reader->cursor < reader->end && *reader->cursor == ']')
    {
        reader->cursor++;
        return true;
    }
    while (reader->cursor < reader->end)
    {
        if (!collect_command_value(reader, commands, durations_ms, max_commands, count, depth + 1U)) { return false; }
        json_skip_space(reader);
        if (reader->cursor < reader->end && *reader->cursor == ']')
        {
            reader->cursor++;
            return true;
        }
        if (reader->cursor >= reader->end || *reader->cursor++ != ',') { return false; }
        json_skip_space(reader);
    }
    return false;
}

static void format_observation(char *buffer, size_t size, float value, sensor_health_status_t health)
{
    int written;
    if (health != SENSOR_HEALTH_OK || !isfinite(value))
    {
        (void)snprintf(buffer, size, "null");
        return;
    }
    written = snprintf(buffer, size, "%.2f", (double)value);
    if (written < 0 || (size_t)written >= size)
    {
        (void)snprintf(buffer, size, "null");
    }
}

static const char *observed_bool(bool value, sensor_health_status_t health)
{
    return health != SENSOR_HEALTH_OK ? "null" : (value ? "true" : "false");
}

size_t device_protocol_encode_snapshot(const system_snapshot_t *snapshot, char *buffer, size_t buffer_size)
{
    int written;
    char temperature[64];
    char humidity[64];
    char sound[64];
    char object_temperature[64];
    char ambient_temperature[64];
    char battery[8];

    if (snapshot == NULL || buffer == NULL || buffer_size == 0U)
    {
        return 0U;
    }

    format_observation(temperature, sizeof(temperature), snapshot->temperature_c, snapshot->dht11_health);
    format_observation(humidity, sizeof(humidity), snapshot->humidity_rh, snapshot->dht11_health);
    format_observation(sound, sizeof(sound), snapshot->sound_level, snapshot->microphone_health);
    format_observation(object_temperature, sizeof(object_temperature), snapshot->ir_temp_object_c, snapshot->ir_temp_health);
    format_observation(ambient_temperature, sizeof(ambient_temperature), snapshot->ir_temp_ambient_c, snapshot->ir_temp_health);
    if (snapshot->battery_health == SENSOR_HEALTH_OK && snapshot->battery_percent <= 100U)
    {
        (void)snprintf(battery, sizeof(battery), "%u", snapshot->battery_percent);
    }
    else
    {
        (void)snprintf(battery, sizeof(battery), "null");
    }

    written = snprintf(
        buffer,
        buffer_size,
        "{\"schemaVersion\":2,\"status\":\"%s\",\"esp32Online\":%s,\"ts\":%lu,\"temp\":%s,\"hum\":%s,\"sound\":%s,\"motion\":%s,\"wet\":%s,\"battery\":%s,\"cameraDetected\":%s,\"cameraX\":%u,\"cameraY\":%u,\"cameraArea\":%u,\"cameraId\":%d,\"ambientLight\":%u,\"airVOC\":%u,\"airCO2\":%u,\"presence\":%s,\"presenceDistCm\":%u,\"babyInBed\":%s,\"irObjectTemp\":%s,\"irAmbientTemp\":%s,\"sensorHealth\":{\"dht11\":\"%s\",\"mpu6050\":\"%s\",\"microphone\":\"%s\",\"wetProbe\":\"%s\",\"battery\":\"%s\",\"camera\":\"%s\",\"ambientLight\":\"%s\",\"airQuality\":\"%s\",\"presence\":\"%s\",\"irTemp\":\"%s\",\"loadCell\":\"%s\"}}",
        status_to_text(snapshot->status),
        snapshot->esp32_link_online ? "true" : "false",
        (unsigned long)snapshot->timestamp_ms,
        temperature,
        humidity,
        sound,
        observed_bool(snapshot->motion_active, snapshot->mpu6050_health),
        observed_bool(snapshot->wet_detected, snapshot->wet_probe_health),
        battery,
        observed_bool(snapshot->camera_detected, snapshot->camera_health),
        snapshot->camera_x,
        snapshot->camera_y,
        snapshot->camera_area,
        snapshot->camera_id,
        snapshot->ambient_light_lux,
        snapshot->air_quality_voc,
        snapshot->air_quality_co2,
        observed_bool(snapshot->presence_detected, snapshot->presence_health),
        snapshot->presence_distance_cm,
        observed_bool(snapshot->baby_in_bed, snapshot->presence_health),
        object_temperature,
        ambient_temperature,
        sensor_health_to_text(snapshot->dht11_health),
        sensor_health_to_text(snapshot->mpu6050_health),
        sensor_health_to_text(snapshot->microphone_health),
        sensor_health_to_text(snapshot->wet_probe_health),
        sensor_health_to_text(snapshot->battery_health),
        sensor_health_to_text(snapshot->camera_health),
        sensor_health_to_text(snapshot->ambient_light_health),
        sensor_health_to_text(snapshot->air_quality_health),
        sensor_health_to_text(snapshot->presence_health),
        sensor_health_to_text(snapshot->ir_temp_health),
        sensor_health_to_text(snapshot->load_cell_health));

    if (written < 0 || (size_t)written >= buffer_size)
    {
        buffer[0] = '\0';
        return 0U;
    }

    return (size_t)written;
}

size_t device_protocol_encode_alert(baby_status_t status, const system_snapshot_t *snapshot, char *buffer, size_t buffer_size)
{
    int written;

    if (snapshot == NULL || buffer == NULL || buffer_size == 0U)
    {
        return 0U;
    }

    written = snprintf(
        buffer,
        buffer_size,
        "{\"ts\":%lu,\"alert\":\"%s\",\"severity\":\"%s\",\"suggestion\":\"%s\"}",
        (unsigned long)snapshot->timestamp_ms,
        status_to_text(status),
        status == BABY_STATUS_UNCOMFORTABLE ? "warning" : "high",
        suggestion_for_status(status));

    if (written < 0 || (size_t)written >= buffer_size)
    {
        buffer[0] = '\0';
        return 0U;
    }

    return (size_t)written;
}

bool device_protocol_parse_command(const char *payload, device_command_t *command, uint32_t *duration_ms)
{
    return device_protocol_parse_commands(payload, command, duration_ms, 1U) == 1U;
}

size_t device_protocol_parse_commands(
    const char *payload,
    device_command_t *commands,
    uint32_t *duration_ms,
    size_t max_commands)
{
    json_reader_t reader;
    size_t parsed_count = 0U;

    if (payload == NULL || commands == NULL || duration_ms == NULL || max_commands == 0U)
    {
        return 0U;
    }

    reader.cursor = payload;
    reader.end = payload + strlen(payload);
    json_skip_space(&reader);
    if (!collect_command_value(&reader, commands, duration_ms, max_commands, &parsed_count, 0U))
    {
        return 0U;
    }
    json_skip_space(&reader);
    return (reader.cursor == reader.end && parsed_count > 0U) ? parsed_count : 0U;
}
