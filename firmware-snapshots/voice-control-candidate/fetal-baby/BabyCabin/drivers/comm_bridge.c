#include "comm_bridge.h"

#include <string.h>

static size_t copy_text(char *buffer, size_t buffer_size, const char *text)
{
    size_t index = 0U;

    if (buffer == NULL || buffer_size == 0U || text == NULL)
    {
        return 0U;
    }

    while (text[index] != '\0' && index < (buffer_size - 1U))
    {
        buffer[index] = text[index];
        index++;
    }

    buffer[index] = '\0';
    return text[index] == '\0' ? index : 0U;
}

static size_t build_prefixed_line(const char *prefix, const char *json, char *buffer, size_t buffer_size)
{
    const size_t prefix_len = prefix == NULL ? 0U : strlen(prefix);
    const size_t json_len = json == NULL ? 0U : strlen(json);

    if (prefix == NULL || json == NULL || buffer == NULL || buffer_size == 0U)
    {
        return 0U;
    }

    if ((prefix_len + json_len + 1U) > buffer_size)
    {
        buffer[0] = '\0';
        return 0U;
    }

    (void)memcpy(buffer, prefix, prefix_len);
    (void)memcpy(buffer + prefix_len, json, json_len);
    buffer[prefix_len + json_len] = '\0';
    return prefix_len + json_len;
}

size_t comm_bridge_build_report_line(const char *json, char *buffer, size_t buffer_size)
{
    return build_prefixed_line(COMM_BRIDGE_REPORT_PREFIX, json, buffer, buffer_size);
}

size_t comm_bridge_build_alert_line(const char *json, char *buffer, size_t buffer_size)
{
    return build_prefixed_line(COMM_BRIDGE_ALERT_PREFIX, json, buffer, buffer_size);
}

size_t comm_bridge_build_command_poll_line(char *buffer, size_t buffer_size)
{
    return copy_text(buffer, buffer_size, COMM_BRIDGE_COMMAND_POLL);
}

bool comm_bridge_extract_json(const char *line, char *buffer, size_t buffer_size)
{
    const char *json_start = line;
    size_t index = 0U;

    if (line == NULL || buffer == NULL || buffer_size == 0U)
    {
        return false;
    }

    while (*json_start != '\0' && *json_start != '{' && *json_start != '[')
    {
        json_start++;
    }

    if (*json_start == '\0')
    {
        buffer[0] = '\0';
        return false;
    }

    while (json_start[index] != '\0' && json_start[index] != '\r' && json_start[index] != '\n' && index < (buffer_size - 1U))
    {
        buffer[index] = json_start[index];
        index++;
    }

    buffer[index] = '\0';
    return index > 0U && (json_start[index] == '\0' || json_start[index] == '\r' || json_start[index] == '\n');
}
