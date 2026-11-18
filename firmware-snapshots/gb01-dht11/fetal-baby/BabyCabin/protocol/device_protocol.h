#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "inc/app_types.h"

size_t device_protocol_encode_snapshot(const system_snapshot_t *snapshot, char *buffer, size_t buffer_size);
size_t device_protocol_encode_alert(baby_status_t status, const system_snapshot_t *snapshot, char *buffer, size_t buffer_size);
bool device_protocol_parse_command(const char *payload, device_command_t *command, uint32_t *duration_ms);
size_t device_protocol_parse_commands(
    const char *payload,
    device_command_t *commands,
    uint32_t *duration_ms,
    size_t max_commands);
