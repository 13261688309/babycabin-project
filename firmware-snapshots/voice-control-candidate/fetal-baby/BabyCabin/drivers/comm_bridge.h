#pragma once

#include <stdbool.h>
#include <stddef.h>

#define COMM_BRIDGE_COMMAND_POLL "GET /api/commands"
#define COMM_BRIDGE_REPORT_PREFIX "POST /api/device/report "
#define COMM_BRIDGE_ALERT_PREFIX "POST /api/device/alert "

size_t comm_bridge_build_report_line(const char *json, char *buffer, size_t buffer_size);
size_t comm_bridge_build_alert_line(const char *json, char *buffer, size_t buffer_size);
size_t comm_bridge_build_command_poll_line(char *buffer, size_t buffer_size);
bool comm_bridge_extract_json(const char *line, char *buffer, size_t buffer_size);
