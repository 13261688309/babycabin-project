#pragma once

#include <stdbool.h>

void app_init(void);
void app_task(void);
bool app_handle_command_packet(const char *payload);
