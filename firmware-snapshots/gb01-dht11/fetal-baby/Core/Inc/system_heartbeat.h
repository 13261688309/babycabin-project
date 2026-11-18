#ifndef SYSTEM_HEARTBEAT_H
#define SYSTEM_HEARTBEAT_H

void SystemHeartbeat_Init(void);
/* Call only after the complete foreground task; never from an interrupt. */
void SystemHeartbeat_Service(void);

#endif
