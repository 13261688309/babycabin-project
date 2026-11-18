#ifndef VISION_UPLINK_H
#define VISION_UPLINK_H

#include <stdint.h>
#include <stddef.h>

#define VISION_UPLINK_FRAME_SIZE 64U
#define VISION_UPLINK_TTL_MS 1500U
#define VISION_UPLINK_MODEL_SHA256 "2adf7e59db4a3263b209fb43f955192cd2fddae08539af04c179c9f4e44085e7"

typedef struct {
    uint32_t host, sequence, confidence, label, age_ms, fresh;
    uint32_t accepted, rejected, drops, alerts;
} VisionUplink_Snapshot;

/* USB callbacks only copy bounded chunks. Parsing and alerts run in foreground. */
void VisionUplink_Init(void);
void VisionUplink_OnSessionReset(void);
void VisionUplink_OnBytes(const uint8_t *data, uint32_t length);
void VisionUplink_Task(void);
void VisionUplink_GetSnapshot(VisionUplink_Snapshot *snapshot);
uint32_t VisionUplink_AdvertiseTick(void);
uint8_t VisionUplink_TakeCryAlert(uint32_t *sequence);

#endif
