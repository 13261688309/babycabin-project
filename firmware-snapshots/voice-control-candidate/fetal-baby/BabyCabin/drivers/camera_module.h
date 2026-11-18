#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAMERA_AI_MODE_NORMAL = 0,
    CAMERA_AI_MODE_CAT_DOG,
    CAMERA_AI_MODE_FACE_DETECTION,
    CAMERA_AI_MODE_COLOR_IDENTIFY,
    CAMERA_AI_MODE_FACE_IDENTIFY,
    CAMERA_AI_MODE_QR_CODE
} camera_ai_mode_t;

typedef struct
{
    uint16_t middle_x;
    uint16_t middle_y;
    int16_t id;
    uint16_t area;
    bool valid;
} camera_detection_t;

bool camera_module_init(void);
bool camera_module_set_mode(camera_ai_mode_t mode);
bool camera_module_read_detection(camera_detection_t *detection);

