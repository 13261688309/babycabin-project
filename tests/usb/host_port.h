#ifndef USB_STACK_HOST_PORT_H
#define USB_STACK_HOST_PORT_H

/* Replace only the MCU port/configuration boundary, not ST's USB core or CDC. */
#define __USBD_CONF__H__
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#define __IO volatile
#define __STATIC_INLINE static inline
#define __ALIGN_BEGIN
#define __ALIGN_END
#define __PACKED __attribute__((packed))
#define UNUSED(value) ((void)(value))
#define USBD_MAX_NUM_INTERFACES 1U
#define USBD_MAX_NUM_CONFIGURATION 1U
#define USBD_MAX_STR_DESC_SIZ 512U
#define USBD_DEBUG_LEVEL 0U
#define USBD_LPM_ENABLED 0U
#define USBD_SELF_POWERED 0U
#define DEVICE_FS 0
#define DEVICE_HS 1
#define USBD_malloc (void *)USBD_static_malloc
#define USBD_free USBD_static_free
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)
typedef struct { uint32_t maxpacket; } HostPcdEndpoint;
typedef struct { HostPcdEndpoint IN_ep[16]; } PCD_HandleTypeDef;
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *memory);

#endif
