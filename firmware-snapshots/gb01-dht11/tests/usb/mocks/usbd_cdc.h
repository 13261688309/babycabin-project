#ifndef USB_OBSERVER_TEST_CDC_H
#define USB_OBSERVER_TEST_CDC_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define USBD_STATE_CONFIGURED 3U
#define CDC_SET_LINE_CODING 0x20U
#define CDC_GET_LINE_CODING 0x21U
#define CDC_SET_CONTROL_LINE_STATE 0x22U
#define CDC_REQ_MAX_DATA_SIZE 7U
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define UNUSED(value) ((void)(value))
typedef struct { uint32_t TxState; uint8_t *TxBuffer; uint32_t TxLength; } USBD_CDC_HandleTypeDef;
typedef struct { uint8_t dev_state; void *pClassData; } USBD_HandleTypeDef;
typedef struct { uint8_t bmRequest, bRequest; uint16_t wValue, wIndex, wLength; } USBD_SetupReqTypedef;
typedef struct {
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Control)(uint8_t, uint8_t *, uint16_t);
    int8_t (*Receive)(uint8_t *, uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *, uint32_t *, uint8_t);
} USBD_CDC_ItfTypeDef;
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *, uint8_t *, uint32_t);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *);
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *, uint8_t *);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *);
#endif
