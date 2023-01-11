#ifndef Vision_PROTOCOL_H
#define Vision_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "string.h"

/* macro ---------------------------------------------------------------------*/
#define USART1_RX_BUF_LEN			200
#define USART1_TX_BUF_LEN			200

/* private functions ---------------------------------------------------------*/
void USART1_rxDataHandler(uint8_t *rxBuf);

#endif