/**
 * @file        vision_uart.c
 * @author      Delitor@2022
 * @Version     V1.0
 * @date        20-March-2022
 * @brief       some functions about vision's uart.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "vision_uart.h"

/* private variables ---------------------------------------------------------*/
uint8_t usart1_rxbuf[USART1_RX_BUF_LEN];
uint8_t usart1_dma_txbuf[USART1_TX_BUF_LEN];

/* external variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

/* private functions ---------------------------------------------------------*/
static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	/* Process locked */
	__HAL_LOCK(hdma);
	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Configure the source, destination address and the data length */
		/* Clear DBM bit */
		hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;

		/* Memory to Peripheral */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;
		}
		/* Peripheral to Memory */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}

		/* Enable the Peripheral */
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);

		/* Return error status */
		status = HAL_BUSY;
	} 
	return status; 	
}

/* user functions ------------------------------------------------------------*/
/**
 *	@brief	[__WEAK] 需要在vision_protocol中实现具体的 USART1 处理协议
 */
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf)
{	
}

void Vision_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	// 判断是否为空闲中断
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		/* clear idle it flag avoid idle interrupt all the time */
		__HAL_UART_CLEAR_IDLEFLAG(huart);	
		/* handle received data in idle interrupt */
		if(huart == &huart1)
		{
			USART1_rxDataHandler(usart1_rxbuf);
			memset(usart1_rxbuf, 0, USART1_RX_BUF_LEN);
		}
	}
}

void Vision_UART1_Init(void)
{
	// Enable the UART1 IT
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	// Enable the DMA transfer for the transmit request
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
	DMA_Start(huart1.hdmarx, \
			  (uint32_t)usart1_dma_txbuf, \
			  (uint32_t)&huart1.Instance->DR, \
			  USART1_TX_BUF_LEN);
}
