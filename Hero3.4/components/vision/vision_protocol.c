/**
 * @file        vision_protocol.c
 * @author      Delitor@2022
 * @Version     V1.0
 * @date        20-March-2022
 * @brief       some functions about vision's protocols.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "vision_protocol.h"

/* external variables ---------------------------------------------------------*/
extern uint8_t usart1_rxbuf[USART1_RX_BUF_LEN];
extern uint8_t usart1_dma_txbuf[USART1_TX_BUF_LEN];
extern fp32 INS_quat[4];
extern fp32 INS_angle[3];
extern bmi088_real_data_t bmi088_real_data;
extern UART_HandleTypeDef huart1;

/* private variables ---------------------------------------------------------*/
struct sensor_data_t _attribute;

void USART1_rxDataHandler(uint8_t *rxBuf)
{
	
}

void sensor_data_update(void)
{
	_attribute.acc[0] = bmi088_real_data.accel[0];
	_attribute.acc[1] = bmi088_real_data.accel[1];
	_attribute.acc[2] = bmi088_real_data.accel[2];
	
	_attribute.gyro[0] = bmi088_real_data.accel[0];
	_attribute.gyro[1] = bmi088_real_data.accel[1];
	_attribute.gyro[2] = bmi088_real_data.accel[2];
	
	_attribute.q[0] = INS_quat[0];
	_attribute.q[1] = INS_quat[1];
	_attribute.q[2] = INS_quat[2];
	_attribute.q[3] = INS_quat[3];
	
	_attribute.e[0] = INS_angle[0] * 180 / PI;
	_attribute.e[1] = INS_angle[1] * 180 / PI;
	_attribute.e[2] = INS_angle[2] * 180 / PI;
	
	_attribute.timestamp_ms = osKernelSysTick();       //osKernelSysTick()获取的即为任务运行时间，单位ms
	
	if (((uint32_t)_attribute.timestamp_ms)%100 == 0)
	{
		_attribute.trigger = 1;
	}
	else 
	{
		_attribute.trigger = 0;
	}
}

/*
	将32位的data拆成4个8位的二进制数，并按高位到低位存入buf数组
*/
void data_split_32to8(uint8_t *buf, uint32_t data)
{
	*buf = (data>>24) & 0xFF;
	*(buf+1) = (data>>16) & 0xFF;
	*(buf+2) = (data>>8) & 0xFF;
	*(buf+3) = data & 0xFF;
}

void sensor_data_encode(void)
{
	data_split_32to8(usart1_dma_txbuf, *((unsigned int*) &_attribute.acc[0]));
	data_split_32to8(usart1_dma_txbuf+4, *((unsigned int*) &_attribute.acc[1]));
	data_split_32to8(usart1_dma_txbuf+8, *((unsigned int*) &_attribute.acc[2]));
	
	data_split_32to8(usart1_dma_txbuf+12, *((unsigned int*) &_attribute.gyro[0]));
	data_split_32to8(usart1_dma_txbuf+16, *((unsigned int*) &_attribute.gyro[1]));
	data_split_32to8(usart1_dma_txbuf+24, *((unsigned int*) &_attribute.gyro[2]));
	
	data_split_32to8(usart1_dma_txbuf+28, *((unsigned int*) &_attribute.q[0]));
	data_split_32to8(usart1_dma_txbuf+32, *((unsigned int*) &_attribute.q[1]));
	data_split_32to8(usart1_dma_txbuf+36, *((unsigned int*) &_attribute.q[2]));
	data_split_32to8(usart1_dma_txbuf+40, *((unsigned int*) &_attribute.q[3]));
	
	data_split_32to8(usart1_dma_txbuf+44, *((unsigned int*) &_attribute.e[0]));
	data_split_32to8(usart1_dma_txbuf+48, *((unsigned int*) &_attribute.e[1]));
	data_split_32to8(usart1_dma_txbuf+52, *((unsigned int*) &_attribute.e[2]));
	
	data_split_32to8(usart1_dma_txbuf+56, *((unsigned int*) &_attribute.timestamp_ms));
	*(usart1_dma_txbuf+60) =  _attribute.trigger;
}

void vision_sent(void)
{
	sensor_data_encode();
	//可能要判断一下上一帧是否传输完成再发送，出问题了再改
	HAL_UART_Transmit_DMA(&huart1, usart1_dma_txbuf, DATA_PACKAGE_SIZE);
}