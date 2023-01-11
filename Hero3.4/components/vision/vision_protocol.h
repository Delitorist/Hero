#ifndef Vision_PROTOCOL_H
#define Vision_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "bsp_imu_pwm.h"
#include "bmi088driver.h"
#include "cmsis_os.h"
#include "arm_math.h"

/* macro ---------------------------------------------------------------------*/
#define USART1_TX_BUF_LEN			200
#define USART1_RX_BUF_LEN			200
#define DATA_PACKAGE_SIZE     61

struct sensor_data_t {
	float acc[3];          //������ٶ�[��λ��g]
	float gyro[3];         //������ٶ�[��λ��dps]
	float q[4];            //��Ԫ����̬
	float e[3];            //ŷ������̬[��λ��degree]
	float timestamp_ms;
	uint8_t trigger;       //0:����֡��1:����֡
};

/* private functions ---------------------------------------------------------*/
void USART1_rxDataHandler(uint8_t *rxBuf);
void vision_sent(void);
void sensor_data_update(void);
void sensor_data_encode(void);
void data_split_32to8(uint8_t *buf, uint32_t data);

#endif