#ifndef USART_TRAIN_TASK_H
#define USART_TRAIN_TASK_H

#include "main.h"
#include "usart_rc_dbus.h"
#include "usart.h"
#include "function.h"
#include "gimbal.h"
#include "can_receive.h"

typedef struct
{
	//下板接收
//	uint8_t frame_rx_header1;//帧头1
//	int16_t vx_rx;//x轴向速度
//	int16_t vy_rx;//y轴向速度
//	int16_t vw_rx;//角速度
//	uint8_t frame_rx_end;//帧尾
	
	//上板发送
	uint8_t frame_tx_header1;//帧头1
	int16_t vx_tx;//x轴向速度
	int16_t vy_tx;//y轴向速度
	int16_t vw_tx;//角速度
	char s_tx[2];//模式切换
	uint8_t frame_tx_end;//帧尾
}Communicate_measure_t;
extern Communicate_measure_t Send_data;

void USART_485_init(void);
void data_Tx_analysis(Communicate_measure_t *send_data,uint8_t *tx_data);
void data_Rx_analysis(uint8_t *rx_data,Communicate_measure_t *send_data);
void USART4_rx_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void USART_task(void);


#endif



