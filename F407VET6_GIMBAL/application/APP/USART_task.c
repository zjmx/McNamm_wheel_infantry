#include "USART_task.h"

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern INS_t INS;
Communicate_measure_t Send_data;//双板通讯数据打包
uint8_t uart4_tx_data[10];//8个字节，给了16个字节长度，防止DMA传输越界
uint8_t uart4_rx_data[18];//18个字节，给了36个字节长度，防止DMA传输越界

//双板通讯初始化
void USART_485_init(void)
{
	//上下板数据接收初始化
	HAL_UART_Receive_DMA(&huart1,uart4_rx_data,sizeof(uart4_rx_data));
}

//双板通讯函数
//发送vx，vy，和vw
//3,2,0
void USART_task(void)
{
	//数据赋值
	Send_data.frame_tx_header1=0xFF;
//	if(rc_ctrl.rc.s[1]==1)
//	{
//		Send_data.vx_tx=rc_ctrl.rc.ch[2];
//		Send_data.vy_tx=rc_ctrl.rc.ch[3];
//	}
//	Send_data.vw_tx=rc_ctrl.rc.ch[0];
	Send_data.s_tx[0]=rc_ctrl.rc.s[0];
	Send_data.s_tx[1]=rc_ctrl.rc.s[1];
	Send_data.frame_tx_end=0xFE;
	
	data_Tx_analysis(&Send_data,uart4_tx_data);
	HAL_UART_Transmit_DMA(&huart4,uart4_tx_data,sizeof(uart4_tx_data));
//	osDelay(1);
}

//双板通讯数据打包,发送
void data_Tx_analysis(Communicate_measure_t *send_data,uint8_t *tx_data)
{//发送8位数据
	tx_data[0]=(send_data->frame_tx_header1);
	tx_data[1]=(send_data->vx_tx>>8)&0xFF;
	tx_data[2]=(send_data->vx_tx)&0xFF;
	tx_data[3]=(send_data->vy_tx>>8)&0xFF;
	tx_data[4]=(send_data->vy_tx)&0xFF;
	tx_data[5]=(send_data->vw_tx>>8)&0xFF;
	tx_data[6]=(send_data->vw_tx)&0xFF;
	tx_data[7]=(send_data->s_tx[0]);
	tx_data[8]=(send_data->s_tx[1]);
	tx_data[9]=(send_data->frame_tx_end);
}


//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART4)
	{
		HAL_UART_Receive_DMA(&huart4,uart4_rx_data,sizeof(uart4_rx_data));//重新开启DMA接收
	}
}


