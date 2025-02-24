#include "USART_task.h"

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
Communicate_measure_t Send_data;//˫��ͨѶ���ݴ��
uint8_t uart4_tx_data[18];//18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
//uint8_t uart4_rx_data[2][Commu_FRAME_LENGTH];//18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
uint8_t uart4_rx_data[20];
uint8_t uart4_rx_data_revise[10];

//˫��ͨѶ����
void USART_task(void)
{
	
}

//˫��ͨѶ���ݽ���
void data_Rx_analysis(uint8_t *rx_data,Communicate_measure_t *send_data)
{//����8λ����
	send_data->frame_rx_header1=rx_data[0];
	send_data->vx_rx=((rx_data[1]<<8)|(rx_data[2]));
	send_data->vy_rx=((rx_data[3]<<8)|(rx_data[4]));
	send_data->vw_rx=((rx_data[5]<<8)|(rx_data[6]));
	send_data->s_rx[0]=rx_data[7];
	send_data->s_rx[1]=rx_data[8];
	send_data->frame_rx_end=rx_data[9];
}

//ң������ʼ��
void USART_485_init(void)
{
	HAL_UART_Receive_DMA(&huart4,uart4_rx_data,sizeof(uart4_rx_data));//���¿���DMA����
//	USART_RxDMA_DoubleBuffer_Init(&huart4,uart4_rx_data[0],uart4_rx_data[1],Commu_dobule_FRAME_LENGTH);
//	send_init(uart4_rx_data[0],uart4_rx_data[1],Commu_dobule_FRAME_LENGTH);
}



//�����жϻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART4)
	{
		//�������ݱ�־λ�������ݰ�У׼
		for(int i=0;i<20;i++)
		{
			if((uart4_rx_data[i]==0xFF)&&(uart4_rx_data[i+9]==0xFE))
			{
				for(int j=0;j<10;j++)
				{
					uart4_rx_data_revise[j]=uart4_rx_data[j+i];
				}
			}
		}
		data_Rx_analysis(uart4_rx_data_revise, &Send_data);
		HAL_UART_Receive_DMA(&huart4,uart4_rx_data,sizeof(uart4_rx_data));//���¿���DMA����
	}
}


