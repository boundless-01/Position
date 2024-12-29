/**
  ******************************************************************************
  * @file    my_dma.c
  * @author  Falonss
  * @version V1.0.0
  * @date    2023.03.06
  * @brief   ��dma�йصĳ���
  ******************************************************************************
**/
#include "my_dma.h"

/********************************************************
�������ܣ�dma���ڷ�����ɻص�����                
��ڲ�����
����  ֵ��
*********************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_DMA_DISABLE(&hdma_usart1_tx);
	}
	
	if(huart == &huart6)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_tx);
		
		if(UART6_DMA.flag & OPERABLE_BUFFER)		//��ǰCPU�����Ļ�����Ϊbuffer1
		{
			if(UART6_DMA.flag & BUFFER1_WAIT_SEND)
			{
				UART6_DMA.flag &= ~OPERABLE_BUFFER;					//CPU�ɲ����Ĵ�����Ϊbuffer0
				HAL_UART_Transmit_DMA(DEBUG_UART , UART6_DMA.buffer1 , UART6_DMA.buffer1Num);
				UART6_DMA.flag &= ~BUFFER1_WAIT_SEND;
				UART6_DMA.buffer1Num = 0;
			}
		}
		else
		{
			if(UART6_DMA.flag & BUFFER0_WAIT_SEND)
			{
				UART6_DMA.flag |= OPERABLE_BUFFER;					//CPU�ɲ����Ĵ�����Ϊbuffer1
				HAL_UART_Transmit_DMA(DEBUG_UART , UART6_DMA.buffer0 , UART6_DMA.buffer0Num);
				UART6_DMA.flag &= ~BUFFER0_WAIT_SEND;
				UART6_DMA.buffer0Num = 0;
			}
		}
	}
}


