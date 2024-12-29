/**
  ******************************************************************************
  * @file    my_dma.c
  * @author  Falonss
  * @version V1.0.0
  * @date    2023.03.06
  * @brief   与dma有关的程序
  ******************************************************************************
**/
#include "my_dma.h"

/********************************************************
函数功能：dma串口发送完成回调函数                
入口参数：
返回  值：
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
		
		if(UART6_DMA.flag & OPERABLE_BUFFER)		//当前CPU操作的缓冲区为buffer1
		{
			if(UART6_DMA.flag & BUFFER1_WAIT_SEND)
			{
				UART6_DMA.flag &= ~OPERABLE_BUFFER;					//CPU可操作寄存器改为buffer0
				HAL_UART_Transmit_DMA(DEBUG_UART , UART6_DMA.buffer1 , UART6_DMA.buffer1Num);
				UART6_DMA.flag &= ~BUFFER1_WAIT_SEND;
				UART6_DMA.buffer1Num = 0;
			}
		}
		else
		{
			if(UART6_DMA.flag & BUFFER0_WAIT_SEND)
			{
				UART6_DMA.flag |= OPERABLE_BUFFER;					//CPU可操作寄存器改为buffer1
				HAL_UART_Transmit_DMA(DEBUG_UART , UART6_DMA.buffer0 , UART6_DMA.buffer0Num);
				UART6_DMA.flag &= ~BUFFER0_WAIT_SEND;
				UART6_DMA.buffer0Num = 0;
			}
		}
	}
}


