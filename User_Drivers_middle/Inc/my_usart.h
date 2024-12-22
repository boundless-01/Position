#ifndef _MY_USART_H
#define _MY_USART_H

#include "usart.h"
#include <stdio.h>
#include "stdlib.h"
#include "stdarg.h"
#include	"math.h"
#include "dma.h"
#include "rotate.h"

#define UART_DMA_BUFFER_SIZE		300
#define OPERABLE_BUFFER					0x01
#define BUFFER0_WAIT_SEND				0x02
#define BUFFER1_WAIT_SEND				0x04
#define UART_RX_BUFFER_SIZE			20

//UART
#define USER_UART						(&huart1)
#define DEBUG_UART					(&huart6)

//DMA
#define DMA_USER_UART1_TX		(DMA1_Stream7)
#define DMA_USER_UART6_TX		(DMA1_Stream6)

typedef struct{
	uint8_t buffer0[UART_DMA_BUFFER_SIZE];		//DMA������
	uint8_t buffer1[UART_DMA_BUFFER_SIZE];
	uint16_t buffer0Num;											//�����������ֽ���
	uint16_t buffer1Num;			
	
	uint8_t flag;		//��0λ��ʾ����������ݵĻ���������1λ��ʾbuffer0�Ƿ�����ͣ���2λ��ʾbuffer1�Ƿ������
}UART_DMA_t;

typedef struct{
	uint8_t buffer[UART_RX_BUFFER_SIZE];	//UART���ջ�����
	uint16_t bufferNum;										//�����������ֽ���
}UART_RX_t;

extern UART_DMA_t	UART6_DMA;
extern UART_RX_t	UART_RX_1_6;

void Uart_User_Init(void);
void DataSend(void);
void DeadWhileReport(char* a);
void ReportHardFault(void);
void DebugDataSendByDMA(void);
void AT_CMD_Judge(void);
char *itoa_10(int value, char *string);
void UART6_DMA_Out(const uint8_t *Data , ... );
void USART6DMASendData(uint8_t sendData);
void UART1_DMA_Transmit(uint8_t *transformedData , uint8_t dataNum);
void USART_Out_Float(UART_HandleTypeDef *huart , float value);
void sendUSBData(uint8_t *data, uint32_t length);
#endif
