/**
  ******************************************************************************
  * @file    my_usart.c
  * @author  Falonss && Yin Qiyao
  * @version V1.0.0
  * @date    2023.03.06 - 2023.09.16
  * @brief   ����ͨ�Ŵ���
  ******************************************************************************
**/

#include "my_main.h"
#include "my_usart.h"
#include "config.h"
#include "usbd_cdc_if.h"

UART_DMA_t	UART1_DMA;
UART_DMA_t	UART6_DMA;
UART_RX_t		UART1_RX;
UART_RX_t		UART_RX_1_6;

static uint8_t data1;
static uint8_t data6;

uint8_t data55[52] = "AT123456789098765432123456789098765432123456789098ta";
uint16_t len = sizeof(data55) - 1; // ��ȥ null ��β�ַ�
extern int USB_flag;
// ���ʵ��ĵط����÷��ͺ���
void sendUSBData(uint8_t *data, uint32_t length) {
    // �������ݵ� USB �豸
    //CDC_Transmit_HS(data, length);
	if(USB_flag != 1)
	{
		CDC_Transmit_HS(data, length);
	}
}

/********************************************************
�������ܣ����ڳ�ʼ��
��ڲ�����
����  ֵ��
*********************************************************/
void Uart_User_Init(void)
{
    __HAL_UART_ENABLE(USER_UART);
    HAL_UART_Receive_IT(USER_UART, (uint8_t *)&data1, 1);
    __HAL_UART_ENABLE(DEBUG_UART);
    HAL_UART_Receive_IT(DEBUG_UART, (uint8_t *)&data6, 1);
}
/********************************************************
�������ܣ�������ͨ�ţ������巢������
��ڲ�����
����  ֵ��
*********************************************************/
#define DATA_SEND_SIZE 52


void DataSend(void)
{
    uint8_t tdata[DATA_SEND_SIZE];
	
    union
    {
        float   val;
        uint8_t data[4];
    } valSend;

    tdata[0]='A';
    tdata[1]='T';
    tdata[DATA_SEND_SIZE-2]='t';
    tdata[DATA_SEND_SIZE-1]='a';

    UpdateTalkInfo();

    psPara.uPara.sendAngle = psPara.imuPara.imuResult_Angle[2] + psPara.uPara.correctAngle[2];

    if(psPara.uPara.sendAngle > 180.f)
    {
        psPara.uPara.sendAngle -= 360.f;
    }
    else if(psPara.uPara.sendAngle < -180.f)
    {
        psPara.uPara.sendAngle += 360.f;
    }

    valSend.val=(float)psPara.rotatedPosX_robort;
    memcpy(tdata+2,valSend.data,4);
    valSend.val=(float)psPara.rotatedVelX_robort;
    memcpy(tdata+6,valSend.data,4);
    valSend.val=(float)psPara.rotatedPosY_robort;
    memcpy(tdata+10,valSend.data,4);
    valSend.val=(float)psPara.rotatedVelY_robort;
    memcpy(tdata+14,valSend.data,4);
    valSend.val=(float)psPara.imuPara.imuResult_Angle[0];
    memcpy(tdata+18,valSend.data,4);
    valSend.val=(float)psPara.imuPara.imuResult_Angle[1];
    memcpy(tdata+22,valSend.data,4);
    valSend.val=(float)psPara.uPara.sendAngle;
    memcpy(tdata+26,valSend.data,4);
    valSend.val=(float)psPara.imuPara.imuGyro_Real[0];
    memcpy(tdata+30,valSend.data,4);
    valSend.val=(float)psPara.imuPara.imuGyro_Real[2];
    memcpy(tdata+34,valSend.data,4);
    valSend.val=(float)psPara.ePara.encoderSum[0];
    memcpy(tdata+38,valSend.data,4);
    valSend.val=(float)psPara.ePara.encoderSum[1];
    memcpy(tdata+42,valSend.data,4);
    valSend.val=(float)psPara.timeCounter;
    memcpy(tdata+46,valSend.data,4);

#ifdef USART_CHANGE
    UART6_DMA_Out(tdata);
#else
    //UART1_DMA_Transmit(tdata,DATA_SEND_SIZE);
			sendUSBData(tdata,DATA_SEND_SIZE);
			//USB_flag = 0;
#endif
}
/********************************************************
�������ܣ�������ѭ������
��ڲ�����
����  ֵ��
*********************************************************/
void DeadWhileReport(char* a)
{
    char* sendString= "DeadWhileReport";
    uint16_t aLong = sizeof((*a));
    memcpy(sendString+15,a,aLong);
    UART1_DMA_Transmit((uint8_t*)sendString, aLong + 15);
}
/********************************************************
�������ܣ�Ӳ���жϱ���
��ڲ�����
����  ֵ��
*********************************************************/
void ReportHardFault(void)
{
    UART1_DMA_Transmit((uint8_t*)"hardfault\r\n", 11);
}
/********************************************************
�������ܣ�DEBUG����ͨ��
��ڲ�����
����  ֵ��
*********************************************************/
void DebugDataSendByDMA(void)  
{
//    UART6_DMA_Out((uint8_t* )"%f\t%f\t%f\t",psPara.imuPara.imuGyro_Real[0],psPara.imuPara.imuGyro_Real[1],psPara.imuPara.imuGyro_Real[2]);
//    UART6_DMA_Out((uint8_t* )"uart\t%f\t%f\t%f\t%f\t",psPara.rotatedPosX_robort,psPara.rotatedPosY_robort,psPara.rotatedVelX_robort,psPara.rotatedVelY_robort);
//    UART6_DMA_Out((uint8_t* )"uart\t%f\t%f\t%f\t%f\t%f\t%f\t",psPara.imuPara.imuGyro_Real[0],psPara.imuPara.imuGyro_Real[1],psPara.imuPara.imuGyro_Real[2],psPara.imuPara.imuResult_Angle[0],psPara.imuPara.imuResult_Angle[1],psPara.imuPara.imuResult_Angle[2]);
//    UART6_DMA_Out((uint8_t* )"%f,%f\t",psPara.ePara.encoderMileage[0],psPara.ePara.encoderMileage[1]);
//    UART6_DMA_Out((uint8_t* )"%f,%f,%f\t",scha63x_real_data_crc.gyro_x,scha63x_real_data_crc.gyro_y,scha63x_real_data_crc.gyro_z); 
//    UART6_DMA_Out((uint8_t* )"%f\t%f\t",psPara.rotatedPosX_robort,psPara.rotatedPosY_robort); 	// xy
//	UART6_DMA_Out((uint8_t* )"uart\t%f\t",psPara.uPara.sendAngle);
    UART6_DMA_Out((uint8_t* )"uart\t%f\t%f\t",psPara.rotatedPosX_robort,psPara.rotatedPosY_robort);
//    UART6_DMA_Out((uint8_t* )"%f\t%f\t%f\t%f\t%f\t%f\t%f\t",psPara.imuPara.imuGyro_Real[0],psPara.imuPara.imuGyro_Real[1],psPara.imuPara.imuGyro_Real[2],psPara.ePara.posx,psPara.ePara.posy, psPara.ePara.encoderMileage[0], psPara.ePara.encoderMileage[1]);
//    UART6_DMA_Out((uint8_t* )"time\t%f\t",psPara.timeCounter/100.f);
    UART6_DMA_Out((uint8_t* )"\r\n");
}
/********************************************************
�������ܣ�����ָ���ж�
��ڲ�����
����  ֵ��
*********************************************************/
void AT_CMD_Judge(void)
{
//    uint8_t rdata[6];
//	extern int FieldChange;

//    //����������
//    union
//    {
//        float   val;
//        uint8_t data[4];
//    } valSend;

//    //����������
//    union
//    {
//        uint8_t data[4];
//        float value;
//    } convert_u;
//	
//	if((UART_RX_1_6.bufferNum == 4) && strncmp((char*)UART_RX_1_6.buffer, "AB",2) == 0) //including \r\n,buffer number = 16
//    {
//		FieldChange=1;
//		
//    }

//    //AX+[][][][]+\r\n���״﷢��X���꣬�Զ�λϵͳ��X������н���
//    if((UART_RX_1_6.bufferNum == 8) && strncmp((char*)UART_RX_1_6.buffer, "AX", 2)==0)
//    {
//        //����������������
//        convert_u.data[0]=*(UART_RX_1_6.buffer+2);
//        convert_u.data[1]=*(UART_RX_1_6.buffer+3);
//        convert_u.data[2]=*(UART_RX_1_6.buffer+4);
//        convert_u.data[3]=*(UART_RX_1_6.buffer+5);

//        if(!isnan(convert_u.value))									//isnan()����û����
//        {
//            //��У��������и���
//            psPara.uPara.newCorrectX = convert_u.value;				//convert_u.value�������︳ֵ�ģ�
//            psPara.uPara.receiveFlagX = TRUE;

////            //���յ���Ϣ�ط����״������֤
////            valSend.val = convert_u.value;
////            rdata[0]='O';
////            rdata[1] = valSend.data[0];
////            rdata[2] = valSend.data[1];
////            rdata[3] = valSend.data[2];
////            rdata[4] = valSend.data[3];
////            rdata[5]='X';
////            UART1_DMA_Transmit(rdata,6);
//        }
//    }

//    //AY+[][][][]+\r\n���״﷢��Y���꣬�Զ�λϵͳ��Y������н���
//    else if((UART_RX_1_6.bufferNum == 8) && strncmp((char*)UART_RX_1_6.buffer, "AY", 2) == 0)
//    {
//        //����������������
//        convert_u.data[0]=*(UART_RX_1_6.buffer+2);
//        convert_u.data[1]=*(UART_RX_1_6.buffer+3);
//        convert_u.data[2]=*(UART_RX_1_6.buffer+4);
//        convert_u.data[3]=*(UART_RX_1_6.buffer+5);

//        if(!isnan(convert_u.value))
//        {
//            //��У��������и���
//            psPara.uPara.newCorrectY = convert_u.value;
//            psPara.uPara.receiveFlagY = TRUE;

////            //���յ���Ϣ�ط����״������֤
////            valSend.val = convert_u.value;
////            rdata[0]='O';
////            rdata[1] = valSend.data[0];
////            rdata[2] = valSend.data[1];
////            rdata[3] = valSend.data[2];
////            rdata[4] = valSend.data[3];
////            rdata[5]='Y';
////            UART1_DMA_Transmit(rdata,6);
//        }
//    }

//    //AA+[][][][]+\r\n���״﷢�ͽǶȣ��Զ�λϵͳ��ƫ���ǽ��н���
//    else if((UART_RX_1_6.bufferNum == 8) && strncmp((char*)UART_RX_1_6.buffer, "AA", 2) == 0)
//    {
//        //����������������
//        convert_u.data[0]=*(UART_RX_1_6.buffer+2);
//        convert_u.data[1]=*(UART_RX_1_6.buffer+3);
//        convert_u.data[2]=*(UART_RX_1_6.buffer+4);
//        convert_u.data[3]=*(UART_RX_1_6.buffer+5);

//        if(!isnan(convert_u.value))
//        {
//            //��У��������и���
//            psPara.uPara.newCorrectAngle[2] = convert_u.value;
//            psPara.uPara.receiveFlagA = TRUE;

////            //���յ���Ϣ�ط����״������֤
////            valSend.val = convert_u.value;
////            rdata[0]='O';
////            rdata[1] = valSend.data[0];
////            rdata[2] = valSend.data[1];
////            rdata[3] = valSend.data[2];
////            rdata[4] = valSend.data[3];
////            rdata[5]='A';
////            UART1_DMA_Transmit(rdata,6);
//        }
//    }
//    //after receiving ASampleZeroDriftFlag,clear gyroZeroDriftFlag
//    else if((UART_RX_1_6.bufferNum == 4) && strncmp((char*)UART_RX_1_6.buffer, "AR",2) == 0) //including \r\n,buffer number = 16
//    {
//        WORK_OFF;
//        IWDG_Reset();
//    }

//    //�������
//    UART_RX_1_6.bufferNum = 0;
//    memset(UART_RX_1_6.buffer, 0x00, UART_RX_BUFFER_SIZE);
}
/******************************************************
��������ת�ַ�������
char *itoa(int value, char *string, int radix)
radix=10 ��ʾ��10����	��ʮ���ƣ�ת�����Ϊ0;

����d=-379;
ִ��	itoa(d, buf, 10); ��

buf="-379"
**********************************************************/
char *itoa_10(int value, char *string)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';
        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;
        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

}

/*UART6Ϊdebug�����������ַ���������䶯����ȷ����Ҫ�󸡵������������ְ��ַ���ʾ�ͷ���Ч�ʣ��Է�����Ϣ�Ƿ�ÿ֡���������ر��ϸ�*/
void UART6_DMA_Out(const uint8_t *Data, ... )
{
    const char *s;
    float f;
    int d;
    char buf[16];
    char String[20]= {0};
    va_list ap;
    va_start(ap, Data);

    while(*Data != 0)			                //�ж��Ƿ񵽴��ַ���������
    {
        if(*Data == '%')									  //
        {
            switch (*++Data)
            {
            case 's':										  //�ַ���
                s = va_arg(ap, const char *);
                for (; *s; s++)
                {
                    USART6DMASendData(*s);
                }
                Data++;
                break;
            case 'd':										  //ʮ����
                d = va_arg(ap, int);
                itoa_10(d, buf);
                for (s = buf; *s; s++)
                {
                    USART6DMASendData(*s);
                }
                Data++;
                break;

            case 'f':										  //С�������λ
                f = (float)va_arg(ap, double);
                if(f<0.f&&f>-1.f)
                    sprintf( (char*)String, "-%d.%04d", ( int )f, (unsigned int)((fabs(f) - abs((int) f))  * 10000));
                else
                    sprintf( (char*)String, "%d.%04d", ( int )f, (unsigned int)((fabs(f) - abs((int) f))  * 10000));
                for (s=String; *s; s++)
                {
                    USART6DMASendData(*s);
                }
                Data++;
                break;

            default:
                Data++;
                break;
            }
        }
        else
        {
            USART6DMASendData(*Data++);
        }
    }
    va_end(ap);
}


void USART6DMASendData(uint8_t sendData)
{
    if(UART6_DMA.flag & OPERABLE_BUFFER)			//CPU���ڷ��ʵļĴ���λbuffer1
    {
        UART6_DMA.buffer1[UART6_DMA.buffer1Num++] = sendData;
        if(UART6_DMA.buffer1Num >= UART_DMA_BUFFER_SIZE / 2)	//���������������ݳ���һ��
        {
            if(READ_BIT(DMA_USER_UART6_TX->CR, DMA_SxCR_EN) == RESET)			//DMA����
            {
                UART6_DMA.flag &= ~OPERABLE_BUFFER;		//CPU�ɲ�����Ϊbuffer0
                UART6_DMA.flag &= ~BUFFER1_WAIT_SEND;
                HAL_UART_Transmit_DMA(DEBUG_UART, UART6_DMA.buffer1, UART6_DMA.buffer1Num);			//ʹ��DMA����buffer1����
                UART6_DMA.buffer1Num = 0;
            }
            else
            {
                UART6_DMA.flag |= BUFFER1_WAIT_SEND;
            }
            if(UART6_DMA.buffer1Num >= UART_DMA_BUFFER_SIZE)		//���泬����������С
            {
                UART6_DMA.buffer1Num = 0;
                HAL_UART_Abort(DEBUG_UART);
                HAL_UART_Transmit(DEBUG_UART, (uint8_t*)"\r\n UART6 DMA BUFFER1 OVERFLOW\r\n", 31, 10);
            }
        }
    }
    else
    {
        UART6_DMA.buffer0[UART6_DMA.buffer0Num++] = sendData;
        if(UART6_DMA.buffer0Num >= UART_DMA_BUFFER_SIZE / 2)	//���������������ݳ���һ��
        {
            if(READ_BIT(DMA_USER_UART6_TX->CR, DMA_SxCR_EN) == RESET)			//DMA����
            {
                UART6_DMA.flag |= OPERABLE_BUFFER;					//CPU�ɲ����Ĵ�����Ϊbuffer1
                UART6_DMA.flag &= ~BUFFER0_WAIT_SEND;
                HAL_UART_Transmit_DMA(DEBUG_UART, UART6_DMA.buffer0, UART6_DMA.buffer0Num);
                UART6_DMA.buffer0Num = 0;
            }
            else
            {
                UART6_DMA.flag |= BUFFER0_WAIT_SEND;
            }
            if(UART6_DMA.buffer0Num >= UART_DMA_BUFFER_SIZE)	//���泬����������С
            {
                UART6_DMA.buffer0Num = 0;
                HAL_UART_Abort(DEBUG_UART);
                HAL_UART_Transmit(DEBUG_UART, (uint8_t*)"\r\n UART6 DMA BUFFER0 OVERFLOW\r\n", 31, 10);
            }
        }

    }
}

/*UART1Ϊ������ͨѶ�������������巢�ͣ�Ҫ��ÿ֡��������ֹ����������֪�������ݳ��ȣ�bufferNum������ʹ��*/
/*���κε��ж��в�����ʹ��UART1���ڷ�������ֹUART���䵽һ�뱻��ϵ������ؽ��մ���*/
void UART1_DMA_Transmit(uint8_t *transformedData, uint8_t dataNum)
{
    uint16_t dmaCounter = 0;
    if(UART1_DMA.flag & OPERABLE_BUFFER)			//CPU���ڷ��ʵļĴ���λbuffer1
    {
        for(uint8_t i = 0 ; i < dataNum ; i++)
        {
            UART1_DMA.buffer1[i] = transformedData[i];
        }
        UART1_DMA.flag |= BUFFER1_WAIT_SEND;
        while(READ_BIT(DMA_USER_UART1_TX->CR, DMA_SxCR_EN) != RESET)			//DMA���ڴ���
        {
            dmaCounter++;
            if(dmaCounter > 0x7FFF)
            {
                HAL_UART_Abort(DEBUG_UART);
                HAL_UART_Transmit(DEBUG_UART, (uint8_t*)"\r\n UART1 TRANSMIT ERR\r\n", 23, 10);
                break;
            }
        }
        UART1_DMA.flag &= ~OPERABLE_BUFFER;		//CPU�ɲ�����Ϊbuffer0
        UART1_DMA.flag &= ~BUFFER1_WAIT_SEND;
        HAL_UART_Transmit_DMA(USER_UART, UART1_DMA.buffer1, dataNum);			//ʹ��DMA����buffer1����
    }
    else
    {
        for(uint8_t i = 0 ; i < dataNum ; i++)
        {
            UART1_DMA.buffer0[i] = transformedData[i];
        }
        UART1_DMA.flag |= BUFFER0_WAIT_SEND;
        while(READ_BIT(DMA_USER_UART1_TX->CR, DMA_SxCR_EN) != RESET)			//DMA���ڴ���
        {
            dmaCounter++;
            if(dmaCounter > 0x7FFF)
            {
                HAL_UART_Abort(DEBUG_UART);
                HAL_UART_Transmit(DEBUG_UART, (uint8_t*)"\r\n UART1 TRANSMIT ERR\r\n", 23, 10);
                break;
            }
        }
        UART1_DMA.flag |= OPERABLE_BUFFER;		//CPU�ɲ�����Ϊbuffer1
        UART1_DMA.flag &= ~BUFFER0_WAIT_SEND;
        HAL_UART_Transmit_DMA(USER_UART, UART1_DMA.buffer0, dataNum);			//ʹ��DMA����buffer1����
    }
}

/********************************************************
�������ܣ�������������жϻص�����
��ڲ�����
����  ֵ��
*********************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    UNUSED(huart);

    //�����ⲿ�����ж�

    if(huart == USER_UART)
    {
//        UART_RX_1_6.buffer[UART_RX_1_6.bufferNum++] = data1;
//        if(UART_RX_1_6.bufferNum >= UART_RX_BUFFER_SIZE)
//        {
//            UART_RX_1_6.bufferNum = 0;
//            memset(UART_RX_1_6.buffer, 0x00, UART_RX_BUFFER_SIZE);
//        }
//        if( (UART_RX_1_6.bufferNum > 1) && (UART_RX_1_6.buffer[UART_RX_1_6.bufferNum - 1] == '\n') && (UART_RX_1_6.buffer[UART_RX_1_6.bufferNum - 2] == '\r') )		//���ս�����־
//        {
//            AT_CMD_Judge();
//        }
//        else
//        {   if(UART_RX_1_6.buffer[0] != 'A')
//            {
//                UART_RX_1_6.bufferNum = 0;
//                for(int i = 0 ; i < UART_RX_BUFFER_SIZE ; i++)
//                {
//                    UART_RX_1_6.buffer[i] = 0;
//                }
//            }
//        }
//        HAL_UART_Receive_IT(USER_UART, (uint8_t *)&data1, 1);
//    }

//    if(huart == DEBUG_UART)
//    {
//        UART_RX_1_6.buffer[UART_RX_1_6.bufferNum++] = data6;
//        if(UART_RX_1_6.bufferNum >= UART_RX_BUFFER_SIZE)
//        {
//            UART_RX_1_6.bufferNum = 0;
//            memset(UART_RX_1_6.buffer, 0x00, UART_RX_BUFFER_SIZE);
//        }
//        if( (UART_RX_1_6.bufferNum > 1) && (UART_RX_1_6.buffer[UART_RX_1_6.bufferNum - 1] == '\n') && (UART_RX_1_6.buffer[UART_RX_1_6.bufferNum - 2] == '\r') )		//���ս�����־
//        {
//            AT_CMD_Judge();
//        }
//        else
//        {
//            if(UART_RX_1_6.buffer[0] != 'A')
//            {
//                UART_RX_1_6.bufferNum = 0;
//                for(int i = 0 ; i < UART_RX_BUFFER_SIZE ; i++)
//                {
//                    UART_RX_1_6.buffer[i] = 0;
//                }
//            }
//        }
//        HAL_UART_Receive_IT(DEBUG_UART, (uint8_t *)&data6, 1);
    }
}

