/**
  ******************************************************************************
  * @file    my_usart.c
  * @author  Falonss && Yin Qiyao
  * @version V1.0.0
  * @date    2023.03.06 - 2023.09.16
  * @brief   串口通信代码
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
uint16_t len = sizeof(data55) - 1; // 减去 null 结尾字符
extern int USB_flag;
// 在适当的地方调用发送函数
void sendUSBData(uint8_t *data, uint32_t length) {
    // 发送数据到 USB 设备
    //CDC_Transmit_HS(data, length);
	if(USB_flag != 1)
	{
		CDC_Transmit_HS(data, length);
	}
}

/********************************************************
函数功能：串口初始化
入口参数：
返回  值：
*********************************************************/
void Uart_User_Init(void)
{
    __HAL_UART_ENABLE(USER_UART);
    HAL_UART_Receive_IT(USER_UART, (uint8_t *)&data1, 1);
    __HAL_UART_ENABLE(DEBUG_UART);
    HAL_UART_Receive_IT(DEBUG_UART, (uint8_t *)&data6, 1);
}
/********************************************************
函数功能：与主控通信：联合体发送数据
入口参数：
返回  值：
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
函数功能：程序死循环报错
入口参数：
返回  值：
*********************************************************/
void DeadWhileReport(char* a)
{
    char* sendString= "DeadWhileReport";
    uint16_t aLong = sizeof((*a));
    memcpy(sendString+15,a,aLong);
    UART1_DMA_Transmit((uint8_t*)sendString, aLong + 15);
}
/********************************************************
函数功能：硬件中断报错
入口参数：
返回  值：
*********************************************************/
void ReportHardFault(void)
{
    UART1_DMA_Transmit((uint8_t*)"hardfault\r\n", 11);
}
/********************************************************
函数功能：DEBUG串口通信
入口参数：
返回  值：
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
函数功能：串口指令判断
入口参数：
返回  值：
*********************************************************/
void AT_CMD_Judge(void)
{
//    uint8_t rdata[6];
//	extern int FieldChange;

//    //发数联合体
//    union
//    {
//        float   val;
//        uint8_t data[4];
//    } valSend;

//    //收数联合体
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

//    //AX+[][][][]+\r\n即雷达发送X坐标，对定位系统的X坐标进行矫正
//    if((UART_RX_1_6.bufferNum == 8) && strncmp((char*)UART_RX_1_6.buffer, "AX", 2)==0)
//    {
//        //浮点数联合体收数
//        convert_u.data[0]=*(UART_RX_1_6.buffer+2);
//        convert_u.data[1]=*(UART_RX_1_6.buffer+3);
//        convert_u.data[2]=*(UART_RX_1_6.buffer+4);
//        convert_u.data[3]=*(UART_RX_1_6.buffer+5);

//        if(!isnan(convert_u.value))									//isnan()定义没明白
//        {
//            //将校正坐标进行更改
//            psPara.uPara.newCorrectX = convert_u.value;				//convert_u.value是在哪里赋值的？
//            psPara.uPara.receiveFlagX = TRUE;

////            //将收到信息重发给雷达，进行验证
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

//    //AY+[][][][]+\r\n即雷达发送Y坐标，对定位系统的Y坐标进行矫正
//    else if((UART_RX_1_6.bufferNum == 8) && strncmp((char*)UART_RX_1_6.buffer, "AY", 2) == 0)
//    {
//        //浮点数联合体收数
//        convert_u.data[0]=*(UART_RX_1_6.buffer+2);
//        convert_u.data[1]=*(UART_RX_1_6.buffer+3);
//        convert_u.data[2]=*(UART_RX_1_6.buffer+4);
//        convert_u.data[3]=*(UART_RX_1_6.buffer+5);

//        if(!isnan(convert_u.value))
//        {
//            //将校正坐标进行更改
//            psPara.uPara.newCorrectY = convert_u.value;
//            psPara.uPara.receiveFlagY = TRUE;

////            //将收到信息重发给雷达，进行验证
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

//    //AA+[][][][]+\r\n即雷达发送角度，对定位系统的偏航角进行矫正
//    else if((UART_RX_1_6.bufferNum == 8) && strncmp((char*)UART_RX_1_6.buffer, "AA", 2) == 0)
//    {
//        //浮点数联合体收数
//        convert_u.data[0]=*(UART_RX_1_6.buffer+2);
//        convert_u.data[1]=*(UART_RX_1_6.buffer+3);
//        convert_u.data[2]=*(UART_RX_1_6.buffer+4);
//        convert_u.data[3]=*(UART_RX_1_6.buffer+5);

//        if(!isnan(convert_u.value))
//        {
//            //将校正坐标进行更改
//            psPara.uPara.newCorrectAngle[2] = convert_u.value;
//            psPara.uPara.receiveFlagA = TRUE;

////            //将收到信息重发给雷达，进行验证
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

//    //清空数组
//    UART_RX_1_6.bufferNum = 0;
//    memset(UART_RX_1_6.buffer, 0x00, UART_RX_BUFFER_SIZE);
}
/******************************************************
整形数据转字符串函数
char *itoa(int value, char *string, int radix)
radix=10 标示是10进制	非十进制，转换结果为0;

例：d=-379;
执行	itoa(d, buf, 10); 后

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

/*UART6为debug发数，发数字符数随情况变动，不确定，要求浮点数和整形数字按字符显示和发送效率，对发送信息是否每帧完整不是特别严格*/
void UART6_DMA_Out(const uint8_t *Data, ... )
{
    const char *s;
    float f;
    int d;
    char buf[16];
    char String[20]= {0};
    va_list ap;
    va_start(ap, Data);

    while(*Data != 0)			                //判断是否到达字符串结束符
    {
        if(*Data == '%')									  //
        {
            switch (*++Data)
            {
            case 's':										  //字符串
                s = va_arg(ap, const char *);
                for (; *s; s++)
                {
                    USART6DMASendData(*s);
                }
                Data++;
                break;
            case 'd':										  //十进制
                d = va_arg(ap, int);
                itoa_10(d, buf);
                for (s = buf; *s; s++)
                {
                    USART6DMASendData(*s);
                }
                Data++;
                break;

            case 'f':										  //小数点后四位
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
    if(UART6_DMA.flag & OPERABLE_BUFFER)			//CPU正在访问的寄存器位buffer1
    {
        UART6_DMA.buffer1[UART6_DMA.buffer1Num++] = sendData;
        if(UART6_DMA.buffer1Num >= UART_DMA_BUFFER_SIZE / 2)	//若缓冲区储存数据超过一半
        {
            if(READ_BIT(DMA_USER_UART6_TX->CR, DMA_SxCR_EN) == RESET)			//DMA空闲
            {
                UART6_DMA.flag &= ~OPERABLE_BUFFER;		//CPU可操作改为buffer0
                UART6_DMA.flag &= ~BUFFER1_WAIT_SEND;
                HAL_UART_Transmit_DMA(DEBUG_UART, UART6_DMA.buffer1, UART6_DMA.buffer1Num);			//使能DMA发送buffer1数据
                UART6_DMA.buffer1Num = 0;
            }
            else
            {
                UART6_DMA.flag |= BUFFER1_WAIT_SEND;
            }
            if(UART6_DMA.buffer1Num >= UART_DMA_BUFFER_SIZE)		//储存超过缓冲区大小
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
        if(UART6_DMA.buffer0Num >= UART_DMA_BUFFER_SIZE / 2)	//若缓冲区储存数据超过一半
        {
            if(READ_BIT(DMA_USER_UART6_TX->CR, DMA_SxCR_EN) == RESET)			//DMA空闲
            {
                UART6_DMA.flag |= OPERABLE_BUFFER;					//CPU可操作寄存器改为buffer1
                UART6_DMA.flag &= ~BUFFER0_WAIT_SEND;
                HAL_UART_Transmit_DMA(DEBUG_UART, UART6_DMA.buffer0, UART6_DMA.buffer0Num);
                UART6_DMA.buffer0Num = 0;
            }
            else
            {
                UART6_DMA.flag |= BUFFER0_WAIT_SEND;
            }
            if(UART6_DMA.buffer0Num >= UART_DMA_BUFFER_SIZE)	//储存超过缓冲区大小
            {
                UART6_DMA.buffer0Num = 0;
                HAL_UART_Abort(DEBUG_UART);
                HAL_UART_Transmit(DEBUG_UART, (uint8_t*)"\r\n UART6 DMA BUFFER0 OVERFLOW\r\n", 31, 10);
            }
        }

    }
}

/*UART1为与主控通讯，发数以联合体发送，要求每帧的完整防止解析出错，已知发数数据长度，bufferNum变量不使用*/
/*在任何的中断中不允许使用UART1串口发数，防止UART传输到一半被打断导致主控接收错误*/
void UART1_DMA_Transmit(uint8_t *transformedData, uint8_t dataNum)
{
    uint16_t dmaCounter = 0;
    if(UART1_DMA.flag & OPERABLE_BUFFER)			//CPU正在访问的寄存器位buffer1
    {
        for(uint8_t i = 0 ; i < dataNum ; i++)
        {
            UART1_DMA.buffer1[i] = transformedData[i];
        }
        UART1_DMA.flag |= BUFFER1_WAIT_SEND;
        while(READ_BIT(DMA_USER_UART1_TX->CR, DMA_SxCR_EN) != RESET)			//DMA正在传输
        {
            dmaCounter++;
            if(dmaCounter > 0x7FFF)
            {
                HAL_UART_Abort(DEBUG_UART);
                HAL_UART_Transmit(DEBUG_UART, (uint8_t*)"\r\n UART1 TRANSMIT ERR\r\n", 23, 10);
                break;
            }
        }
        UART1_DMA.flag &= ~OPERABLE_BUFFER;		//CPU可操作改为buffer0
        UART1_DMA.flag &= ~BUFFER1_WAIT_SEND;
        HAL_UART_Transmit_DMA(USER_UART, UART1_DMA.buffer1, dataNum);			//使能DMA发送buffer1数据
    }
    else
    {
        for(uint8_t i = 0 ; i < dataNum ; i++)
        {
            UART1_DMA.buffer0[i] = transformedData[i];
        }
        UART1_DMA.flag |= BUFFER0_WAIT_SEND;
        while(READ_BIT(DMA_USER_UART1_TX->CR, DMA_SxCR_EN) != RESET)			//DMA正在传输
        {
            dmaCounter++;
            if(dmaCounter > 0x7FFF)
            {
                HAL_UART_Abort(DEBUG_UART);
                HAL_UART_Transmit(DEBUG_UART, (uint8_t*)"\r\n UART1 TRANSMIT ERR\r\n", 23, 10);
                break;
            }
        }
        UART1_DMA.flag |= OPERABLE_BUFFER;		//CPU可操作改为buffer1
        UART1_DMA.flag &= ~BUFFER0_WAIT_SEND;
        HAL_UART_Transmit_DMA(USER_UART, UART1_DMA.buffer0, dataNum);			//使能DMA发送buffer1数据
    }
}

/********************************************************
函数功能：串口收数完成中断回调函数
入口参数：
返回  值：
*********************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    UNUSED(huart);

    //接收外部命令中断

    if(huart == USER_UART)
    {
//        UART_RX_1_6.buffer[UART_RX_1_6.bufferNum++] = data1;
//        if(UART_RX_1_6.bufferNum >= UART_RX_BUFFER_SIZE)
//        {
//            UART_RX_1_6.bufferNum = 0;
//            memset(UART_RX_1_6.buffer, 0x00, UART_RX_BUFFER_SIZE);
//        }
//        if( (UART_RX_1_6.bufferNum > 1) && (UART_RX_1_6.buffer[UART_RX_1_6.bufferNum - 1] == '\n') && (UART_RX_1_6.buffer[UART_RX_1_6.bufferNum - 2] == '\r') )		//接收结束标志
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
//        if( (UART_RX_1_6.bufferNum > 1) && (UART_RX_1_6.buffer[UART_RX_1_6.bufferNum - 1] == '\n') && (UART_RX_1_6.buffer[UART_RX_1_6.bufferNum - 2] == '\r') )		//接收结束标志
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

