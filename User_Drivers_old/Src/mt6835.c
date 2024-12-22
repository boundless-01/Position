/**
  ******************************************************************************
  * @file    mt6835.c
  * @author  Tom, Falonss
  * @version Action1.0
  * @date    
  * @brief   21位高精度编码器MT6835
  ******************************************************************************
**/

#include "mt6835.h"

uint8_t MT6835_Read_SingleByte(uint16_t reg, SPI_HandleTypeDef *hspi)
{
	uint16_t data = MT6835_CMD_READ | reg;
	uint8_t data_send[3];
	uint8_t data_rev[3];
	data_send[0] = data & 0x00ff;
	data_send[1] = data >> 8;
	data_send[2] = 0;

	if(hspi == &hspi2)
	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port,ENCODER_Y_SPI2_CS_Pin,GPIO_PIN_RESET);
	else if(hspi == &hspi3)
	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port,ENCODER_X_SPI3_CS_Pin,GPIO_PIN_RESET);	

	HAL_SPI_TransmitReceive(hspi,data_send,data_rev,3,10);

	if(hspi == &hspi2)
	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port,ENCODER_Y_SPI2_CS_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi3)
	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port,ENCODER_X_SPI3_CS_Pin,GPIO_PIN_SET);	

	return data_rev[2];
}

void MT6835_Write_SingleByte(uint16_t reg, uint8_t data2write, SPI_HandleTypeDef *hspi)
{
	uint16_t data = MT6835_CMD_WRITE | reg;
	uint8_t data_send[3];
	uint8_t data_rev[3];
	data_send[0] = data & 0x00ff;
	data_send[1] = data >> 8;
	data_send[2] = data2write;

	if(hspi == &hspi2)
	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port,ENCODER_Y_SPI2_CS_Pin,GPIO_PIN_RESET);
	else if(hspi == &hspi3)
	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port,ENCODER_X_SPI3_CS_Pin,GPIO_PIN_RESET);	

	HAL_SPI_TransmitReceive(hspi,data_send,data_rev,3,10);

	if(hspi == &hspi2)
	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port,ENCODER_Y_SPI2_CS_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi3)
	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port,ENCODER_X_SPI3_CS_Pin,GPIO_PIN_SET);	
}

uint8_t MT6835_Write_EEPROM(SPI_HandleTypeDef *hspi)
{
	static uint8_t write_state;
	uint16_t data = MT6835_CMD_WRITE_EEPROM;
	uint8_t data_send[3];
	uint8_t data_rev[3];
	data_send[0] = 0;
	data_send[1] = data & 0x00ff;
	data_send[2] = data >> 8;

	if(hspi == &hspi2)
	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port,ENCODER_Y_SPI2_CS_Pin,GPIO_PIN_RESET);
	else if(hspi == &hspi3)
	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port,ENCODER_X_SPI3_CS_Pin,GPIO_PIN_RESET);	

	HAL_SPI_TransmitReceive(hspi,data_send,data_rev,3,10);
	if(data_rev[2] == MT6835_SET_ZERO_POINT_ACK)
	{
		write_state = 1;
	}
	else
	{
		write_state = 0;
	}

	if(hspi == &hspi2)
	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port,ENCODER_Y_SPI2_CS_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi3)
	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port,ENCODER_X_SPI3_CS_Pin,GPIO_PIN_SET);	
	
	return write_state;
}

uint8_t MT6835_Set_Zero_Point(SPI_HandleTypeDef *hspi)
{
	static uint8_t set_state;
	uint16_t data = MT6835_CMD_SET_ZERO_POINT;
	uint8_t data_send[3];
	uint8_t data_rev[3];
	data_send[0] = 0;
	data_send[1] = data & 0x00ff;
	data_send[2] = data >> 8;

	if(hspi == &hspi2)
	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port,ENCODER_Y_SPI2_CS_Pin,GPIO_PIN_RESET);
	else if(hspi == &hspi3)
	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port,ENCODER_X_SPI3_CS_Pin,GPIO_PIN_RESET);	

	HAL_SPI_TransmitReceive(hspi,data_send,data_rev,3,10);
	if(data_rev[2] == MT6835_SET_ZERO_POINT_ACK)
	{
		set_state = 1;
	}
	else
	{
		set_state = 0;
	}

	if(hspi == &hspi2)
	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port,ENCODER_Y_SPI2_CS_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi3)
	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port,ENCODER_X_SPI3_CS_Pin,GPIO_PIN_SET);	

	return set_state;
}

void MT6835_Init(void)
{
	__HAL_SPI_ENABLE(&hspi2);
	ENCODER_Y_SPI->Init.DataSize = SPI_DATASIZE_16BIT;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_SPI_ENABLE(&hspi3);
	ENCODER_X_SPI->Init.DataSize = SPI_DATASIZE_16BIT;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
	{
		Error_Handler();
	}
	
	uint16_t data_send[10];
	
	for(uint8_t i = 0;i<10;i++)
	{
		data_send[i] = 0xFFFF;
	}
	
	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port, ENCODER_Y_SPI2_CS_Pin, GPIO_PIN_RESET);	
	
	HAL_SPI_Transmit(ENCODER_Y_SPI,(unsigned char *)data_send,10,10);

	HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port, ENCODER_Y_SPI2_CS_Pin, GPIO_PIN_SET);
	
	for(uint8_t i = 0;i<10;i++)
	{
		data_send[i] = 0xFFFF;
	}
	
	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port, ENCODER_X_SPI3_CS_Pin, GPIO_PIN_RESET);	
	
	HAL_SPI_Transmit(ENCODER_X_SPI,(unsigned char *)data_send,10,10);

	HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port, ENCODER_X_SPI3_CS_Pin, GPIO_PIN_SET);

}


uint32_t MT6835_ReadAngle_21bit(SPI_HandleTypeDef *hspi)
{
	uint16_t data_send = MT6835_CMD_CONTINUOUS_READ | MT6835_READ_REG_ANGLE_3;
	uint16_t data_rev[3];
	uint32_t Angle_21bit;
	
	if(hspi == ENCODER_Y_SPI)
	{
		ENCODER_Y_CS_ENABLE;
	}
	else if(hspi == ENCODER_X_SPI)
	{
		ENCODER_X_CS_ENABLE;
	}

	HAL_SPI_TransmitReceive(hspi,(unsigned char *)&data_send,(unsigned char *)&data_rev,3,10);
	Angle_21bit = (data_rev[1]<<5)|(data_rev[2]>>11);

	if(hspi == ENCODER_Y_SPI)
	{
		ENCODER_Y_CS_DISABLE;
	}
	else if(hspi == ENCODER_X_SPI)
	{
		ENCODER_X_CS_DISABLE;
	}	

	return Angle_21bit;
}

uint32_t MT6835ReadAbsPos_X(void)
{
	uint32_t Angle_21bit;
	Angle_21bit = 	MT6835_ReadAngle_21bit(ENCODER_X_SPI);
	uint32_t Data = Angle_21bit & 0x1FFFFF;
	return Data;
}

uint32_t MT6835ReadAbsPos_Y(void)
{
	uint32_t Angle_21bit;
	Angle_21bit = 	MT6835_ReadAngle_21bit(ENCODER_Y_SPI);
	uint32_t Data = Angle_21bit & 0x1FFFFF;
	return Data;
}

//uint8_t MT6835_Self_Cal(void)
//{
//	static uint8_t mt6835_cal_state = 0;
//	static uint32_t eleAngle = 0;	
//	static uint32_t cnt = 0;	
//	float sine = 0.f;
//	float cosine = 0.f;	
//	static uint8_t self_cal_state = 0;
//	static uint8_t self_cal_state1 = 0;
//	
//	switch (mt6835_cal_state)
//  {
//  	case 0:

//				Set_SPI_8bit(&hspi2);
//				Set_SPI_8bit(&hspi3);
//				// 配置自校准允许的转速区间
//				MT6835_Write_SingleByte(MT6835_EEPROM_REG_DRV_CAL_SET, 0xF0, &hspi2);
//				MT6835_Write_SingleByte(MT6835_EEPROM_REG_DRV_CAL_SET, 0xF0, &hspi3);		
//				mt6835_cal_state = 1;
//		
//  		break;

//  	case 1:
//			
//			//将系统平稳的运行在目标转速 -> 47.61rpm
//			cnt ++;

//			arm_sin_cos_f32(eleAngle,&sine,&cosine);
//			SpaceVectorVol_MT1(0.9f, 0.f, sine, cosine);
//			SpaceVectorVol_MT2(0.9f, 0.f, sine, cosine);
//			if(cnt == 50)
//			{
//				eleAngle += 5;
//				//在系统已经平稳匀速运转的情况下，将引脚4接高电平进入自校准状态。
//				if(eleAngle == 360)
//				{
//					HAL_GPIO_WritePin(SPI1_CAL_EN_GPIO_Port, SPI1_CAL_EN_Pin, GPIO_PIN_SET);
//					HAL_GPIO_WritePin(SPI2_CAL_EN_GPIO_Port, SPI2_CAL_EN_Pin, GPIO_PIN_SET);
//				}
//				//保持这一速度继续运转64圈以上
//				else if(eleAngle >= 176400)
//				{
//					self_cal_state = MT6835_Read_SingleByte(MT6835_READ_REG_CAL_STATE, &hspi2);
//					self_cal_state1 = MT6835_Read_SingleByte(MT6835_READ_REG_CAL_STATE, &hspi3);
//					self_cal_state >>= 6;
//					self_cal_state1 >>= 6;
//					if((self_cal_state == MT6835_SELF_CAL_SUCCESS) && (self_cal_state1 == MT6835_SELF_CAL_SUCCESS))
//					{
//						mt6835_cal_state = 2;
//						cnt = 0;
//					}
//					else
//					{
//						mt6835_cal_state = 3;
//						PutStr("calculate err!\r\n");
//						SendBuf();
//						cnt = 0;
//					}
//				}
//				cnt = 0;
//			}

//  		break;

//  	case 2:
//			
//				cnt ++;
//				//等待>6秒钟，请务必给芯片断电
//				if(cnt == 140000)
//				{
//					cnt = 0;
//					PutStr("please shut up the power supply, calculate ok!\r\n");
//					SendBuf();
//				}

//  		break;

//  	case 3:
//			
//		

//  		break;
//		
//  	default:
//  		break;
//  }

//  return mt6835_cal_state;
//}	


