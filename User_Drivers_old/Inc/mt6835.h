#ifndef __MT6835_H
#define __MT6835_H

#include "spi.h"
#include "arm_math.h"

//CMD
#define MT6835_CMD_READ							    0x3000
#define MT6835_CMD_WRITE 						    0x6000
#define MT6835_CMD_WRITE_EEPROM			    0xC000
#define MT6835_CMD_SET_ZERO_POINT		    0x5000
#define MT6835_CMD_CONTINUOUS_READ			0xA000

//EEPROM
#define MT6835_EEPROM_REG_USER_ID						0x001
#define MT6835_EEPROM_REG_ABZ_RES_2					0x007
#define MT6835_EEPROM_REG_ABZ_RES_1					0x008
#define MT6835_EEPROM_REG_ZERO_POS_2				0x009
#define MT6835_EEPROM_REG_ZERO_POS_1				0x00A
#define MT6835_EEPROM_REG_UVW_SET						0x00B
#define MT6835_EEPROM_REG_PWM_SET						0x00C
#define MT6835_EEPROM_REG_ROT_SET						0x00D
#define MT6835_EEPROM_REG_DRV_CAL_SET				0x00E
#define MT6835_EEPROM_REG_BW_SET						0x011

//READ ONLY
#define MT6835_READ_REG_ANGLE_3						0x003
#define MT6835_READ_REG_ANGLE_2						0x004
#define MT6835_READ_REG_ANGLE_1						0x005
#define MT6835_READ_REG_CRC								0x006
#define MT6835_READ_REG_CAL_STATE					0x113	//[7:6]:00未运行 01正在进行 10失败 11成功

#define MT6835_AUTO_CAL_FREQ_3200_6400_RPM				0x0
#define MT6835_AUTO_CAL_FREQ_1600_3200_RPM				0x1
#define MT6835_AUTO_CAL_FREQ_800_1600_RPM				0x2
#define MT6835_AUTO_CAL_FREQ_400_800_RPM					0x3
#define MT6835_AUTO_CAL_FREQ_200_400_RPM					0x4
#define MT6835_AUTO_CAL_FREQ_100_200_RPM					0x5
#define MT6835_AUTO_CAL_FREQ_50_100_RPM					0x6
#define MT6835_AUTO_CAL_FREQ_25_50_RPM						0x7

#define MT6835_WRITE_EEPROM_ACK							0x55
#define MT6835_SET_ZERO_POINT_ACK						0x55

#define MT6835_SELF_CAL_NSTART						0x00
#define MT6835_SELF_CAL_OPRATING					0x01
#define MT6835_SELF_CAL_FAILED						0x02
#define MT6835_SELF_CAL_SUCCESS						0x03

/*SPI*/
//按定位系统板子等腰三角形顶角朝左下角定义定位系统原始坐标系原点，两个编码轮分别朝X正方向和Y正方形
#define ENCODER_Y_SPI				(&hspi2)
#define ENCODER_X_SPI				(&hspi3)

#define ENCODER_Y_CS_ENABLE    HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port , ENCODER_Y_SPI2_CS_Pin , GPIO_PIN_RESET)
#define ENCODER_Y_CS_DISABLE   HAL_GPIO_WritePin(ENCODER_Y_SPI2_CS_GPIO_Port , ENCODER_Y_SPI2_CS_Pin , GPIO_PIN_SET)

#define ENCODER_X_CS_ENABLE    HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port , ENCODER_X_SPI3_CS_Pin , GPIO_PIN_RESET)
#define ENCODER_X_CS_DISABLE   HAL_GPIO_WritePin(ENCODER_X_SPI3_CS_GPIO_Port , ENCODER_X_SPI3_CS_Pin , GPIO_PIN_SET)

uint8_t MT6835_Read_SingleByte(uint16_t reg, SPI_HandleTypeDef *hspi);
void MT6835_Write_SingleByte(uint16_t reg, uint8_t data2write, SPI_HandleTypeDef *hspi);
uint8_t MT6835_Write_EEPROM(SPI_HandleTypeDef *hspi);
uint8_t MT6835_Set_Zero_Point(SPI_HandleTypeDef *hspi);
uint32_t MT6835_ReadAngle_21bit(SPI_HandleTypeDef *hspi);
void MT6835_User_Init(SPI_HandleTypeDef *hspi);
uint32_t MT6835ReadAbsPos_X(void);
uint32_t MT6835ReadAbsPos_Y(void);
void MT6835_Init(void);

#endif
