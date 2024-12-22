/**
  ******************************************************************************
  * @file    my_imu.c
  * @author  Falonss && Yin Qiyao
  * @version V1.0.0
  * @date    2023.03.06 - 2023.09.16
  * @brief   与IMU数据处理相关的代码
  ******************************************************************************
**/

#include "my_imu.h"

//IMU原始数据
static int32_t imuGyroSumRaw[AXIS_NUMBER] = {0};
static int32_t imuAccSumRaw[AXIS_NUMBER]  = {0};
static int32_t imuTempRaw[2] = {0};


/********************************************************
函数功能：陀螺仪初始化                
入口参数：
返回  值：
*********************************************************/
void IMU_Init(void)
{
	char serial_num[14];
	scha63x_init(serial_num);
	UART6_DMA_Out((uint8_t* )"%s\r\n",serial_num);
}

/********************************************************
函数功能：陀螺仪数据采样                
入口参数：
返回  值：
*********************************************************/
void IMU_DataSampling(void)
{
	//IMU原始数据结构体
	static scha63x_raw_data scha63x_raw_data_last;
	
	//通过SPI通信读取数据
	scha63x_read_data(&scha63x_raw_data_last);
	
	//从结构体中提取数据
	imuGyroSumRaw[0] += scha63x_raw_data_last.gyro_x_lsb;
	imuGyroSumRaw[1] += scha63x_raw_data_last.gyro_y_lsb;
	imuGyroSumRaw[2] += scha63x_raw_data_last.gyro_z_lsb;
	imuAccSumRaw[0] += scha63x_raw_data_last.acc_x_lsb;
	imuAccSumRaw[1] += scha63x_raw_data_last.acc_y_lsb;
	imuAccSumRaw[2] += scha63x_raw_data_last.acc_z_lsb;
	imuTempRaw[0] += scha63x_raw_data_last.temp_uno_lsb;
	imuTempRaw[1] += scha63x_raw_data_last.temp_due_lsb;
 
}

/********************************************************
函数功能：陀螺仪数据处理                
入口参数：
返回  值：
*********************************************************/
//IMU实际数据结构体
scha63x_real_data scha63x_real_data_crc;
void IMU_DataHandling(void)
{
    
	//将原始数据转换为标准单位制，并取平均值
	scha63x_real_data_crc.acc_x =  imuAccSumRaw[0] / (4905.f * RUN_PERIOD);
	scha63x_real_data_crc.acc_y =  imuAccSumRaw[1] / (4905.f * RUN_PERIOD);
	scha63x_real_data_crc.acc_z =  imuAccSumRaw[2] / (4905.f * RUN_PERIOD);
			
	scha63x_real_data_crc.gyro_x =  imuGyroSumRaw[0] / (80.f * RUN_PERIOD);
	scha63x_real_data_crc.gyro_y =  imuGyroSumRaw[1] / (80.f * RUN_PERIOD);
	scha63x_real_data_crc.gyro_z =  imuGyroSumRaw[2] / (80.f * RUN_PERIOD); // (80.21* RUN_PERIOD)
			
	scha63x_real_data_crc.temp_uno = 25.f + (float)imuTempRaw[0] / (30.f * RUN_PERIOD);
	scha63x_real_data_crc.temp_due = 25.f + (float)imuTempRaw[1] / (30.f * RUN_PERIOD);
	
	//利用IMU出厂校准信息对数据进行矫正
	scha63x_cross_axis_compensation(&scha63x_real_data_crc);
	
	//根据不同模式，将测量数据赋值给程序变量
//	#ifdef X_FACE_UP
//		psPara.imuPara.imuGyroWithoutRemoveDrift[0] = -scha63x_real_data_crc.gyro_x;
//		psPara.imuPara.imuGyroWithoutRemoveDrift[1] = -scha63x_real_data_crc.gyro_y;
//		psPara.imuPara.imuGyroWithoutRemoveDrift[2] = scha63x_real_data_crc.gyro_z;
//		psPara.imuPara.accAverWithZeroDrift[0] = scha63x_real_data_crc.acc_x;
//		psPara.imuPara.accAverWithZeroDrift[1] = scha63x_real_data_crc.acc_y;
//		psPara.imuPara.accAverWithZeroDrift[2] = scha63x_real_data_crc.acc_z;
//	#else
//		#ifdef Y_FACE_DOWN
//			psPara.imuPara.imuGyroWithoutRemoveDrift[0] = scha63x_real_data_crc.gyro_x;
//			psPara.imuPara.imuGyroWithoutRemoveDrift[1] = scha63x_real_data_crc.gyro_z;
//			psPara.imuPara.imuGyroWithoutRemoveDrift[2] = -scha63x_real_data_crc.gyro_y;
//			psPara.imuPara.accAverWithZeroDrift[0] = scha63x_real_data_crc.acc_x;
//			psPara.imuPara.accAverWithZeroDrift[1] = -scha63x_real_data_crc.acc_z;
//			psPara.imuPara.accAverWithZeroDrift[2] = scha63x_real_data_crc.acc_y;
//		#else
//			psPara.imuPara.imuGyroWithoutRemoveDrift[0] = scha63x_real_data_crc.gyro_x;
//			psPara.imuPara.imuGyroWithoutRemoveDrift[1] = -scha63x_real_data_crc.gyro_y;
//			psPara.imuPara.imuGyroWithoutRemoveDrift[2] = -scha63x_real_data_crc.gyro_z;
//			psPara.imuPara.accAverWithZeroDrift[0] = scha63x_real_data_crc.acc_x;
//			psPara.imuPara.accAverWithZeroDrift[1] = -scha63x_real_data_crc.acc_y;
//			psPara.imuPara.accAverWithZeroDrift[2] = -scha63x_real_data_crc.acc_z;
//		#endif
//	#endif
	
    psPara.imuPara.imuGyroWithoutRemoveDrift[0] =  scha63x_real_data_crc.gyro_x;
    psPara.imuPara.imuGyroWithoutRemoveDrift[1] = -scha63x_real_data_crc.gyro_y;
    psPara.imuPara.imuGyroWithoutRemoveDrift[2] =  scha63x_real_data_crc.gyro_z;
    psPara.imuPara.accAverWithZeroDrift[0] =  scha63x_real_data_crc.acc_x;
    psPara.imuPara.accAverWithZeroDrift[1] = -scha63x_real_data_crc.acc_y;
    psPara.imuPara.accAverWithZeroDrift[2] =  scha63x_real_data_crc.acc_z;    
    
	psPara.imuPara.imuTemperature = (scha63x_real_data_crc.temp_uno + scha63x_real_data_crc.temp_uno)/2;
	
	for(int axis = 0; axis < AXIS_NUMBER; axis ++)
	{
		//移除温漂后赋值(缺失)
		psPara.imuPara.imuGyroRemoveDrift[axis] = psPara.imuPara.imuGyroWithoutRemoveDrift[axis];
		
		//将数据清零
		imuGyroSumRaw[axis] = 0;
		imuAccSumRaw[axis] 	= 0;
	}
	
	imuTempRaw[0] = 0;
	imuTempRaw[1] = 0;
}


