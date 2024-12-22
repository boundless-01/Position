/**
  ******************************************************************************
  * @file    my_imu.c
  * @author  Falonss && Yin Qiyao
  * @version V1.0.0
  * @date    2023.03.06 - 2023.09.16
  * @brief   ��IMU���ݴ�����صĴ���
  ******************************************************************************
**/

#include "my_imu.h"

//IMUԭʼ����
static int32_t imuGyroSumRaw[AXIS_NUMBER] = {0};
static int32_t imuAccSumRaw[AXIS_NUMBER]  = {0};
static int32_t imuTempRaw[2] = {0};


/********************************************************
�������ܣ������ǳ�ʼ��                
��ڲ�����
����  ֵ��
*********************************************************/
void IMU_Init(void)
{
	char serial_num[14];
	scha63x_init(serial_num);
	UART6_DMA_Out((uint8_t* )"%s\r\n",serial_num);
}

/********************************************************
�������ܣ����������ݲ���                
��ڲ�����
����  ֵ��
*********************************************************/
void IMU_DataSampling(void)
{
	//IMUԭʼ���ݽṹ��
	static scha63x_raw_data scha63x_raw_data_last;
	
	//ͨ��SPIͨ�Ŷ�ȡ����
	scha63x_read_data(&scha63x_raw_data_last);
	
	//�ӽṹ������ȡ����
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
�������ܣ����������ݴ���                
��ڲ�����
����  ֵ��
*********************************************************/
//IMUʵ�����ݽṹ��
scha63x_real_data scha63x_real_data_crc;
void IMU_DataHandling(void)
{
    
	//��ԭʼ����ת��Ϊ��׼��λ�ƣ���ȡƽ��ֵ
	scha63x_real_data_crc.acc_x =  imuAccSumRaw[0] / (4905.f * RUN_PERIOD);
	scha63x_real_data_crc.acc_y =  imuAccSumRaw[1] / (4905.f * RUN_PERIOD);
	scha63x_real_data_crc.acc_z =  imuAccSumRaw[2] / (4905.f * RUN_PERIOD);
			
	scha63x_real_data_crc.gyro_x =  imuGyroSumRaw[0] / (80.f * RUN_PERIOD);
	scha63x_real_data_crc.gyro_y =  imuGyroSumRaw[1] / (80.f * RUN_PERIOD);
	scha63x_real_data_crc.gyro_z =  imuGyroSumRaw[2] / (80.f * RUN_PERIOD); // (80.21* RUN_PERIOD)
			
	scha63x_real_data_crc.temp_uno = 25.f + (float)imuTempRaw[0] / (30.f * RUN_PERIOD);
	scha63x_real_data_crc.temp_due = 25.f + (float)imuTempRaw[1] / (30.f * RUN_PERIOD);
	
	//����IMU����У׼��Ϣ�����ݽ��н���
	scha63x_cross_axis_compensation(&scha63x_real_data_crc);
	
	//���ݲ�ͬģʽ�����������ݸ�ֵ���������
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
		//�Ƴ���Ư��ֵ(ȱʧ)
		psPara.imuPara.imuGyroRemoveDrift[axis] = psPara.imuPara.imuGyroWithoutRemoveDrift[axis];
		
		//����������
		imuGyroSumRaw[axis] = 0;
		imuAccSumRaw[axis] 	= 0;
	}
	
	imuTempRaw[0] = 0;
	imuTempRaw[1] = 0;
}


