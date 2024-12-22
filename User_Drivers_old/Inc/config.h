#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_it.h"
#include "stm32h7xx.h"




/*��λϵͳ����ȳ�ֵ����*/

//���ݲ�ͬ���������ò�ͬ��ֵ

//#define RR								//rabbit robot

#ifdef RR

//��λϵͳ����ϵ->��������ϵ��ת�Ƕ�
#define psPara.oPara.ROTATE_ANGLE 			-45.f     //������ת���ٶ�
//��λϵͳ��װλ�þ೵����λ�õľ���
#define psPara.oPara.PPS2CENTER 				150.0086f

//��������ϵ����������ϵ��X��������нǣ���ʱ��Ϊ��
#define psPara.oPara.POSEANGLE					0.f

//��λϵͳ��װ����������x��y�᷽��(��ǽ�ƶ�)�ƶ���ȫ�̽Ƕȱ仯Ϊ0ʱ��ֻ��x��y����仯
#define CODEWHEELANGLEERR 0.02601f

//��������ϵ�³���������
#define psPara.oPara.ORIGINAL_POSX 		6451.704f
#define psPara.oPara.ORIGINAL_POSY 		247.376f

//�������־�
#define R_WHEEL_X					25.4f
#define R_WHEEL_Y					25.4f

#endif

//#define ER						//elephant robot
#ifdef ER

//��λϵͳ����ϵ->��������ϵ��ת�Ƕ�
#define psPara.oPara.ROTATE_ANGLE 			-135.f

//��λϵͳ��װλ�þ೵����λ��
#define psPara.oPara.PPS2CENTER 				192.02f

//��������ϵ����������ϵ��X��������нǣ���ʱ��Ϊ��
#define psPara.oPara.POSEANGLE					0.f

//��λϵͳ��װ����������x��y�᷽��(��ǽ�ƶ�)�ƶ���ȫ�̽Ƕȱ仯Ϊ0ʱ��ֻ��x��y����仯
#define CODEWHEELANGLEERR 0.02601f

//��������ϵ�³���������
#define psPara.oPara.ORIGINAL_POSX 		5658.000f
#define psPara.oPara.ORIGINAL_POSY 		380.993f

//�������־�
#define R_WHEEL_X					25.4f
#define R_WHEEL_Y					25.4f

#endif

#define wheel							//����
#ifdef wheel


//�������־�
#define R_WHEEL_X					25.14f
#define R_WHEEL_Y					25.04f

#endif
/*��λϵͳ��������*/

//���������ǳ���ѡ��
#define X_FACE_UP

//��������
#define RUN_PERIOD 			5

//����ʱ�䣺1ms * 5 = 0.005s
#define dT							0.005f

/*��λϵͳ�豸����*/
#define AXIS_NUMBER 		3

/*IMU���ݽṹ��*/
typedef struct
{
	/*IMU GYRO*/
	
	//������������ƫ����
	double imuGyroZeroDrift[AXIS_NUMBER];
	
	//������������ƫ��3��
	double imuGyroRateIntegralThreshold[AXIS_NUMBER];
	
	//���������ǽ��ٶȲ���ֵ
	double imuGyroWithoutRemoveDrift[AXIS_NUMBER];
	
	//�����������Ƴ���Ư��Ľ��ٶȲ���ֵ
	double imuGyroRemoveDrift[AXIS_NUMBER];
	
	//�����������Ƴ���Ư��Ľ��ٶȲ���ֵ
	double imuGyro_Real[AXIS_NUMBER];
	
	/*IMU ACC*/
	
	//���ٶȳ�ʼ����ֵ
	double accInit[AXIS_NUMBER];
	
	//���ٶȳ�ʼ�ɼ���ģ��
	double gInit;
	
	//���ٶȳ�ʼ���ݵ�3��
	double accIntegralThresholdAll;
	
	//������ٶȳ�ʼ���ݵ�3��
	double accIntegralThreshold[AXIS_NUMBER];
	
	//������ٶȲ���ֵ
	double accAverWithZeroDrift[AXIS_NUMBER];
	
	//���ó�ʼģ�����ţ����������µĽ��ٶ�
	double Acc_Real[AXIS_NUMBER];

	//���������µĽ��ٶȳ�ʼֵ
	double accByAngleZeroDrift[AXIS_NUMBER];
	
	//�Ƴ����������������¸�����ٶ�
	double Acc_World[AXIS_NUMBER];
	
	/*IMU TEMP*/
	double imuTemperature;
	
	/*IMU ANGLE*/
	
	//ͨ����ʼ״̬�ļ��ٶȼ�ȷ����ǰ�����ͺ����(������)
	double startAngle[AXIS_NUMBER];
	
	//�ɳ�ʼ״̬�ĸ����Ǽ�����Ǽ������Ԫ��
	double startquaternion[4];
	
	//������ֽǶȵ���Ԫ��
	double quaternion[4];
	
	//������ֽǶ�
	double imuResult_Angle[AXIS_NUMBER];
	
	//���������ڽǶ�
	double lastAngle[AXIS_NUMBER];
	
		//���������ڽǶ�
	double lastAngle_rotate[AXIS_NUMBER];
		
}IMUPara_t;

/*ENCODER���ݽṹ��*/
typedef struct
{
	//�����ڱ�������ֵ
	uint32_t lastEncoder[2];
	
	//��������ֵ
	uint32_t encoderDATA[2];
	
	//���α�������ֵʱ�����������¼�����������λmm
	double encoderMileage[2];
	
	//����y����ǰ��x������������ϵ��������ٶȼ���λϵͳ����ϵ
	double posx;
	
	double posy;
	
	double vellx;
	
	double velly;
	
	uint32_t encoderSum[2];
	
}EPara_t;

/*USART���ݽṹ��*/
typedef struct
{	
	/*��������ϵ�¶Ի����������У��*/
	double correctX;
	
	double correctY;
	
	double correctAngle[AXIS_NUMBER];
	
	/*���յ���У����Ϣ�����������������ͳһ�Գ���ʹ�õ�У����Ϣ���и���*/
	/*������ֲ���У������δУ�����쳣���*/
	double newCorrectX;
	
	double newCorrectY;
	
	double newCorrectAngle[AXIS_NUMBER];
	
	/*������Ϣ���ձ�־λ*/
	uint8_t receiveFlagX;
	
	uint8_t receiveFlagY;
	
	uint8_t receiveFlagA;
	
	uint8_t correctFlag;
	
	/*��У����ĽǶ�*/
	double sendAngle;
	
}UPara_t;

typedef struct
{
/*��λϵͳ��ʼ���ݽṹ��*/

	//��λϵͳ����ϵ->��������ϵ��ת�Ƕȣ�һ�㶼Ҫ�����ֵ��
	double ROTATE_ANGLE;

	//��λϵͳ��װλ�þ೵����λ��ʸ��y������
	double PPS2CENTER_Y;
												  
	//��λϵͳ��װλ�þ೵����λ��ʸ��x������
	double PPS2CENTER_X;

	//��������ϵ����������ϵ��X��������нǣ���ʱ��Ϊ�������˿صģ�
	double  POSEANGLE;

	double ORIGINYAW;

	double INSTALLYAW;
		
	double  PPS2CENTER;


	//��λϵͳ��װ����������x��y�᷽��(��ǽ�ƶ�)�ƶ���ȫ�̽Ƕȱ仯Ϊ0ʱ��ֻ��x��y����仯
	double  CODEWHEELANGLEERR_YAW;
	double  CODEWHEELANGLEERR_PITCH;
			

	//��������ϵ�³���������
	double ORIGINAL_POSX;
	double ORIGINAL_POSY;


}OPara_t;  //Origin

/*POSITIONER���ݽṹ��*/
typedef struct
{
	/*��������ϵ�µ�������ٶ���Ϣ*/
	double rotatedPosX;
	
	double rotatedPosY;
	
	double rotatedVelX;
	
	double rotatedVelY;
	
	/*��������ϵ�µ�������ٶ���Ϣ*/
	double rotatedPosX_robort;
	
	double rotatedPosY_robort;
	
	double rotatedVelX_robort;
	
	double rotatedVelY_robort;
	
	double last_RotatedPosX_robort;
	
	double last_RotatedPosY_robort;
	
	double rotatedPosX_robort_inverse;
	
	double rotatedPosY_robort_inverse;
	
	int timeCounter;
	
	OPara_t oPara;
	
	UPara_t uPara;
	
	EPara_t ePara;
	
	IMUPara_t imuPara;
	
}PSPara_t;

typedef enum
{
	FALSE = 0, TRUE = !FALSE
}BOOL;

extern PSPara_t psPara;
#endif
