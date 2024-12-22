/**
  ******************************************************************************
  * @file    my_main.c
  * @author  Falonss && Yin Qiyao
  * @version V1.0.0
  * @date    2023.03.06 - 2023.09.16
  * @brief   ������
  ******************************************************************************
**/

#include "my_main.h"
PSPara_t psPara;

/********************************************************
�������ܣ�������                
��ڲ�����
����  ֵ��
*********************************************************/
void main_loop(void)
{
	
	//�û������ʼ��
	Tim_User_Start();
	Uart_User_Init();
	IMU_Init();
	MT6835_Init();
	PsPallInit();
	while(1)
	{
		if(GetOneReadTimeFlag()) //1ms
		{
			//IMU���ݲ���
			IMU_DataSampling();
		}
		
		//���������ﵽRUN_PERIOD(RC����5�Σ����������������Ҫ3��)�󣬽���IMU���ݴ����Լ�����������
		if(GetTimeFlag())//5ms
		{
			
			//IMU���ݴ���
			IMU_DataHandling();
			
			//�������������ֵ��˶�����
			FigureVell();
			
			if(RoughHandle()) //���ͺ������л���־λ��imu������Ʈ�ɼ��͸���
			{
				//��̬�ǽ���
				UpdateAngleByPixhawk();
				
				if(UpdateAccByAngleZeroDrift()) //������̬�ǽ������ֽ⵽���Ტ�ɼ�������Ʈ
				{
					//�������
					CalculatePos();
				
					//����1����
					DataSend();
				
					//��������ָʾ��
					WORK_ON;
				}
				
			}
			
			//DEBUG������
			DebugDataSendByDMA();
			
			//У����Ϣ����
			CorrectHandler();
			
		}
	}
}
/********************************************************
�������ܣ���λϵͳ���ݽṹ���ʼ��                
��ڲ�����
����  ֵ��
*********************************************************/
void PsPallInit(void)
{
	for(int axis = 0; axis < AXIS_NUMBER; axis++)
	{
		psPara.imuPara.imuGyroRemoveDrift[axis] = 0.f;
		psPara.imuPara.imuGyroRateIntegralThreshold[axis] = 0.f;
		psPara.imuPara.imuGyroWithoutRemoveDrift[axis] = 0.f;
		psPara.imuPara.imuGyroZeroDrift[axis] = 0.f;
		psPara.imuPara.imuGyro_Real[axis] = 0.f;
		psPara.imuPara.accInit[axis] = 0.f;
		psPara.imuPara.accIntegralThreshold[axis] = 0.f;
		psPara.imuPara.accAverWithZeroDrift[axis] = 0.f;
		psPara.imuPara.accByAngleZeroDrift[axis] = 0.f;
		psPara.imuPara.Acc_Real[axis] = 0.f;
		psPara.imuPara.Acc_World[axis] = 0.f;
		psPara.imuPara.startAngle[axis] = 0.f;
		psPara.imuPara.imuResult_Angle[axis] = 0.f;
		psPara.imuPara.lastAngle[axis] = 0.f;
		psPara.imuPara.lastAngle_rotate[axis] = 0.f;
		psPara.uPara.correctAngle[axis] = 0.f;
		psPara.uPara.newCorrectAngle[axis] = 0.f;
	}
	
	psPara.oPara.ROTATE_ANGLE=-135.f;

	//��λϵͳ��װλ�þ೵����λ��ʸ��y������
	psPara.oPara.PPS2CENTER_Y = 0.0f;

                                              
	//��λϵͳ��װλ�þ೵����λ��ʸ��x������
	psPara.oPara.PPS2CENTER_X = 0.0f;


	//��������ϵ����������ϵ��X��������нǣ���ʱ��Ϊ�������˿صģ�
	psPara.oPara.POSEANGLE = 45.f;

	psPara.oPara.ORIGINYAW = 45.0f;
	
	psPara.oPara.INSTALLYAW = 0.0f;
	
	psPara.oPara.CODEWHEELANGLEERR_YAW = 0.0f;
	
	psPara.oPara.CODEWHEELANGLEERR_PITCH = 0.0f;
	
	psPara.oPara.ORIGINAL_POSX = 0.0f;
	psPara.oPara.ORIGINAL_POSY = 0.0f;
	
	psPara.oPara.PPS2CENTER = 0.0f;
	
	psPara.imuPara.startquaternion[0] = 0.f;
	psPara.imuPara.startquaternion[1] = 0.f;
	psPara.imuPara.startquaternion[2] = 0.f;
	psPara.imuPara.startquaternion[3] = 0.f;
	psPara.imuPara.quaternion[0] = 0.f;
	psPara.imuPara.quaternion[1] = 0.f;
	psPara.imuPara.quaternion[2] = 0.f;
	psPara.imuPara.quaternion[3] = 0.f;
	psPara.imuPara.gInit = 0.f;
	psPara.imuPara.accIntegralThresholdAll = 0.f;
	psPara.ePara.encoderDATA[0] = 0;
	psPara.ePara.encoderDATA[1] = 0;
	psPara.ePara.lastEncoder[0] = 0;
	psPara.ePara.lastEncoder[1] = 0;
	psPara.ePara.encoderMileage[0] = 0.f;
	psPara.ePara.encoderMileage[1] = 0.f;
	psPara.ePara.posx = 0.f;
	psPara.ePara.posy = 0.f;
	psPara.ePara.vellx = 0.f;
	psPara.ePara.velly = 0.f;
	psPara.uPara.correctX = psPara.oPara.ORIGINAL_POSX;
	psPara.uPara.correctY = psPara.oPara.ORIGINAL_POSY;
	psPara.uPara.correctAngle[2] = psPara.oPara.POSEANGLE;
	psPara.uPara.receiveFlagA = FALSE;
	psPara.uPara.receiveFlagX = FALSE;
	psPara.uPara.receiveFlagY = FALSE;
	psPara.uPara.correctFlag = TRUE;
	psPara.uPara.sendAngle = 0.f;
	psPara.last_RotatedPosX_robort = 0.f;
	psPara.last_RotatedPosY_robort = 0.f;
	psPara.rotatedPosX = 0.f;
	psPara.rotatedPosX_robort = 0.f;
	psPara.rotatedPosY = 0.f;
	psPara.rotatedPosY_robort = 0.f;
	psPara.rotatedVelX = 0.f;
	psPara.rotatedVelX_robort = 0.f;
	psPara.rotatedVelY = 0.f;
	psPara.rotatedVelY_robort = 0.f;
	psPara.timeCounter = 0;
}
