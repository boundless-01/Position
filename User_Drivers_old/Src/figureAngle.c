/**
  ******************************************************************************
  * @file    figureAngle.c
  * @author  Lxy Zlq  XFR HJN/PLG/LXQ YQY
  * @version Action2.0
  * @date
  * @brief   ��̬�ǽ����㷨
  ******************************************************************************
**/

#include "figureAngle.h"
#include "config.h"
#include "my_usart.h"
/********************************************************
�������ܣ�imu�������ݵ���ƫ�ɼ�������
��ڲ�����
����  ֵ����ƫ�ɼ����ı�־λ
*********************************************************/

//�ϵ�ȴ� WAIT_CYCLE * 5ms��ʼ�ɼ�����
#define WAIT_CYCLE	200
uint8_t RoughHandle(void)
{
    static int Wait=0;
	
	extern int FieldChange; 
	
    if(Wait <= 10 * WAIT_CYCLE)
	{
		
	uint8_t tdata[52];
    
	union
    {
        float   val;
        uint8_t data[4];
    } valSend;
	
	tdata[0]='A';
    tdata[1]='T';
    tdata[50]='t';
    tdata[51]='a';
	if(FieldChange==0)
	{
		valSend.val=(float)0;
		memcpy(tdata+2,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+6,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+10,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+14,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+18,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+22,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+26,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+30,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+34,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+38,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+42,valSend.data,4);
		valSend.val=(float)0;
		memcpy(tdata+46,valSend.data,4);

	}
	if(FieldChange==1)
	{
		valSend.val=(float)1;
		memcpy(tdata+2,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+6,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+10,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+14,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+18,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+22,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+26,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+30,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+34,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+38,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+42,valSend.data,4);
		valSend.val=(float)1;
		memcpy(tdata+46,valSend.data,4);

	}
		Wait++;
	
		//UART1_DMA_Transmit(tdata,52);
		sendUSBData(tdata,52);

	}


    /*�ȴ�10s��ʼ�ռ���Ʈ����*/
    if(Wait > 5*WAIT_CYCLE)		//5s
    {
        if(UpdateIMUZeroDrift())
        {
            for(int axis = 0; axis < AXIS_NUMBER; axis++)
            {
                psPara.imuPara.imuGyro_Real[axis] = psPara.imuPara.imuGyroRemoveDrift[axis] - psPara.imuPara.imuGyroZeroDrift[axis];
                psPara.imuPara.Acc_Real[axis] = psPara.imuPara.accAverWithZeroDrift[axis] / psPara.imuPara.gInit;
            }
            return 1;
        }
    }

    return 0;
}

/********************************************************
�������ܣ�imu�������ݵ���ƫ�ɼ�
��ڲ�����
����  ֵ����ƫ�ɼ����ı�־λ
*********************************************************/
uint8_t UpdateIMUZeroDrift(void)
{
    /*3����ٶ�+3����ٶ�*/
    uint8_t returnValueGyro[3]= {0};
    uint8_t returnValueAcc[3]= {0};
    uint8_t returnValueSum=0;
    for(int axis = 0; axis < AXIS_NUMBER; axis ++ )
    {
        returnValueGyro[axis]=imuUpdateGyroOneAxisZeroDrift(axis);
        returnValueSum+=returnValueGyro[axis];
    }
    for(int axis = 0; axis < AXIS_NUMBER; axis ++ )
    {
        returnValueAcc[axis]=UpdateAccelerometerOneZeroDrift(axis);
        returnValueSum+=returnValueAcc[axis];
    }
    if(returnValueSum >= 6)
    {
        return 1;
    }
    return 0;
}

/**
  * @brief	���������ǽǶ���Ʈ���ռ�CALCULATE_NUMBER(500)������
  * @name	imuUpdateGyroOneAxisZeroDrift
	* @param	axis �����ĸ����ϵ���Ʈ
  * @retval ��Ʈ������ϣ�����1 ����0
  * @note ����=S^2 ��׼��=S
  */
#define CALCULATE_NUMBER 500
uint8_t imuUpdateGyroOneAxisZeroDrift(uint8_t axis)	// 0--0000 1--0001 2--0010
{
    static double gyroDeviation[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f}; //����������ƽ��ֵ��ֵ�����ڶ�̬����²�׼������ֻ�ھ�̬�¸���
    static double gyroBuffer[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f};
    static uint16_t gyroStdDevBufferNum[AXIS_NUMBER] = {0};
    static double gyroSum[AXIS_NUMBER] = {0};
    static double gyroMean[AXIS_NUMBER] = {0};
    static double gyroStdDev[AXIS_NUMBER] = {0};//��׼��
    static uint16_t	updateCounter[AXIS_NUMBER] = {0};
    static uint8_t gyroZeroDriftFlag = 0;
    //����ɼ�����ֵ��3��������  {����-3�ң���+3��)�еĸ���Ϊ0.9973 ����̬�ֲ��ЦҴ����׼��̴����ֵ��}
    if( (fabs(psPara.imuPara.imuGyroRemoveDrift[axis]-gyroMean[axis]) < 3*gyroStdDev[axis]) || !(gyroZeroDriftFlag & (0x01<<axis)) )
    {
        updateCounter[axis]++;
        gyroSum[axis] -= gyroBuffer[axis][gyroStdDevBufferNum[axis]];
        gyroBuffer[axis][gyroStdDevBufferNum[axis]] = psPara.imuPara.imuGyroRemoveDrift[axis];
        gyroSum[axis] += gyroBuffer[axis][gyroStdDevBufferNum[axis]];
        if(gyroZeroDriftFlag & (0x01<<axis))
        {
            gyroMean[axis] = gyroSum[axis] / CALCULATE_NUMBER;
        }
        else
        {
            gyroMean[axis] = gyroSum[axis] / (gyroStdDevBufferNum[axis]+1);
        }
        gyroDeviation[axis][gyroStdDevBufferNum[axis]] = psPara.imuPara.imuGyroRemoveDrift[axis] - gyroMean[axis];//(x - ��)
        gyroStdDev[axis] = 0;
        for(int i=0 ; i < CALCULATE_NUMBER ; i++)
        {
            gyroStdDev[axis] += (gyroDeviation[axis][i]*gyroDeviation[axis][i]);	//��(x - ��)^2
        }
        gyroStdDev[axis] = __sqrtf(gyroStdDev[axis] / (CALCULATE_NUMBER*1.f));	//���±�׼��
        gyroStdDevBufferNum[axis]++;
        if(gyroStdDevBufferNum[axis] >= CALCULATE_NUMBER)
        {
            gyroStdDevBufferNum[axis] = 0;
        }

        if(gyroZeroDriftFlag & (0x01<<axis))		//��׼��������
        {
            if(updateCounter[axis] > 500)
            {
                if( (3*gyroStdDev[axis] < 2.f*psPara.imuPara.imuGyroRateIntegralThreshold[axis]) 	\
                        && (3*gyroStdDev[axis] > 0.71f*psPara.imuPara.imuGyroRateIntegralThreshold[axis]))	//����������趨��Χ��
                {
                    psPara.imuPara.imuGyroZeroDrift[axis] = gyroMean[axis];
                    updateCounter[axis] = 300;	//���һֱ��⵽��ֹ��1s����һ��
                }
                if( !(gyroZeroDriftFlag & (0x01<<(axis+3))) )	//��Ʈδ����
                {
                    psPara.imuPara.imuGyroZeroDrift[axis] = gyroMean[axis];
                    psPara.imuPara.imuGyroRateIntegralThreshold[axis] = 3*gyroStdDev[axis];
                    if(psPara.imuPara.imuGyroRateIntegralThreshold[axis]>0.3f)
                    {
                        psPara.imuPara.imuGyroRateIntegralThreshold[axis]=0.3f;
                    }
                    gyroZeroDriftFlag |= (0x01<<(axis+3));
                    updateCounter[axis] = 0;
                }
            }
        }
        else if(gyroStdDevBufferNum[axis] == 0)	//gyroStdDevBufferNum����һ��ѭ�����0��֤����׼��;�ֵ�������
        {
            gyroZeroDriftFlag |= (0x01<<axis); //��0x01����axisλ ���   (0000 0001)    (0000 0010)     (0000 0100)
        }
    }
    else
    {
        updateCounter[axis] = 0;
    }

    if( gyroZeroDriftFlag & (0x01<<(axis+3)) )// �� (0000 1000)    (0001 0000)     (0010 0000)
    {
        return 1;
    }
    return 0;
}
/********************************************************
�������ܣ������ڸ�������������ݵ���ƫ�ɼ�
��ڲ�����
����  ֵ����ƫ�ɼ����ı�־λ
*********************************************************/
uint8_t UpdateAccByAngleZeroDrift(void)
{
    uint8_t returnValueAccByAngle[3]= {0};
    uint8_t returnValueSum=0;
    UpdateAccByAngle();
    for(int i=0; i<3; i++)
    {
        returnValueAccByAngle[i] = UpdateAccByAngleOneZeroDrift(i);
        returnValueSum+=returnValueAccByAngle[i];
    }
    if(returnValueSum>=3)
    {
        for(int axis=0; axis<3; axis++)
        {
            psPara.imuPara.Acc_World[axis] -= psPara.imuPara.accByAngleZeroDrift[axis];
            psPara.imuPara.Acc_World[axis] = -psPara.imuPara.Acc_World[axis];
        }
        return 1;
    }
    return 0;
}
/**
  * @brief	���ݵ�ǰ��̬�ǽ������ֽ⣬�õ������������¸���ķ���
  * @name	UpdateAccByAngle
	* @param
  * @retval
  * @note
  */
void UpdateAccByAngle(void)//����g���µ������ͶӰ����ȥ���ǣ��Ӷ��õ�ȥ��g�ļ��ٶ�
{
    double angle[3] = {0.f};
    double acc[3]   = {0.f};
    /*�Ƕ���ת������, ˳���Ǹ����ǣ�����ǣ�ƫ���ǣ���x,y,z�᷽��ļ��ٶ�*/
    for(int axis = 0; axis < 3; axis++)
    {
        angle[axis] = psPara.imuPara.imuResult_Angle[axis] *PI / 180.f;
    }

    //2022���㣬��������ϵ, �������ǲ����ļ��ٶȴӻ�������ϵת����������ϵ
    acc[0] =   psPara.imuPara.Acc_Real[1];
    acc[1] = - psPara.imuPara.Acc_Real[0];
    acc[2] = - psPara.imuPara.Acc_Real[2];

    psPara.imuPara.Acc_World[0] = acc[1]*cos(-angle[1])*sin(angle[2]) + acc[2]*(-cos(angle[2])*sin(-angle[0]) - cos(-angle[0])*sin(-angle[1])*sin(angle[2])) + acc[0]*(cos(-angle[0])*cos(angle[2]) - sin(-angle[1])*sin(-angle[0])*sin(angle[2]));
    psPara.imuPara.Acc_World[1] = acc[1]*cos(-angle[1])*cos(angle[2]) + acc[0]*(-cos(angle[2])*sin(-angle[1])*sin(-angle[0]) - cos(-angle[0])*sin(angle[2]))+acc[2]*(-cos(-angle[0])*cos(angle[2])*sin(-angle[1]) + sin(-angle[0])*sin(angle[2]));
    psPara.imuPara.Acc_World[2] = acc[2]*cos(angle[1])*cos(-angle[0]) - acc[1]*sin(angle[1]) + acc[0]*cos(angle[1])*sin(-angle[0]);
}
/**
  * @brief	���¸����������ٶȵķ��������Դ���Ϊ��ƫ
  * @name	UpdateAccByAngleOneZeroDrift
	* @param	axis �����ĸ����ϵ�ģ��
  * @retval ��Ʈ������ϣ�����1 ����0
  * @note ����=S^2 ��׼��=S
  */
uint8_t UpdateAccByAngleOneZeroDrift(uint8_t axis)
{
    static float accDeviation[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f}; //����������ƽ��ֵ��ֵ�����ڶ�̬����²�׼������ֻ�ھ�̬�¸���
    static float accBuffer[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f};
    static uint16_t accStdDevBufferNum[AXIS_NUMBER] = {0};
    static double accSum[AXIS_NUMBER] = {0};
    static double accMean[AXIS_NUMBER] = {0};
    static double accStdDev[AXIS_NUMBER] = {0};
    static uint8_t accZeroDriftFlag = 0;		//0-2Ϊ��ʾbuffer�����Ƿ�������3-5Ϊ��ʾ��Ʈ�Ƿ����,��6λ��ʾ����ȫ���������
    static uint16_t	accUpdateCounter[AXIS_NUMBER] = {0};

    if( accZeroDriftFlag & (0x01<<6) )		//���ٶȵ���ƯӰ�첻������ٶȲ���ȡ��̬������Ʈ����ʼ������ȷ��
    {
        return 1;
    }

    if( !(accZeroDriftFlag & (0x01<<axis)) || (fabs(psPara.imuPara.Acc_World[axis]-accMean[axis]) < 3*accStdDev[axis]) )
    {
        accUpdateCounter[axis]++;
        accSum[axis] -= accBuffer[axis][accStdDevBufferNum[axis]];
        accBuffer[axis][accStdDevBufferNum[axis]] = psPara.imuPara.Acc_World[axis];
        accSum[axis] += accBuffer[axis][accStdDevBufferNum[axis]];
        if(accZeroDriftFlag & (0x01<<axis))
        {
            accMean[axis] = accSum[axis] / CALCULATE_NUMBER;
        }
        else
        {
            accMean[axis] = accSum[axis] / (accStdDevBufferNum[axis]+1);
        }
        accDeviation[axis][accStdDevBufferNum[axis]] = psPara.imuPara.Acc_World[axis] - accMean[axis];
        accStdDev[axis] = 0;
        for(int i=0 ; i < CALCULATE_NUMBER ; i++)
        {
            accStdDev[axis] += (accDeviation[axis][i]*accDeviation[axis][i]);	//�Ӻ�
        }
        accStdDev[axis] = __sqrtf(accStdDev[axis] / (CALCULATE_NUMBER*1.f));	//���±�׼��
        accStdDevBufferNum[axis]++;
        if(accStdDevBufferNum[axis] >= CALCULATE_NUMBER)
        {
            accStdDevBufferNum[axis] = 0;
        }

        if(accZeroDriftFlag & (0x01<<axis))		//��׼��������
        {
            if( (accUpdateCounter[axis] > 500) && !(accZeroDriftFlag & (0x01<<(axis+3))))
            {
                accZeroDriftFlag |= (0x01<<(axis+3));
                accUpdateCounter[axis] = 400;
                psPara.imuPara.accByAngleZeroDrift[axis] = accMean[axis];
                if((accZeroDriftFlag & 0x38) == 0x38)		//��������
                {
                    accZeroDriftFlag |= (0x01<<6);
                }
            }
        }
        else if(accStdDevBufferNum[axis] == 0)	//accStdDevBufferNum����һ��ѭ�����0��֤����׼��;�ֵ�������
        {
            accZeroDriftFlag |= (0x01<<axis);
        }
    }
    else
    {
        accUpdateCounter[axis] = 0;
    }

    if( accZeroDriftFlag & (0x01<<(axis+3)) )
    {
        return 1;
    }
    return 0;
}
/**
  * @brief	���¼��ٶȼ�ģ�����ռ�CALCULATE_NUMBER(500)������
  * @name	imuUpdateGyroOneAxisZeroDrift
	* @param	axis �����ĸ����ϵ�ģ��
  * @retval ��Ʈ������ϣ�����1 ����0
  * @note ����=S^2 ��׼��=S
  */
uint8_t UpdateAccelerometerOneZeroDrift(uint8_t axis)
{
    static float accDeviation[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f}; //����������ƽ��ֵ��ֵ�����ڶ�̬����²�׼������ֻ�ھ�̬�¸���
    static float accBuffer[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f};
    static uint16_t accStdDevBufferNum[AXIS_NUMBER] = {0};
    static double accSum[AXIS_NUMBER] = {0};
    static double accMean[AXIS_NUMBER] = {0};
    static double accStdDev[AXIS_NUMBER] = {0};
    static uint16_t	accUpdateCounter[AXIS_NUMBER] = {0};
    static uint8_t accZeroDriftFlag = 0;

    if( accZeroDriftFlag  & (0x01<<6) )		//���ٶȵ���ƯӰ�첻������ٶȲ���ȡ��̬������Ʈ����ʼ������ȷ��
    {
        return 1;
    }

    if( !(accZeroDriftFlag  & (0x01<<axis)) || (fabs(psPara.imuPara.accAverWithZeroDrift[axis]-accMean[axis]) < 3*accStdDev[axis]) )
    {
        accUpdateCounter[axis]++;
        accSum[axis] -= accBuffer[axis][accStdDevBufferNum[axis]];
        accBuffer[axis][accStdDevBufferNum[axis]] = psPara.imuPara.accAverWithZeroDrift[axis];
        accSum[axis] += accBuffer[axis][accStdDevBufferNum[axis]];
        if(accZeroDriftFlag  & (0x01<<axis))
        {
            accMean[axis] = accSum[axis] / CALCULATE_NUMBER;
        }
        else
        {
            accMean[axis] = accSum[axis] / (accStdDevBufferNum[axis]+1);
        }
        accDeviation[axis][accStdDevBufferNum[axis]] = psPara.imuPara.accAverWithZeroDrift[axis] - accMean[axis];
        accStdDev[axis] = 0;
        for(int i=0 ; i < CALCULATE_NUMBER ; i++)
        {
            accStdDev[axis] += (accDeviation[axis][i]*accDeviation[axis][i]);	//�Ӻ�
        }
        accStdDev[axis] = __sqrtf(accStdDev[axis] / (CALCULATE_NUMBER*1.f));	//���±�׼��
        accStdDevBufferNum[axis]++;
        if(accStdDevBufferNum[axis] >= CALCULATE_NUMBER)
        {
            accStdDevBufferNum[axis] = 0;
        }

        if(accZeroDriftFlag  & (0x01<<axis))		//��׼��������
        {
            if( (accUpdateCounter[axis] > 500) && !(accZeroDriftFlag  & (0x01<<(axis+3))))
            {
                psPara.imuPara.accInit[axis] = accMean[axis];
                psPara.imuPara.accIntegralThreshold[axis] = 3*accStdDev[axis];
                if(psPara.imuPara.accIntegralThreshold[axis]>0.03f)
                {
                    psPara.imuPara.accIntegralThreshold[axis]=0.03f;
                }
                accZeroDriftFlag  |= (0x01<<(axis+3));
                accUpdateCounter[axis] = 400;
                if((accZeroDriftFlag  & 0x38) == 0x38)		//��������
                {
                    //������ٶȱ�׼��ʸ���͵�ģ
                    psPara.imuPara.accIntegralThresholdAll=2*sqrt(pow(psPara.imuPara.accIntegralThreshold[0],2) \
                                                           + pow(psPara.imuPara.accIntegralThreshold[1],2) + pow(psPara.imuPara.accIntegralThreshold[2],2));
                    //��ʼ�����ٶȴ�С
                    psPara.imuPara.gInit = sqrtf(psPara.imuPara.accInit[0]*psPara.imuPara.accInit[0] + \
                                                 psPara.imuPara.accInit[1]*psPara.imuPara.accInit[1] + psPara.imuPara.accInit[2]*psPara.imuPara.accInit[2]);

                    psPara.imuPara.accInit[0] /= psPara.imuPara.gInit;
                    psPara.imuPara.accInit[1] /= psPara.imuPara.gInit;
                    psPara.imuPara.accInit[2] /= psPara.imuPara.gInit;
                    /*�õ�ǰ���ٶ�ȷ����ʼ��Ԫ��*/
                    if(psPara.imuPara.accInit[2]==0&&psPara.imuPara.accInit[1]<=0)
                        psPara.imuPara.startAngle[0]=90;
                    else if(psPara.imuPara.accInit[2]==0&&psPara.imuPara.accInit[1]>0)
                        psPara.imuPara.startAngle[0]=-90;
                    else
                        psPara.imuPara.startAngle[0]= atan2(psPara.imuPara.accInit[1],psPara.imuPara.accInit[2]);
                    psPara.imuPara.startAngle[1]=-asin(psPara.imuPara.accInit[0]);
                    psPara.imuPara.quaternion[0]= cos(psPara.imuPara.startAngle[0]/2)*cos(psPara.imuPara.startAngle[1]/2);
                    psPara.imuPara.quaternion[1]= sin(psPara.imuPara.startAngle[0]/2)*cos(psPara.imuPara.startAngle[1]/2);
                    psPara.imuPara.quaternion[2]= cos(psPara.imuPara.startAngle[0]/2)*sin(psPara.imuPara.startAngle[1]/2);
                    psPara.imuPara.quaternion[3]=-sin(psPara.imuPara.startAngle[0]/2)*sin(psPara.imuPara.startAngle[1]/2);
                    psPara.imuPara.startquaternion[0]= cos(psPara.imuPara.startAngle[0]/2)*cos(psPara.imuPara.startAngle[1]/2);
                    psPara.imuPara.startquaternion[1]= sin(psPara.imuPara.startAngle[0]/2)*cos(psPara.imuPara.startAngle[1]/2);
                    psPara.imuPara.startquaternion[2]= cos(psPara.imuPara.startAngle[0]/2)*sin(psPara.imuPara.startAngle[1]/2);
                    psPara.imuPara.startquaternion[3]=-sin(psPara.imuPara.startAngle[0]/2)*sin(psPara.imuPara.startAngle[1]/2);
                    accZeroDriftFlag |= (0x01<<6);
                }
            }
        }
        else if(accStdDevBufferNum[axis] == 0)	//accStdDevBufferNum����һ��ѭ�����0��֤����׼��;�ֵ�������
        {
            accZeroDriftFlag |= (0x01<<axis);
        }
    }
    else
    {
        accUpdateCounter[axis] = 0;
    }

    if( accZeroDriftFlag & (0x01<<(axis+3)) )
    {
        return 1;
    }
    return 0;

}

/**
  * @brief  ���½Ƕ�ֵ
  *
  * @param[in] gyrData,���������ǵļ��ٶ�ֵ
  *            magData,��������Ƶ�ֵ
  *            accData,������ٶȵ�ֵ
  * @retval ��ʼ����ɵı�־λ
  */
void UpdateAngleByPixhawk(void)
{
    static double gyroAngle[3];	//�Ի����Ʊ�ʾ���ٶȣ�������ٶȼ�������
    static double euler[3];

    gyroAngle[0] = psPara.imuPara.imuGyro_Real[0];
    gyroAngle[1] = psPara.imuPara.imuGyro_Real[1];
    gyroAngle[2] = psPara.imuPara.imuGyro_Real[2];

    /* ��ֵ */
    for(int axis = 0; axis < AXIS_NUMBER; axis++)
    {
        if(fabs(gyroAngle[axis])<psPara.imuPara.imuGyroRateIntegralThreshold[axis])
            gyroAngle[axis]=0;
    }

    /* �ǶȻ���ת�� */
    gyroAngle[0] = gyroAngle[0] / 180.f * PI;
    gyroAngle[1] = gyroAngle[1] / 180.f * PI;
    gyroAngle[2] = gyroAngle[2] / 180.f * PI;

    GyroCorrection(gyroAngle, psPara.imuPara.quaternion);	//��Ԫ����������   							


    /*2�������������Ԫ������*/
    QuaternionInt(psPara.imuPara.quaternion, gyroAngle);

    /* ��Ԫ��ת����ŷ���� ��������Z-Y-X���к�˳�����ŷ����*/
    Quaternion_to_Euler(psPara.imuPara.quaternion,euler);
    /*ŷ���� ����ʵ�����岢���������ϵĸ������ƫ�����Ǹ���ͨ����Ԫ������������
    �������ŷ���ǽ�����˳��*/

    psPara.imuPara.imuResult_Angle[0] = euler[0] / PI * 180.f;
    psPara.imuPara.imuResult_Angle[1] = euler[1] / PI * 180.f;
    psPara.imuPara.imuResult_Angle[2] = euler[2] / PI * 180.f;

    for(uint8_t axis=0 ; axis < AXIS_NUMBER ; axis++)
    {
        if(psPara.imuPara.imuResult_Angle[axis]>180.0)
            psPara.imuPara.imuResult_Angle[axis]-=360.0;
        else if(psPara.imuPara.imuResult_Angle[axis]<-180.0)
            psPara.imuPara.imuResult_Angle[axis]+=360.0;
    }
}

/*����������*/
void GyroCorrection(double gyroAngleCorrect[3], double qCorrect[4])
{
    /*nϵ��Rϵ����ת*/
    double nZtoRZ[3] = {0.f};
    double gNormalized[3] = {0.f};
    /*������*/
    static double errCorrectKi[3] = {0.f};
    double errCorrect[3] = {0.f};
    static double lastErr[3] = {0.f};
    static double errKp[3] = {4.3255, 4.3255, 4.995};
    static double errKi[3] = {0.000473, 0.000473, 0.000473};
    static double errKd[3] = {0.0f, 0.0f, 0.f};
//    static double ki_test[3]= {0.f};
    static double err[3] = {0.f};
    //����Z���������ڻ����е�����
    nZtoRZ[0] = 2 *( qCorrect[1] * qCorrect[3] - qCorrect[0] * qCorrect[2] );	//����T31
    nZtoRZ[1] = 2 *( qCorrect[2] * qCorrect[3] + qCorrect[0] * qCorrect[1] );	//T32
    nZtoRZ[2] = (qCorrect[3] * qCorrect[3] - qCorrect[2] * qCorrect[2] - qCorrect[1] * qCorrect[1] + qCorrect[0] * qCorrect[0]);	//T33

    double aNow = sqrtf(psPara.imuPara.Acc_Real[0] * psPara.imuPara.Acc_Real[0] + psPara.imuPara.Acc_Real[1]*psPara.imuPara.Acc_Real[1] + psPara.imuPara.Acc_Real[2] * psPara.imuPara.Acc_Real[2]);	//��һ����ĸ        
    /*���ٶȼƹ�һ��������Z������������*/
    for(int axis=0; axis<3; axis++)
    {
        gNormalized[axis] = psPara.imuPara.Acc_Real[axis] / aNow;
    }
    //�������Z����������������������Z������������
    err[0] =  - nZtoRZ[2] * gNormalized[1] + nZtoRZ[1] * gNormalized[2];										
    err[1] =  - nZtoRZ[0] * gNormalized[2] + nZtoRZ[2] * gNormalized[0];
    err[2] =  - nZtoRZ[1] * gNormalized[0] + nZtoRZ[0] * gNormalized[1];
    for(int axis=0; axis<3; axis++)
    {
        if((fabs(aNow - psPara.imuPara.gInit) <= fabs(psPara.imuPara.accIntegralThresholdAll)) && (fabs(aNow - psPara.imuPara.gInit) <= fabs(psPara.imuPara.accIntegralThresholdAll)))
        {
            if(fabs(errCorrectKi[axis]+errKi[axis]*err[axis])<0.0001)
            {
                errCorrectKi[axis] += errKi[axis]*err[axis];
            }
            else
            {
                errCorrectKi[axis] = 0;
            }
            errCorrect[axis] = errKp[axis]*err[axis] + errCorrectKi[axis]+errKd[axis]*(err[axis]-lastErr[axis]);
        }
        else if((fabs(aNow - psPara.imuPara.gInit) <= 1.3 * fabs(psPara.imuPara.accIntegralThresholdAll)) && (fabs(aNow - psPara.imuPara.gInit) > fabs(psPara.imuPara.accIntegralThresholdAll)))
        {
            if(fabs(errCorrectKi[axis]+errKi[axis]*err[axis])<0.0001)
            {
                errCorrectKi[axis] += errKi[axis]*err[axis];
            }
            else
            {
                errCorrectKi[axis] = 0;
            }
            errCorrect[axis] = errKp[axis]*err[axis] + errCorrectKi[axis]+errKd[axis]*(err[axis]-lastErr[axis]);
            errCorrect[axis] = errCorrect[axis] * 0.1;
        }
        else
        {
            errCorrectKi[axis]=0.f;
            errCorrect[axis] = 0.f;
        }
        gyroAngleCorrect[axis] = gyroAngleCorrect[axis] - errCorrect[axis];
    }
    for(int axis=0; axis<3; axis++)
    {
        lastErr[axis]=err[axis];
    }
}

