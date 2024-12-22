/**
  ******************************************************************************
  * @file    figureAngle.c
  * @author  Lxy Zlq  XFR HJN/PLG/LXQ YQY
  * @version Action2.0
  * @date
  * @brief   姿态角解算算法
  ******************************************************************************
**/

#include "figureAngle.h"
#include "config.h"
#include "my_usart.h"
/********************************************************
函数功能：imu测量数据的零偏采集及处理
入口参数：
返回  值：零偏采集与否的标志位
*********************************************************/

//上电等待 WAIT_CYCLE * 5ms后开始采集数据
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


    /*等待10s后开始收集零飘数据*/
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
函数功能：imu测量数据的零偏采集
入口参数：
返回  值：零偏采集与否的标志位
*********************************************************/
uint8_t UpdateIMUZeroDrift(void)
{
    /*3轴角速度+3轴加速度*/
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
  * @brief	更新陀螺仪角度零飘，收集CALCULATE_NUMBER(500)个数据
  * @name	imuUpdateGyroOneAxisZeroDrift
	* @param	axis 计算哪个轴上的零飘
  * @retval 零飘计算完毕，返回1 否则0
  * @note 方差=S^2 标准差=S
  */
#define CALCULATE_NUMBER 500
uint8_t imuUpdateGyroOneAxisZeroDrift(uint8_t axis)	// 0--0000 1--0001 2--0010
{
    static double gyroDeviation[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f}; //单次数据与平均值差值，由于动态检测下不准，所以只在静态下更新
    static double gyroBuffer[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f};
    static uint16_t gyroStdDevBufferNum[AXIS_NUMBER] = {0};
    static double gyroSum[AXIS_NUMBER] = {0};
    static double gyroMean[AXIS_NUMBER] = {0};
    static double gyroStdDev[AXIS_NUMBER] = {0};//标准差
    static uint16_t	updateCounter[AXIS_NUMBER] = {0};
    static uint8_t gyroZeroDriftFlag = 0;
    //如果采集到的值在3西格玛内  {（μ-3σ，μ+3σ)中的概率为0.9973 （正态分布中σ代表标准差，μ代表均值）}
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
        gyroDeviation[axis][gyroStdDevBufferNum[axis]] = psPara.imuPara.imuGyroRemoveDrift[axis] - gyroMean[axis];//(x - μ)
        gyroStdDev[axis] = 0;
        for(int i=0 ; i < CALCULATE_NUMBER ; i++)
        {
            gyroStdDev[axis] += (gyroDeviation[axis][i]*gyroDeviation[axis][i]);	//Σ(x - μ)^2
        }
        gyroStdDev[axis] = __sqrtf(gyroStdDev[axis] / (CALCULATE_NUMBER*1.f));	//更新标准差
        gyroStdDevBufferNum[axis]++;
        if(gyroStdDevBufferNum[axis] >= CALCULATE_NUMBER)
        {
            gyroStdDevBufferNum[axis] = 0;
        }

        if(gyroZeroDriftFlag & (0x01<<axis))		//标准差更新完毕
        {
            if(updateCounter[axis] > 500)
            {
                if( (3*gyroStdDev[axis] < 2.f*psPara.imuPara.imuGyroRateIntegralThreshold[axis]) 	\
                        && (3*gyroStdDev[axis] > 0.71f*psPara.imuPara.imuGyroRateIntegralThreshold[axis]))	//如果方差在设定范围内
                {
                    psPara.imuPara.imuGyroZeroDrift[axis] = gyroMean[axis];
                    updateCounter[axis] = 300;	//如果一直检测到静止就1s更新一次
                }
                if( !(gyroZeroDriftFlag & (0x01<<(axis+3))) )	//零飘未更新
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
        else if(gyroStdDevBufferNum[axis] == 0)	//gyroStdDevBufferNum经过一个循环变回0，证明标准差和均值更新完毕
        {
            gyroZeroDriftFlag |= (0x01<<axis); //将0x01左移axis位 变成   (0000 0001)    (0000 0010)     (0000 0100)
        }
    }
    else
    {
        updateCounter[axis] = 0;
    }

    if( gyroZeroDriftFlag & (0x01<<(axis+3)) )// 将 (0000 1000)    (0001 0000)     (0010 0000)
    {
        return 1;
    }
    return 0;
}
/********************************************************
函数功能：重力在各轴分量测量数据的零偏采集
入口参数：
返回  值：零偏采集与否的标志位
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
  * @brief	根据当前姿态角将重力分解，得到在世界坐标下各轴的分量
  * @name	UpdateAccByAngle
	* @param
  * @retval
  * @note
  */
void UpdateAccByAngle(void)//计算g在新的三轴的投影，减去他们，从而得到去除g的加速度
{
    double angle[3] = {0.f};
    double acc[3]   = {0.f};
    /*角度制转弧度制, 顺序是俯仰角，横滚角，偏航角；沿x,y,z轴方向的加速度*/
    for(int axis = 0; axis < 3; axis++)
    {
        angle[axis] = psPara.imuPara.imuResult_Angle[axis] *PI / 180.f;
    }

    //2022四足，右手坐标系, 将陀螺仪测量的加速度从机体坐标系转到世界坐标系
    acc[0] =   psPara.imuPara.Acc_Real[1];
    acc[1] = - psPara.imuPara.Acc_Real[0];
    acc[2] = - psPara.imuPara.Acc_Real[2];

    psPara.imuPara.Acc_World[0] = acc[1]*cos(-angle[1])*sin(angle[2]) + acc[2]*(-cos(angle[2])*sin(-angle[0]) - cos(-angle[0])*sin(-angle[1])*sin(angle[2])) + acc[0]*(cos(-angle[0])*cos(angle[2]) - sin(-angle[1])*sin(-angle[0])*sin(angle[2]));
    psPara.imuPara.Acc_World[1] = acc[1]*cos(-angle[1])*cos(angle[2]) + acc[0]*(-cos(angle[2])*sin(-angle[1])*sin(-angle[0]) - cos(-angle[0])*sin(angle[2]))+acc[2]*(-cos(-angle[0])*cos(angle[2])*sin(-angle[1]) + sin(-angle[0])*sin(angle[2]));
    psPara.imuPara.Acc_World[2] = acc[2]*cos(angle[1])*cos(-angle[0]) - acc[1]*sin(angle[1]) + acc[0]*cos(angle[1])*sin(-angle[0]);
}
/**
  * @brief	更新各轴重力加速度的分量，并以此作为零偏
  * @name	UpdateAccByAngleOneZeroDrift
	* @param	axis 计算哪个轴上的模长
  * @retval 零飘计算完毕，返回1 否则0
  * @note 方差=S^2 标准差=S
  */
uint8_t UpdateAccByAngleOneZeroDrift(uint8_t axis)
{
    static float accDeviation[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f}; //单次数据与平均值差值，由于动态检测下不准，所以只在静态下更新
    static float accBuffer[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f};
    static uint16_t accStdDevBufferNum[AXIS_NUMBER] = {0};
    static double accSum[AXIS_NUMBER] = {0};
    static double accMean[AXIS_NUMBER] = {0};
    static double accStdDev[AXIS_NUMBER] = {0};
    static uint8_t accZeroDriftFlag = 0;		//0-2为表示buffer数组是否填满，3-5为表示零飘是否更新,第6位表示三轴全部更新完毕
    static uint16_t	accUpdateCounter[AXIS_NUMBER] = {0};

    if( accZeroDriftFlag & (0x01<<6) )		//加速度的温漂影响不大，则加速度不采取动态更新零飘，初始化完则确定
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
            accStdDev[axis] += (accDeviation[axis][i]*accDeviation[axis][i]);	//加和
        }
        accStdDev[axis] = __sqrtf(accStdDev[axis] / (CALCULATE_NUMBER*1.f));	//更新标准差
        accStdDevBufferNum[axis]++;
        if(accStdDevBufferNum[axis] >= CALCULATE_NUMBER)
        {
            accStdDevBufferNum[axis] = 0;
        }

        if(accZeroDriftFlag & (0x01<<axis))		//标准差更新完毕
        {
            if( (accUpdateCounter[axis] > 500) && !(accZeroDriftFlag & (0x01<<(axis+3))))
            {
                accZeroDriftFlag |= (0x01<<(axis+3));
                accUpdateCounter[axis] = 400;
                psPara.imuPara.accByAngleZeroDrift[axis] = accMean[axis];
                if((accZeroDriftFlag & 0x38) == 0x38)		//三轴均完成
                {
                    accZeroDriftFlag |= (0x01<<6);
                }
            }
        }
        else if(accStdDevBufferNum[axis] == 0)	//accStdDevBufferNum经过一个循环变回0，证明标准差和均值更新完毕
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
  * @brief	更新加速度计模长，收集CALCULATE_NUMBER(500)个数据
  * @name	imuUpdateGyroOneAxisZeroDrift
	* @param	axis 计算哪个轴上的模长
  * @retval 零飘计算完毕，返回1 否则0
  * @note 方差=S^2 标准差=S
  */
uint8_t UpdateAccelerometerOneZeroDrift(uint8_t axis)
{
    static float accDeviation[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f}; //单次数据与平均值差值，由于动态检测下不准，所以只在静态下更新
    static float accBuffer[AXIS_NUMBER][CALCULATE_NUMBER]= {0.f};
    static uint16_t accStdDevBufferNum[AXIS_NUMBER] = {0};
    static double accSum[AXIS_NUMBER] = {0};
    static double accMean[AXIS_NUMBER] = {0};
    static double accStdDev[AXIS_NUMBER] = {0};
    static uint16_t	accUpdateCounter[AXIS_NUMBER] = {0};
    static uint8_t accZeroDriftFlag = 0;

    if( accZeroDriftFlag  & (0x01<<6) )		//加速度的温漂影响不大，则加速度不采取动态更新零飘，初始化完则确定
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
            accStdDev[axis] += (accDeviation[axis][i]*accDeviation[axis][i]);	//加和
        }
        accStdDev[axis] = __sqrtf(accStdDev[axis] / (CALCULATE_NUMBER*1.f));	//更新标准差
        accStdDevBufferNum[axis]++;
        if(accStdDevBufferNum[axis] >= CALCULATE_NUMBER)
        {
            accStdDevBufferNum[axis] = 0;
        }

        if(accZeroDriftFlag  & (0x01<<axis))		//标准差更新完毕
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
                if((accZeroDriftFlag  & 0x38) == 0x38)		//三轴均完成
                {
                    //三轴加速度标准差矢量和的模
                    psPara.imuPara.accIntegralThresholdAll=2*sqrt(pow(psPara.imuPara.accIntegralThreshold[0],2) \
                                                           + pow(psPara.imuPara.accIntegralThreshold[1],2) + pow(psPara.imuPara.accIntegralThreshold[2],2));
                    //初始化加速度大小
                    psPara.imuPara.gInit = sqrtf(psPara.imuPara.accInit[0]*psPara.imuPara.accInit[0] + \
                                                 psPara.imuPara.accInit[1]*psPara.imuPara.accInit[1] + psPara.imuPara.accInit[2]*psPara.imuPara.accInit[2]);

                    psPara.imuPara.accInit[0] /= psPara.imuPara.gInit;
                    psPara.imuPara.accInit[1] /= psPara.imuPara.gInit;
                    psPara.imuPara.accInit[2] /= psPara.imuPara.gInit;
                    /*用当前加速度确定初始四元数*/
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
        else if(accStdDevBufferNum[axis] == 0)	//accStdDevBufferNum经过一个循环变回0，证明标准差和均值更新完毕
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
  * @brief  更新角度值
  *
  * @param[in] gyrData,输入陀螺仪的加速度值
  *            magData,输入磁力计的值
  *            accData,输入加速度的值
  * @retval 初始化完成的标志位
  */
void UpdateAngleByPixhawk(void)
{
    static double gyroAngle[3];	//以弧度制表示角速度，加入加速度计误差矫正
    static double euler[3];

    gyroAngle[0] = psPara.imuPara.imuGyro_Real[0];
    gyroAngle[1] = psPara.imuPara.imuGyro_Real[1];
    gyroAngle[2] = psPara.imuPara.imuGyro_Real[2];

    /* 阈值 */
    for(int axis = 0; axis < AXIS_NUMBER; axis++)
    {
        if(fabs(gyroAngle[axis])<psPara.imuPara.imuGyroRateIntegralThreshold[axis])
            gyroAngle[axis]=0;
    }

    /* 角度弧度转换 */
    gyroAngle[0] = gyroAngle[0] / 180.f * PI;
    gyroAngle[1] = gyroAngle[1] / 180.f * PI;
    gyroAngle[2] = gyroAngle[2] / 180.f * PI;

    GyroCorrection(gyroAngle, psPara.imuPara.quaternion);	//四元数补偿矫正   							


    /*2阶龙格库塔，四元数积分*/
    QuaternionInt(psPara.imuPara.quaternion, gyroAngle);

    /* 四元数转换成欧垃角 地理到机体Z-Y-X三中和顺序解析欧拉角*/
    Quaternion_to_Euler(psPara.imuPara.quaternion,euler);
    /*欧拉角 它的实际意义并不是物理上的俯仰横滚偏航，是个人通过四元数解析出来的
    所以这个欧拉角解析有顺序*/

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

/*误差补偿控制器*/
void GyroCorrection(double gyroAngleCorrect[3], double qCorrect[4])
{
    /*n系到R系的旋转*/
    double nZtoRZ[3] = {0.f};
    double gNormalized[3] = {0.f};
    /*误差补偿量*/
    static double errCorrectKi[3] = {0.f};
    double errCorrect[3] = {0.f};
    static double lastErr[3] = {0.f};
    static double errKp[3] = {4.3255, 4.3255, 4.995};
    static double errKi[3] = {0.000473, 0.000473, 0.000473};
    static double errKd[3] = {0.0f, 0.0f, 0.f};
//    static double ki_test[3]= {0.f};
    static double err[3] = {0.f};
    //地理Z轴正方向在机体中的坐标
    nZtoRZ[0] = 2 *( qCorrect[1] * qCorrect[3] - qCorrect[0] * qCorrect[2] );	//计算T31
    nZtoRZ[1] = 2 *( qCorrect[2] * qCorrect[3] + qCorrect[0] * qCorrect[1] );	//T32
    nZtoRZ[2] = (qCorrect[3] * qCorrect[3] - qCorrect[2] * qCorrect[2] - qCorrect[1] * qCorrect[1] + qCorrect[0] * qCorrect[0]);	//T33

    double aNow = sqrtf(psPara.imuPara.Acc_Real[0] * psPara.imuPara.Acc_Real[0] + psPara.imuPara.Acc_Real[1]*psPara.imuPara.Acc_Real[1] + psPara.imuPara.Acc_Real[2] * psPara.imuPara.Acc_Real[2]);	//归一化分母        
    /*加速度计归一化操作，Z轴正方向坐标*/
    for(int axis=0; axis<3; axis++)
    {
        gNormalized[axis] = psPara.imuPara.Acc_Real[axis] / aNow;
    }
    //计算地理Z轴正方向叉乘重力反馈地理Z轴正方向坐标
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

