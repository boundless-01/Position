#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_it.h"
#include "stm32h7xx.h"




/*定位系统坐标等初值设置*/

//根据不同机器人设置不同的值

//#define RR								//rabbit robot

#ifdef RR

//定位系统坐标系->机体坐标系旋转角度
#define psPara.oPara.ROTATE_ANGLE 			-45.f     //决定旋转多少度
//定位系统安装位置距车中心位置的距离
#define psPara.oPara.PPS2CENTER 				150.0086f

//世界坐标系到机体坐标系的X轴正方向夹角，逆时针为正
#define psPara.oPara.POSEANGLE					0.f

//定位系统安装误差：调整至沿x或y轴方向(贴墙移动)移动，全程角度变化为0时，只有x或y坐标变化
#define CODEWHEELANGLEERR 0.02601f

//世界坐标系下出发点坐标
#define psPara.oPara.ORIGINAL_POSX 		6451.704f
#define psPara.oPara.ORIGINAL_POSY 		247.376f

//编码轮轮径
#define R_WHEEL_X					25.4f
#define R_WHEEL_Y					25.4f

#endif

//#define ER						//elephant robot
#ifdef ER

//定位系统坐标系->机体坐标系旋转角度
#define psPara.oPara.ROTATE_ANGLE 			-135.f

//定位系统安装位置距车中心位置
#define psPara.oPara.PPS2CENTER 				192.02f

//世界坐标系到机体坐标系的X轴正方向夹角，逆时针为正
#define psPara.oPara.POSEANGLE					0.f

//定位系统安装误差：调整至沿x或y轴方向(贴墙移动)移动，全程角度变化为0时，只有x或y坐标变化
#define CODEWHEELANGLEERR 0.02601f

//世界坐标系下出发点坐标
#define psPara.oPara.ORIGINAL_POSX 		5658.000f
#define psPara.oPara.ORIGINAL_POSY 		380.993f

//编码轮轮径
#define R_WHEEL_X					25.4f
#define R_WHEEL_Y					25.4f

#endif

#define wheel							//轮组
#ifdef wheel


//编码轮轮径
#define R_WHEEL_X					25.14f
#define R_WHEEL_Y					25.04f

#endif
/*定位系统程序设置*/

//根据陀螺仪朝向选择
#define X_FACE_UP

//采样次数
#define RUN_PERIOD 			5

//采样时间：1ms * 5 = 0.005s
#define dT							0.005f

/*定位系统设备设置*/
#define AXIS_NUMBER 		3

/*IMU数据结构体*/
typedef struct
{
	/*IMU GYRO*/
	
	//三轴陀螺仪零偏数据
	double imuGyroZeroDrift[AXIS_NUMBER];
	
	//三轴陀螺仪零偏的3σ
	double imuGyroRateIntegralThreshold[AXIS_NUMBER];
	
	//三轴陀螺仪角速度测量值
	double imuGyroWithoutRemoveDrift[AXIS_NUMBER];
	
	//三轴陀螺仪移除温漂后的角速度测量值
	double imuGyroRemoveDrift[AXIS_NUMBER];
	
	//三轴陀螺仪移除零漂后的角速度测量值
	double imuGyro_Real[AXIS_NUMBER];
	
	/*IMU ACC*/
	
	//加速度初始测量值
	double accInit[AXIS_NUMBER];
	
	//加速度初始采集的模长
	double gInit;
	
	//加速度初始数据的3σ
	double accIntegralThresholdAll;
	
	//三轴加速度初始数据的3σ
	double accIntegralThreshold[AXIS_NUMBER];
	
	//三轴加速度测量值
	double accAverWithZeroDrift[AXIS_NUMBER];
	
	//利用初始模长缩放，机体坐标下的角速度
	double Acc_Real[AXIS_NUMBER];

	//世界坐标下的角速度初始值
	double accByAngleZeroDrift[AXIS_NUMBER];
	
	//移除重力后，世界坐标下各轴加速度
	double Acc_World[AXIS_NUMBER];
	
	double last_Acc_World[AXIS_NUMBER];
	/*IMU TEMP*/
	double imuTemperature;
	
	/*IMU ANGLE*/
	
	//通过初始状态的加速度计确定当前俯仰和横滚角(弧度制)
	double startAngle[AXIS_NUMBER];
	
	//由初始状态的俯仰角及横滚角计算的四元数
	double startquaternion[4];
	
	//三轴积分角度的四元数
	double quaternion[4];//q0=1, q1与 x 轴上的旋转有关, q2与 y 轴上的旋转有关, q3与 z 轴上的旋转有关
	
	//三轴积分角度
	double imuResult_Angle[AXIS_NUMBER];
	
	//三轴上周期角度
	double lastAngle[AXIS_NUMBER];
	
		//三轴上周期角度
	double lastAngle_rotate[AXIS_NUMBER];
		
}IMUPara_t;

/*ENCODER数据结构体*/
typedef struct
{
	//上周期编码器读值
	uint32_t lastEncoder[2];
	
	//编码器读值
	uint32_t encoderDATA[2];
	
	//两次编码器读值时间里编码器记录的里程数，单位mm
	double encoderMileage[2];
	
	//按照y轮向前，x轮向左建立坐标系的坐标和速度即定位系统坐标系
	double posx;//X轴上的位置坐标
	
	double posy;//Y轴上的位置坐标
	
	double vellx;//X轴上的速度分量
	
	double velly;//Y轴上的速度分量
	
	uint32_t encoderSum[2];
	
}EPara_t;

/*USART数据结构体*/
typedef struct
{	
	/*世界坐标系下对机体坐标进行校正*/
	double correctX;//在世界坐标系中的初始x值
	
	double correctY;//在世界坐标系中的初始y值
	
	double correctAngle[AXIS_NUMBER];
	
	/*新收到的校正信息，在主程序运行完后统一对程序使用的校正信息进行更新*/
	/*避免出现部分校正部分未校正的异常情况*/
	double newCorrectX;
	
	double newCorrectY;
	
	double newCorrectAngle[AXIS_NUMBER];
	
	/*矫正信息接收标志位*/
	uint8_t receiveFlagX;
	
	uint8_t receiveFlagY;
	
	uint8_t receiveFlagA;
	
	uint8_t correctFlag;
	
	/*经校正后的角度*/
	double sendAngle;
	
}UPara_t;

typedef struct
{
/*定位系统初始数据结构体*/

	//定位系统坐标系->机体坐标系旋转角度（一般都要改这个值）
	double ROTATE_ANGLE;

	//定位系统安装位置距车中心位置矢量y轴坐标
	double PPS2CENTER_Y;
												  
	//定位系统安装位置距车中心位置矢量x轴坐标
	double PPS2CENTER_X;

	//世界坐标系到机体坐标系的X轴正方向夹角，逆时针为正（听运控的）
	double  POSEANGLE;//

	double ORIGINYAW;//存储原始的偏航角，即机器人或设备在初始状态下的偏航角度

	double INSTALLYAW;//存储安装偏航角，即设备在安装时相对于预期方向的偏航角度。
		
	double  PPS2CENTER;//中心点坐标


	//定位系统安装误差：调整至沿x或y轴方向(贴墙移动)移动，全程角度变化为0时，只有x或y坐标变化
	double  CODEWHEELANGLEERR_YAW;
	double  CODEWHEELANGLEERR_PITCH;
			

	//世界坐标系下出发点坐标
	double ORIGINAL_POSX;
	double ORIGINAL_POSY;


}OPara_t;  //Origin

/*POSITIONER数据结构体*/
typedef struct
{
	/*机体坐标系下的坐标和速度信息*/
	double rotatedPosX;
	
	double rotatedPosY;
	
	double rotatedVelX;
	
	double rotatedVelY;
	
	/*世界坐标系下的坐标和速度信息*/
	double rotatedPosX_robort;
	
	double rotatedPosY_robort;
	
	double rotatedVelX_robort;
	
	double rotatedVelY_robort;
	
	double last_RotatedPosX_robort;
	
	double last_RotatedPosY_robort;
	
	double last_rotatedVelX_robort;
	
	double last_rotatedVelY_robort;
	
	double rotatedPosX_robort_inverse;//逆向旋转
	
	double rotatedPosY_robort_inverse;
	
	int timeCounter;
	
	double kalx;
	
	double kaly;
	
	double Kalmodelength;
		
	double Modelength;
	
	double delPos[2];		//正坐标系单位时间位移
	
	int ifadjustR;
	
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
