/**
  ******************************************************************************
  * @file    figurePos.c
  * @author  Lxy Zlq  XFR HJN/PLG/LXQ YQY
  * @version Action2.0
  * @date    
  * @brief   定位系统坐标积分
  ******************************************************************************
**/

#include "figurePos.h"
#include "quarternion.h"
/********************************************************
函数功能：定位系统自身坐标系下坐标积分                
入口参数：
返回  值：
*********************************************************/
void CalculatePos(void)//位置积分
{
	double real[2]={0.0,0.0};		//斜坐标系的正交坐标位移
	//double delPos[2]={0.0,0.0};		//正坐标系单位时间位移

	double zangle=0.0;				//两次采样期间的中间角度
	
	zangle = psPara.imuPara.imuResult_Angle[2]; 		//z轴积分角度
	/*取两次角度的中间值由于坐标积分，-180到180*/
	if( (zangle-psPara.imuPara.lastAngle[2]) > 180.0)   //相当于刻度，类比encoder的算法
	{
		zangle = (zangle + psPara.imuPara.lastAngle[2]-360.0)/2.0;//取平均
	} 
	else if(zangle-psPara.imuPara.lastAngle[2]<-180.0)
	{
		zangle = (zangle+psPara.imuPara.lastAngle[2]+360.0)/2.0;
	}
	else
	{
		zangle=(zangle+psPara.imuPara.lastAngle[2])/2;							
	}
	if(zangle>180.0)
	{
		zangle -= 360.0;
	}
	else if(zangle<-180.0)
	{
		zangle+=360.0;
	}
	psPara.imuPara.lastAngle[2]=psPara.imuPara.imuResult_Angle[2];
	
	//直角坐标系和非直角坐标系的转换  一定要注意坐标系的正方向和角度正方向一样！               								
	//以x为标准
	real[0] = psPara.ePara.encoderMileage[0];	//两从动轮正交位移,real[0]表示x轴，1则表示y轴。（改方向就添加+ -号，后面delPos也要改）
	real[1] = 1.0/cos(0 * PI/180.0f) * psPara.ePara.encoderMileage[1]\
			- tan(0 * PI/180.0f) * psPara.ePara.encoderMileage[0];
	//角度的影响；X轴位移在y轴方向上的分量

	
//旋转矩阵{cos,sin,-sin,cos}：顺时针———注意：zangle以逆时针为正，即存在相对关系
	 psPara.delPos[0]=(cos(zangle*0.017453292519943)*real[0]+sin(zangle*0.017453292519943)*real[1]);
	//0.017453292519943 是 PI/180
	 psPara.delPos[1]=(-sin(zangle*0.017453292519943)*real[0]+cos(zangle*0.017453292519943)*real[1]);
	/*
	逆时针：
	x1=xcos(β)-ysin(β);
	y1=ycos(β)+xsin(β);
	顺时针：
	x1=xcos(β)+ysin(β);
	y1=ycos(β)-xsin(β);
	*/
	psPara.ePara.posx += psPara.delPos[0];
	psPara.ePara.posy += psPara.delPos[1];
	
	/*获得定位系统x，y方向上的速度*/
	psPara.ePara.vellx = psPara.delPos[0]*200.f;	//单位mm/s
	psPara.ePara.velly = psPara.delPos[1]*200.f;

}

/********************************************************
函数功能：将编码轮采集的本周期旋转角度转换为实际距离                
入口参数：
返回  值：
*********************************************************/
void FigureVell(void)
{
  int difEncoder[2] = {0};//存储编码器数据的差异
		
	//第一次判断静止时可以不用判断角速度
	psPara.ePara.encoderDATA[0]=MT6835ReadAbsPos_X();//从MT6835传感器读取21位角度数据的函数
	psPara.ePara.encoderDATA[1]=MT6835ReadAbsPos_Y();//从MT6835传感器读取21位角度数据的函数
	
	/*前几次采集encoder时还没开始里程积分，当开始积分时lastEncdoer应该是等于encoderData，
	若一开始直接积分要注意初始化lastEncoder一开始为0而encoder不是0而造成的初始位置误差*/
	difEncoder[0] = psPara.ePara.encoderDATA[0] - psPara.ePara.lastEncoder[0];
	difEncoder[1] = psPara.ePara.encoderDATA[1] - psPara.ePara.lastEncoder[1];
	psPara.ePara.lastEncoder[0] = psPara.ePara.encoderDATA[0];//把这次的值赋给上次的值，为下次计算做准备
	psPara.ePara.lastEncoder[1] = psPara.ePara.encoderDATA[1];
	
	//把vell限制在-1048576~1048576，实际单次读值范围在半圈之内
	if(difEncoder[0]>1048576)
	{
		difEncoder[0] -= 2097152;
	}
	else if(difEncoder[0] < -1048576)
	{
		difEncoder[0] += 2097152;
	}
	if(difEncoder[1]>1048576)
	{
		difEncoder[1] -= 2097152;
	}
	else if(difEncoder[1] < -1048576)
	{
		difEncoder[1] += 2097152;
	}
	
	//比例因子 2097152.0（2^21，因为编码器是21位的）
	psPara.ePara.encoderSum[0] += difEncoder[0];//累加X轴编码器数据差异
	psPara.ePara.encoderSum[1] += difEncoder[1];//累加Y轴编码器数据差异
	psPara.ePara.encoderMileage[0] = difEncoder[0]/2097152.0 * 2.0 * PI * R_WHEEL_X;//份数乘周长
	psPara.ePara.encoderMileage[1] = difEncoder[1]/2097152.0 * 2.0 * PI * R_WHEEL_Y;
}

/********************************************************
函数功能：将校正信息进行更新                
入口参数：
返回  值：
*********************************************************/
void CorrectHandler(void)
{
	//收到校正信息
	if(psPara.uPara.receiveFlagX || psPara.uPara.receiveFlagY )
	{
		static double euler[3];//存储欧拉角
		//将原有信息进行清零，并重新赋值
		psPara.ePara.posx = 0.f;
		psPara.ePara.posy = 0.f;
		psPara.uPara.correctX = psPara.last_RotatedPosX_robort;
		psPara.uPara.correctY = psPara.last_RotatedPosY_robort;
//		psPara.uPara.correctAngle[2] += psPara.imuPara.imuResult_Angle[2];
		psPara.imuPara.imuResult_Angle[2] = 0.f;
		euler[0] = psPara.imuPara.imuResult_Angle[0] / 180.f *PI;
		euler[1] = psPara.imuPara.imuResult_Angle[1] / 180.f *PI;
		euler[2] = 0 / 180.f *PI;
		Euler_to_Quaternion(euler, psPara.imuPara.quaternion);//欧拉角转四元数
		
		//根据收到的校正信息修改校正量
		if(psPara.uPara.receiveFlagA && psPara.uPara.receiveFlagX)
		{
			psPara.uPara.correctAngle[2] = psPara.uPara.newCorrectAngle[2];
			psPara.uPara.correctX = psPara.uPara.newCorrectX - psPara.oPara.PPS2CENTER * sinf(psPara.uPara.correctAngle[2]/180.f*PI);
			psPara.uPara.receiveFlagX = FALSE;
			psPara.uPara.correctFlag = TRUE;
		}
		
		if(psPara.uPara.receiveFlagA && psPara.uPara.receiveFlagY)
		{
			psPara.uPara.correctAngle[2] = psPara.uPara.newCorrectAngle[2];
			psPara.uPara.correctY = psPara.uPara.newCorrectY + psPara.oPara.PPS2CENTER * cosf(psPara.uPara.correctAngle[2]/180.f*PI);
			psPara.uPara.receiveFlagY = FALSE;
			psPara.uPara.correctFlag = TRUE;
		}
		
//		if(psPara.uPara.receiveFlagA)
//		{
//			psPara.uPara.correctAngle[2] = psPara.uPara.newCorrectAngle[2];
//			psPara.uPara.receiveFlagA = FALSE;
//		}
		
		
		//限制角度范围
		if(psPara.uPara.correctAngle[2] > 180.0)
		{
			psPara.uPara.correctAngle[2] -= 360.0;
		}
		else if(psPara.uPara.correctAngle[2] < -180.0)
		{
			psPara.uPara.correctAngle[2] += 360.0;
		}
	}
}

void SetPosX(double in)
{
  psPara.ePara.posx=in;
}
void SetPosY(double in)
{
  psPara.ePara.posy=in;
}


