/**
  ******************************************************************************
  * @file    quarternion.c
  * @author  Lxy Zlq  XFR HJN/PLG/LXQ YQY
  * @version Action2.0
  * @date    
  * @brief   四元数计算
  ******************************************************************************
**/

#include "quarternion.h"

/**
* @brief  将欧拉角转换为四元数
* @param  quaternion: 需要转换的欧拉角
* @retval 四元数对应的四元数
*/
void Euler_to_Quaternion(const double Rad[3],double quaternion[4])//欧拉角转四元数
{
	quaternion[0]=arm_cos_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)+arm_sin_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[1]=arm_sin_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)-arm_cos_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[2]=arm_cos_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)+arm_sin_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[3]=arm_cos_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2)-arm_sin_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2);
}

/*这是惯性导航上的算法，人家是  z  x  y 旋转的，所以x是-90到90*/
/**
* @brief  将四元数转换为欧拉角
* @param  quaternion: 需要转换的四元数
* @retval 四元数对应的欧拉角
*/
/*奇异角度阈值*/
#define SINGUALAR_ANGLE_THRESHOLD      0.000976f
void Quaternion_to_Euler(const double quaternion[4],double Rad[3] )
{
	double q0,q1,q2,q3;//存储四元数的分量
	double sum;//存储中间计算结果
	signed char sign=1;//确定旋转的符号
	

	#ifdef MINUS_START_ANGLE//检查是否定义了 MINUS_START_ANGLE
		q0=quaternion[0]*allPara.sDta.startquaternion[0] + quaternion[1]*allPara.sDta.startquaternion[1] + quaternion[2]*allPara.sDta.startquaternion[2] + quaternion[3]*allPara.sDta.startquaternion[3];
    q1=quaternion[0]*allPara.sDta.startquaternion[1] - quaternion[1]*allPara.sDta.startquaternion[0] - quaternion[2]*allPara.sDta.startquaternion[3] + quaternion[3]*allPara.sDta.startquaternion[2];
    q2=quaternion[0]*allPara.sDta.startquaternion[2] - quaternion[2]*allPara.sDta.startquaternion[0] - quaternion[3]*allPara.sDta.startquaternion[1] + quaternion[1]*allPara.sDta.startquaternion[3];
    q3=quaternion[0]*allPara.sDta.startquaternion[3] - quaternion[3]*allPara.sDta.startquaternion[0] - quaternion[1]*allPara.sDta.startquaternion[2] + quaternion[2]*allPara.sDta.startquaternion[1];
	#else
	//左手系转右手系
	q0= quaternion[0];
	q1= quaternion[1];
	q2= quaternion[2];
	q3= quaternion[3];
	#endif
  
	sum = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 / sum;
	q1 = q1 / sum;
	q2 = q2 / sum;
	q3 = q3 / sum;
  
	if(fabs(-q1*q3 + q0*q2) <= 0.5f - SINGUALAR_ANGLE_THRESHOLD)//检查四元数(Pitch轴)是否接近奇异角度：若是,则不接近
	{
		Rad[0] = atan2(2 * q0 * q1 + 2 * q2 * q3 , q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0);//计算偏航角Yaw(Z)
		Rad[1] = safe_asin(2.0 * (-q1 * q3 + q0 * q2) );//计算俯仰角Pitch(Y)
		Rad[2] = atan2(2 * q1 * q2 + 2 * q0 * q3 , -q2 * q2 - q3 * q3 + q0 * q0 + q1 * q1);//计算横滚角Roll(X)
	}
	else /*奇异角度 当rollaxis轴与yaw轴平行时*/
	{
		Rad[0]= 0.f;//将偏航角（Yaw）设置为 0，这是因为在奇异角度下，偏航角的值无法确定
		if((-q1*q3 + q0*q2)<0)//确定俯仰角（Pitch）的符号，如果 -q1*q3 + q0*q2 小于 0，表示俯仰角为正
		{
			sign=-1;//表示俯仰角为正
		}
		Rad[1]= sign*PI/2;//设置俯仰角（Pitch）为 π/2（90°）或 -π/2（-90°）
		Rad[2]=	-2*sign*atan2(q1,q0);//计算滚转角（Roll）
	}
}

double lastDifQuaternion[4] = {0.f};
void QuaternionInt(double quaternion[4],double data[3] )
{
	#ifndef CHANGE_RUNGE_KUTTA	//龙格库塔计算方法//峰哥版
	static double old_w[3];
  double dif_quarterion_f[4];
	double dif_quarterion_l[4];
	double med_quarterion[4];
	
	/*计算四元数导数*/
	dif_quarterion_f[0]=(-quaternion[1]*old_w[0] - quaternion[2]*old_w[1] - quaternion[3]*old_w[2])*0.5f;
	dif_quarterion_f[1]=( quaternion[0]*old_w[0] + quaternion[2]*old_w[2] - quaternion[3]*old_w[1])*0.5f;
	dif_quarterion_f[2]=( quaternion[0]*old_w[1] - quaternion[1]*old_w[2] + quaternion[3]*old_w[0])*0.5f;
	dif_quarterion_f[3]=( quaternion[0]*old_w[2] + quaternion[1]*old_w[1] - quaternion[2]*old_w[0])*0.5f;
	
	med_quarterion[0]=quaternion[0]+dif_quarterion_f[0]*dT;
	med_quarterion[1]=quaternion[1]+dif_quarterion_f[1]*dT;
	med_quarterion[2]=quaternion[2]+dif_quarterion_f[2]*dT;
	med_quarterion[3]=quaternion[3]+dif_quarterion_f[3]*dT; 
  
	dif_quarterion_l[0]=(-med_quarterion[1]*data[0] - med_quarterion[2]*data[1] - med_quarterion[3]*data[2])*0.5f;
	dif_quarterion_l[1]=( med_quarterion[0]*data[0] + med_quarterion[2]*data[2] - med_quarterion[3]*data[1])*0.5f;
	dif_quarterion_l[2]=( med_quarterion[0]*data[1] - med_quarterion[1]*data[2] + med_quarterion[3]*data[0])*0.5f;
	dif_quarterion_l[3]=( med_quarterion[0]*data[2] + med_quarterion[1]*data[1] - med_quarterion[2]*data[0])*0.5f;
	
	
	quaternion[0]=quaternion[0]+0.5f*(dif_quarterion_f[0]+dif_quarterion_l[0])*dT;
	quaternion[1]=quaternion[1]+0.5f*(dif_quarterion_f[1]+dif_quarterion_l[1])*dT;
	quaternion[2]=quaternion[2]+0.5f*(dif_quarterion_f[2]+dif_quarterion_l[2])*dT;
	quaternion[3]=quaternion[3]+0.5f*(dif_quarterion_f[3]+dif_quarterion_l[3])*dT;
	
	
	old_w[0]=data[0];
	old_w[2]=data[2];
	old_w[1]=data[1];
	#endif
}


