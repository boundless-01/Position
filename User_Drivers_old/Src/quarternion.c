/**
  ******************************************************************************
  * @file    quarternion.c
  * @author  Lxy Zlq  XFR HJN/PLG/LXQ YQY
  * @version Action2.0
  * @date    
  * @brief   ��Ԫ������
  ******************************************************************************
**/

#include "quarternion.h"

/**
* @brief  ��ŷ����ת��Ϊ��Ԫ��
* @param  quaternion: ��Ҫת����ŷ����
* @retval ��Ԫ����Ӧ����Ԫ��
*/
void Euler_to_Quaternion(const double Rad[3],double quaternion[4])
{
	quaternion[0]=arm_cos_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)+arm_sin_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[1]=arm_sin_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)-arm_cos_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[2]=arm_cos_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2)+arm_sin_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2);
	quaternion[3]=arm_cos_f32(Rad[0]/2)*arm_cos_f32(Rad[1]/2)*arm_sin_f32(Rad[2]/2)-arm_sin_f32(Rad[0]/2)*arm_sin_f32(Rad[1]/2)*arm_cos_f32(Rad[2]/2);
}

/*���ǹ��Ե����ϵ��㷨���˼���  z  x  y ��ת�ģ�����x��-90��90*/
/**
* @brief  ����Ԫ��ת��Ϊŷ����
* @param  quaternion: ��Ҫת������Ԫ��
* @retval ��Ԫ����Ӧ��ŷ����
*/
/*����Ƕ���ֵ*/
#define SINGUALAR_ANGLE_THRESHOLD      0.000976f
void Quaternion_to_Euler(const double quaternion[4],double Rad[3] )
{
	double q0,q1,q2,q3;
	double sum;
	signed char sign=1;
	

	#ifdef MINUS_START_ANGLE
		q0=quaternion[0]*allPara.sDta.startquaternion[0] + quaternion[1]*allPara.sDta.startquaternion[1] + quaternion[2]*allPara.sDta.startquaternion[2] + quaternion[3]*allPara.sDta.startquaternion[3];
    q1=quaternion[0]*allPara.sDta.startquaternion[1] - quaternion[1]*allPara.sDta.startquaternion[0] - quaternion[2]*allPara.sDta.startquaternion[3] + quaternion[3]*allPara.sDta.startquaternion[2];
    q2=quaternion[0]*allPara.sDta.startquaternion[2] - quaternion[2]*allPara.sDta.startquaternion[0] - quaternion[3]*allPara.sDta.startquaternion[1] + quaternion[1]*allPara.sDta.startquaternion[3];
    q3=quaternion[0]*allPara.sDta.startquaternion[3] - quaternion[3]*allPara.sDta.startquaternion[0] - quaternion[1]*allPara.sDta.startquaternion[2] + quaternion[2]*allPara.sDta.startquaternion[1];
	#else
	//����ϵת����ϵ
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
  
	if(fabs(-q1*q3 + q0*q2) <= 0.5f - SINGUALAR_ANGLE_THRESHOLD)
	{
		Rad[0] = atan2(2 * q0 * q1 + 2 * q2 * q3 , q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0);
		Rad[1] = safe_asin(2.0 * (-q1 * q3 + q0 * q2) );
		Rad[2] = atan2(2 * q1 * q2 + 2 * q0 * q3 , -q2 * q2 - q3 * q3 + q0 * q0 + q1 * q1);
	}
	else /*����Ƕ� ��rollaxis����yaw��ƽ��ʱ*/
	{
		Rad[0]= 0.f;
		if((-q1*q3 + q0*q2)<0)
		{
			sign=-1;
		}
		Rad[1]= sign*PI/2;
		Rad[2]=	-2*sign*atan2(q1,q0);
	}
}

double lastDifQuaternion[4] = {0.f};
void QuaternionInt(double quaternion[4],double data[3] )
{
	#ifndef CHANGE_RUNGE_KUTTA	//����������㷽��//����
	static double old_w[3];
  double dif_quarterion_f[4];
	double dif_quarterion_l[4];
	double med_quarterion[4];
	
	/*������Ԫ������*/
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


