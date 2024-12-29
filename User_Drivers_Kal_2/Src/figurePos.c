/**
  ******************************************************************************
  * @file    figurePos.c
  * @author  Lxy Zlq  XFR HJN/PLG/LXQ YQY
  * @version Action2.0
  * @date    
  * @brief   ��λϵͳ�������
  ******************************************************************************
**/

#include "figurePos.h"
#include "quarternion.h"
/********************************************************
�������ܣ���λϵͳ��������ϵ���������                
��ڲ�����
����  ֵ��
*********************************************************/
void CalculatePos(void)//λ�û���
{
	double real[2]={0.0,0.0};		//б����ϵ����������λ��
	//double delPos[2]={0.0,0.0};		//������ϵ��λʱ��λ��

	double zangle=0.0;				//���β����ڼ���м�Ƕ�
	
	zangle = psPara.imuPara.imuResult_Angle[2]; 		//z����ֽǶ�
	/*ȡ���νǶȵ��м�ֵ����������֣�-180��180*/
	if( (zangle-psPara.imuPara.lastAngle[2]) > 180.0)   //�൱�ڿ̶ȣ����encoder���㷨
	{
		zangle = (zangle + psPara.imuPara.lastAngle[2]-360.0)/2.0;//ȡƽ��
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
	
	//ֱ������ϵ�ͷ�ֱ������ϵ��ת��  һ��Ҫע������ϵ��������ͽǶ�������һ����               								
	//��xΪ��׼
	real[0] = psPara.ePara.encoderMileage[0];	//���Ӷ�������λ��,real[0]��ʾx�ᣬ1���ʾy�ᡣ���ķ�������+ -�ţ�����delPosҲҪ�ģ�
	real[1] = 1.0/cos(0 * PI/180.0f) * psPara.ePara.encoderMileage[1]\
			- tan(0 * PI/180.0f) * psPara.ePara.encoderMileage[0];
	//�Ƕȵ�Ӱ�죻X��λ����y�᷽���ϵķ���

	
//��ת����{cos,sin,-sin,cos}��˳ʱ�롪����ע�⣺zangle����ʱ��Ϊ������������Թ�ϵ
	 psPara.delPos[0]=(cos(zangle*0.017453292519943)*real[0]+sin(zangle*0.017453292519943)*real[1]);
	//0.017453292519943 �� PI/180
	 psPara.delPos[1]=(-sin(zangle*0.017453292519943)*real[0]+cos(zangle*0.017453292519943)*real[1]);
	/*
	��ʱ�룺
	x1=xcos(��)-ysin(��);
	y1=ycos(��)+xsin(��);
	˳ʱ�룺
	x1=xcos(��)+ysin(��);
	y1=ycos(��)-xsin(��);
	*/
	psPara.ePara.posx += psPara.delPos[0];
	psPara.ePara.posy += psPara.delPos[1];
	
	/*��ö�λϵͳx��y�����ϵ��ٶ�*/
	psPara.ePara.vellx = psPara.delPos[0]*200.f;	//��λmm/s
	psPara.ePara.velly = psPara.delPos[1]*200.f;

}

/********************************************************
�������ܣ��������ֲɼ��ı�������ת�Ƕ�ת��Ϊʵ�ʾ���                
��ڲ�����
����  ֵ��
*********************************************************/
void FigureVell(void)
{
  int difEncoder[2] = {0};//�洢���������ݵĲ���
		
	//��һ���жϾ�ֹʱ���Բ����жϽ��ٶ�
	psPara.ePara.encoderDATA[0]=MT6835ReadAbsPos_X();//��MT6835��������ȡ21λ�Ƕ����ݵĺ���
	psPara.ePara.encoderDATA[1]=MT6835ReadAbsPos_Y();//��MT6835��������ȡ21λ�Ƕ����ݵĺ���
	
	/*ǰ���βɼ�encoderʱ��û��ʼ��̻��֣�����ʼ����ʱlastEncdoerӦ���ǵ���encoderData��
	��һ��ʼֱ�ӻ���Ҫע���ʼ��lastEncoderһ��ʼΪ0��encoder����0����ɵĳ�ʼλ�����*/
	difEncoder[0] = psPara.ePara.encoderDATA[0] - psPara.ePara.lastEncoder[0];
	difEncoder[1] = psPara.ePara.encoderDATA[1] - psPara.ePara.lastEncoder[1];
	psPara.ePara.lastEncoder[0] = psPara.ePara.encoderDATA[0];//����ε�ֵ�����ϴε�ֵ��Ϊ�´μ�����׼��
	psPara.ePara.lastEncoder[1] = psPara.ePara.encoderDATA[1];
	
	//��vell������-1048576~1048576��ʵ�ʵ��ζ�ֵ��Χ�ڰ�Ȧ֮��
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
	
	//�������� 2097152.0��2^21����Ϊ��������21λ�ģ�
	psPara.ePara.encoderSum[0] += difEncoder[0];//�ۼ�X����������ݲ���
	psPara.ePara.encoderSum[1] += difEncoder[1];//�ۼ�Y����������ݲ���
	psPara.ePara.encoderMileage[0] = difEncoder[0]/2097152.0 * 2.0 * PI * R_WHEEL_X;//�������ܳ�
	psPara.ePara.encoderMileage[1] = difEncoder[1]/2097152.0 * 2.0 * PI * R_WHEEL_Y;
}

/********************************************************
�������ܣ���У����Ϣ���и���                
��ڲ�����
����  ֵ��
*********************************************************/
void CorrectHandler(void)
{
	//�յ�У����Ϣ
	if(psPara.uPara.receiveFlagX || psPara.uPara.receiveFlagY )
	{
		static double euler[3];//�洢ŷ����
		//��ԭ����Ϣ�������㣬�����¸�ֵ
		psPara.ePara.posx = 0.f;
		psPara.ePara.posy = 0.f;
		psPara.uPara.correctX = psPara.last_RotatedPosX_robort;
		psPara.uPara.correctY = psPara.last_RotatedPosY_robort;
//		psPara.uPara.correctAngle[2] += psPara.imuPara.imuResult_Angle[2];
		psPara.imuPara.imuResult_Angle[2] = 0.f;
		euler[0] = psPara.imuPara.imuResult_Angle[0] / 180.f *PI;
		euler[1] = psPara.imuPara.imuResult_Angle[1] / 180.f *PI;
		euler[2] = 0 / 180.f *PI;
		Euler_to_Quaternion(euler, psPara.imuPara.quaternion);//ŷ����ת��Ԫ��
		
		//�����յ���У����Ϣ�޸�У����
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
		
		
		//���ƽǶȷ�Χ
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


