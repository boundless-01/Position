/**
  ******************************************************************************
  * @file    rotate.c
  * @author  Lxy Zlq  XFR HJN/PLG/LXQ YQY
  * @version Action2.0
  * @date
  * @brief   ����ϵת������������
  ******************************************************************************
**/

#include "rotate.h"

#define Matrix							arm_matrix_instance_f32
#define MatrixInit					arm_mat_init_f32
#define MatrixAdd						arm_mat_add_f32

void MatrixInverse(Matrix *matrix, Matrix *pSrcB)
{
    Matrix matrixTemp;
    float  Array[4]= {0.0};
    MatrixInit(&matrixTemp,2,2,Array);

    //��Ϊ�����ı�ԭ�����������ｨ��һ��ԭ���󸱱���������
    MatrixAdd(matrix,&matrixTemp,&matrixTemp);

    arm_mat_inverse_f32(&matrixTemp,pSrcB);

}

void MatrixMuiltiple(Matrix * pSrcA,Matrix * pSrcB,Matrix * pSrcC)
{
    Matrix matrixTempA;
    float  ArrayA[4]= {0.0};
    MatrixInit(&matrixTempA,2,2,ArrayA);
    Matrix matrixTempB;
    float  ArrayB[4]= {0.0};
    MatrixInit(&matrixTempB,2,2,ArrayB);

    //��Ϊ�����ı�ԭ�����������ｨ��һ��ԭ���󸱱���������
    MatrixAdd(pSrcA,&matrixTempA,&matrixTempA);

    //��Ϊ�����ı�ԭ�����������ｨ��һ��ԭ���󸱱���������
    MatrixAdd(pSrcB,&matrixTempB,&matrixTempB);

    //�ڽ��о��������ʱ��������Ӻͽ��һ�����ͻ������⣬��Ϊ���߹���һƬ�ڴ�
    arm_mat_mult_f32(&matrixTempA,&matrixTempB,pSrcC);
}

//�Ƿ�Ҫ����XY
static int isExchangeXY=1;

/*ͨ��������ٶȸ��������ٶ�*/
void RotateLevel1(void)
{
    Matrix exchangeXY;
    Matrix rotate;

    float exchangeXY_Array[4]= {0,1,1,0};

    float rotateArray[4]= {arm_cos_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f),arm_sin_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f),-arm_sin_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f),arm_cos_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f)};

    MatrixInit(&exchangeXY,2,2,exchangeXY_Array);
    MatrixInit(&rotate,2,2,rotateArray);

    if(isExchangeXY)
        MatrixMuiltiple(&rotate,&exchangeXY,&rotate);	//�������

    Matrix sendXY;
    Matrix selfXY;

    float sendXY_Array[2]= {0,0};
    float selfXY_Array[2]= {psPara.ePara.posx,psPara.ePara.posy};

    MatrixInit(&sendXY,2,1,sendXY_Array);
    MatrixInit(&selfXY,2,1,selfXY_Array);

    arm_mat_mult_f32(&rotate,&selfXY,&sendXY);

    psPara.rotatedPosX=sendXY_Array[0];
    psPara.rotatedPosY=sendXY_Array[1];

    RotatePos2Need( psPara.uPara.correctAngle[2],psPara.oPara.PPS2CENTER);
	
	//	RotatePos2Need( psPara.uPara.correctAngle[2]);
}

void UpdateTalkInfo(void)
{
    static float xLast=0.f;
    static float yLast=0.f;

    RotateLevel1();

    //���յ�У����Ϣ������������������ڻ��нϴ��ֵ������ٶ��쳣��������ٶȽ������㴦��
    if(!psPara.uPara.correctFlag)
    {
        psPara.rotatedVelX_robort=(psPara.rotatedPosX_robort-xLast)*200.f;
        psPara.rotatedVelY_robort=(psPara.rotatedPosY_robort-yLast)*200.f;
    }
    else
    {
        psPara.uPara.correctFlag = FALSE;
        psPara.rotatedVelX_robort = 0.f;
        psPara.rotatedVelY_robort = 0.f;
    }

    xLast=psPara.rotatedPosX_robort;
    yLast=psPara.rotatedPosY_robort;

}

/*
*@name	RotatePos2Need
*@brief ����λϵͳ����ϵѡ��Ϊ��������ϵ
*@para	pps2Center   ��λϵͳ����ϵԭ���복���ĵľ��룬��λ��mm
*/
//void RotatePos2Need(float psPara.oPara.POSEANGLE)
//{
void RotatePos2Need(float POSEANGLE,float pps2Center)
{
    //����λϵͳ����ϵѡ��Ϊ��������ϵ
    psPara.rotatedPosX_robort = (psPara.rotatedPosX*cosf(POSEANGLE/180.f*PI)) - psPara.rotatedPosY*sinf(POSEANGLE/180.f*PI);
    psPara.rotatedPosY_robort = (psPara.rotatedPosX*sinf(POSEANGLE/180.f*PI)) + psPara.rotatedPosY*cosf(POSEANGLE/180.f*PI);

//    //����λϵͳ����ת��Ϊ����������
//    psPara.rotatedPosX_robort = psPara.rotatedPosX_robort - pps2Center*(cosf((0.f+psPara.uPara.correctAngle[2]+psPara.imuPara.imuResult_Angle[2])/180.f*PI) - cosf((0.f+psPara.uPara.correctAngle[2])/180.f*PI));
//    psPara.rotatedPosY_robort = psPara.rotatedPosY_robort - pps2Center*(sinf((0.f+psPara.uPara.correctAngle[2]+psPara.imuPara.imuResult_Angle[2])/180.f*PI) - sinf((0.f+psPara.uPara.correctAngle[2])/180.f*PI) );

	psPara.rotatedPosX_robort +=psPara.uPara.correctX;
    psPara.rotatedPosY_robort +=psPara.uPara.correctY;
	
	psPara.rotatedPosX_robort += psPara.oPara.PPS2CENTER * sinf((180.f-psPara.uPara.sendAngle)/180.f*PI);
	psPara.rotatedPosY_robort += psPara.oPara.PPS2CENTER * cosf((180.f-psPara.uPara.sendAngle)/180.f*PI);
	

    psPara.last_RotatedPosX_robort =psPara.rotatedPosX_robort;
    psPara.last_RotatedPosY_robort =psPara.rotatedPosY_robort;
}


