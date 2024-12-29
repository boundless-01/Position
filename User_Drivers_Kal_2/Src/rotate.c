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
#include"math.h"
#include"string.h"
#include "my_usart.h"

#define Matrix							arm_matrix_instance_f32
#define MatrixInit					arm_mat_init_f32
#define MatrixAdd						arm_mat_add_f32

// ����״̬ά�ȺͲ���ά��
#define STATE_DIM 6  // ״̬����ά�� [x, y, vx, vy, ax, ay]
#define MEAS_DIM 2   // ��������ά�� [x_meas, y_meas]

// �������˲����ṹ��
typedef struct {
    float x[STATE_DIM];           // ״̬���� [x, y, vx, vy, ax, ay]
    float P[STATE_DIM][STATE_DIM]; // ���Э�������
    float F[STATE_DIM][STATE_DIM]; // ״̬ת�ƾ���
    float Q[STATE_DIM][STATE_DIM]; // ��������Э�������
    float H[MEAS_DIM][STATE_DIM];  // ��������
    float R[MEAS_DIM][MEAS_DIM];   // ��������Э�������
    float K[STATE_DIM][MEAS_DIM];  // ����������
    float z[MEAS_DIM];            // �������� [x_meas, y_meas]
} KalmanFilter;

// ��ʼ���������˲���
void Kalman_Init(KalmanFilter *kf, float dt) 
{

    // ��ʼ״̬����
		kf->x[0] = psPara.last_RotatedPosX_robort;
	  kf->x[1] = psPara.last_RotatedPosY_robort;
		kf->x[2] = psPara.last_rotatedVelX_robort;
		kf->x[3] = psPara.last_rotatedVelY_robort;
		kf->x[4] = psPara.imuPara.last_Acc_World[0];
		kf->x[5] = psPara.imuPara.last_Acc_World[1];
	
    // ״̬ת�ƾ��� F
    for (int i = 0; i < STATE_DIM; i++) kf->F[i][i] = 1.0f;
    kf->F[0][2] = dt; kf->F[1][3] = dt;
		kf->F[2][4] = dt; kf->F[3][5] = dt;
    kf->F[0][4] = 0.5f * dt * dt; kf->F[1][5] = 0.5f * dt * dt;

    // �������� H:��ϵͳ��״̬����xת���������z��ƥ���ά�Ⱥ���ʽ
    kf->H[0][0] = 1.0f; kf->H[1][1] = 1.0f;

    // �������� Q
    float q = 0.1f;
    for (int i = 0; i < STATE_DIM; i++) kf->Q[i][i] = q;

    // �������� R
    if(psPara.ifadjustR == 0)
    kf->R[0][0] = 0.05f; kf->R[1][1] = 0.05f;
		if(psPara.ifadjustR == 1)
    kf->R[0][0] = 0.1f; kf->R[1][1] = 0.1f;
		if(psPara.ifadjustR == 2)
    kf->R[0][0] = 0.2f; kf->R[1][1] = 0.2f;
		if(psPara.ifadjustR == 3)
    kf->R[0][0] = 0.3f; kf->R[1][1] = 0.3f;
		if(psPara.ifadjustR == 4)
    kf->R[0][0] = 0.4f; kf->R[1][1] = 0.4f;

    // ��ʼ�����Э���� P����ʾ��ʼ״̬�� ��ȷ����Ϊ1���Ҽ������״̬����֮��û������ԣ�Э����Ϊ 0����
    for (int i = 0; i < STATE_DIM; i++) kf->P[i][i] = 1.0f;
}

// ����ת�ú��� (�� H^T)
void Matrix_Transpose(float src[MEAS_DIM][STATE_DIM], float dst[STATE_DIM][MEAS_DIM]) 
{
    for (int i = 0; i < MEAS_DIM; i++) 
	  {
        for (int j = 0; j < STATE_DIM; j++) 
				{
            dst[j][i] = src[i][j];
        }
    }
}

// ������������㺯�� K = P H^T (H P H^T + R)^(-1)
void Kalman_ComputeGain(KalmanFilter *kf) 
{
    static float HT[STATE_DIM][MEAS_DIM] = {0};
    Matrix_Transpose(kf->H, HT);

    static float S[MEAS_DIM][MEAS_DIM] = {0}; // S = H P H^T + R

    // S = H * P * H^T + R
    for (int i = 0; i < MEAS_DIM; i++) 
		{
        for (int j = 0; j < MEAS_DIM; j++)
 			 {
            S[i][j] = kf->R[i][j];
            for (int k = 0; k < STATE_DIM; k++) 
				    {
                for (int l = 0; l < STATE_DIM; l++)
  							{
                    S[i][j] += kf->H[i][k] * kf->P[k][l] * HT[l][j];
                }
            }
        }
    }

    // �򻯼��� S ���� (����Ϊ�ԽǾ���)
    static float S_inv[MEAS_DIM][MEAS_DIM] = {0};
    for (int i = 0; i < MEAS_DIM; i++)
		{
        S_inv[i][i] = 1.0f / S[i][i];
    }

    // K = P * H^T * S^(-1)
    for (int i = 0; i < STATE_DIM; i++)
   {
        for (int j = 0; j < MEAS_DIM; j++)
				{
            kf->K[i][j] = 0;
            for (int k = 0; k < MEAS_DIM; k++) 
						{
                for (int l = 0; l < STATE_DIM; l++) 
								{
                    kf->K[i][j] += kf->P[i][l] * HT[l][k] * S_inv[k][j];
                }
            }
        }
    }
}

// �������˲���Ԥ�ⲽ��
void Kalman_Predict(KalmanFilter *kf) 
{
    static float x_new[STATE_DIM] = {0};

    // ״̬Ԥ�� x = F * x
    for (int i = 0; i < STATE_DIM; i++) 
		{
        for (int j = 0; j < STATE_DIM; j++) 
				{
            x_new[i] += kf->F[i][j] * kf->x[j];
        }
    }
    memcpy(kf->x, x_new, sizeof(x_new));
}

// �������˲������²���
void Kalman_Update(KalmanFilter *kf, float z_meas[MEAS_DIM]) 
{
    static float y[MEAS_DIM] = {0};  // �����в� y = z - H * x

    // ��������в�
    for (int i = 0; i < MEAS_DIM; i++) 
		{
        y[i] = z_meas[i];
        for (int j = 0; j < STATE_DIM; j++) 
				{
            y[i] -= kf->H[i][j] * kf->x[j];
        }
    }

    // ���㿨�������� K
    Kalman_ComputeGain(kf);

    // ����״̬���� x = x + K * y
    for (int i = 0; i < STATE_DIM; i++)
		{
        for (int j = 0; j < MEAS_DIM; j++)
				{
            kf->x[i] += kf->K[i][j] * y[j];
        }
    }

    // �������Э���� P = (I - K H) P
    static float KH[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++)
		{
        for (int j = 0; j < STATE_DIM; j++)
				{
            KH[i][j] = 0;
            for (int k = 0; k < MEAS_DIM; k++) 
						{
                KH[i][j] += kf->K[i][k] * kf->H[k][j];
            }
        }
    }

    for (int i = 0; i < STATE_DIM; i++) 
		{
        for (int j = 0; j < STATE_DIM; j++) 
				{
            kf->P[i][j] -= KH[i][j] * kf->P[i][j];
        }
    }
}

void  Kalman(double* x, double* y)
{
		KalmanFilter kf;
    float dt = 0.05f;
    Kalman_Init(&kf, dt);

		float z[2] = {*x,*y};
	
    Kalman_Predict(&kf);
    Kalman_Update(&kf, z);
}

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
//Matrix�Ǿ������˼
	
    float exchangeXY_Array[4]= {0,1,1,0};

		//��ת����,��ԭ����ϵ˳ʱ����ת-135��
    float rotateArray[4]= {arm_cos_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f),
													 arm_sin_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f),
													 -arm_sin_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f),
													 arm_cos_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f)};

    MatrixInit(&exchangeXY,2,2,exchangeXY_Array);//��ʼ������
		//exchangeXY�Ǿ������� 2,2�Ǿ���ߴ磻 exchangeXY_Array�����������Ԫ��
		//��һ��2��2�У��ڶ���2��2�У��������ȸ��и�ֵ
    MatrixInit(&rotate,2,2,rotateArray);

    if(isExchangeXY) //����Ƿ�Ҫ����XY
        MatrixMuiltiple(&rotate,&exchangeXY,&rotate);	
		//�� exchangeXY ����� rotate ������ˣ���������洢�� rotate ����

    Matrix sendXY;
    Matrix selfXY;

    float sendXY_Array[2]= {0,0};
    float selfXY_Array[2]= {psPara.ePara.posx,psPara.ePara.posy};

    MatrixInit(&sendXY,2,1,sendXY_Array);
    MatrixInit(&selfXY,2,1,selfXY_Array);

    arm_mat_mult_f32(&rotate,&selfXY,&sendXY);//��������ˣ�����浽������������

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
void adjustR()
{
		psPara.kalx = fabs(psPara.last_rotatedVelX_robort * 0.005) + 0.5 * fabs(psPara.imuPara.last_Acc_World[0] * 0.005 * 0.005);
		psPara.kaly = fabs(psPara.last_rotatedVelY_robort * 0.005) + 0.5 * fabs(psPara.imuPara.last_Acc_World[1] * 0.005 * 0.005);

		psPara.Modelength = sqrtf(psPara.delPos[0] * psPara.delPos[0] + psPara.delPos[1] * psPara.delPos[1]);//������˲ʱģ��
	  psPara.Kalmodelength = sqrtf(psPara.kalx * psPara.kalx + psPara.kaly * psPara.kaly);//Ԥ���˲ʱģ��

		if(fabs(psPara.Kalmodelength - psPara.Modelength) > 0.005) 
	 {
		 psPara.ifadjustR++;
		 if(psPara.ifadjustR >= 4) psPara.ifadjustR = 4;
	 }
	 if(fabs(psPara.Kalmodelength - psPara.Modelength) <= 0.005) 
	 {
		 psPara.ifadjustR--;
		 if(psPara.ifadjustR <= 0) psPara.ifadjustR = 0;
	 }
}
void RotatePos2Need(float POSEANGLE,float pps2Center)
{
    //����λϵͳ����ϵѡ��Ϊ��������ϵ
    psPara.rotatedPosX_robort = (psPara.rotatedPosX*cosf(POSEANGLE/180.f*PI)) - psPara.rotatedPosY*sinf(POSEANGLE/180.f*PI);
    psPara.rotatedPosY_robort = (psPara.rotatedPosX*sinf(POSEANGLE/180.f*PI)) + psPara.rotatedPosY*cosf(POSEANGLE/180.f*PI);

//    //����λϵͳ����ת��Ϊ����������
//    psPara.rotatedPosX_robort = psPara.rotatedPosX_robort - pps2Center*(cosf((0.f+psPara.uPara.correctAngle[2]+psPara.imuPara.imuResult_Angle[2])/180.f*PI) - cosf((0.f+psPara.uPara.correctAngle[2])/180.f*PI));
//    psPara.rotatedPosY_robort = psPara.rotatedPosY_robort - pps2Center*(sinf((0.f+psPara.uPara.correctAngle[2]+psPara.imuPara.imuResult_Angle[2])/180.f*PI) - sinf((0.f+psPara.uPara.correctAngle[2])/180.f*PI) );

	  psPara.rotatedPosX_robort += psPara.uPara.correctX;
    psPara.rotatedPosY_robort += psPara.uPara.correctY;
	
	  psPara.rotatedPosX_robort += psPara.oPara.PPS2CENTER * sinf((180.f-psPara.uPara.sendAngle)/180.f*PI);
	  psPara.rotatedPosY_robort += psPara.oPara.PPS2CENTER * cosf((180.f-psPara.uPara.sendAngle)/180.f*PI);
	
		adjustR();
	
	//	if(psPara.ePara.encoderMileage[0] > 0.05 && psPara.ePara.encoderMileage[1] > 0.05)
		Kalman(&(psPara.rotatedPosX_robort), &(psPara.rotatedPosY_robort));
	
		for(int axis = 0; axis < 3; axis++)
		{
			psPara.imuPara.last_Acc_World[axis] = psPara.imuPara.Acc_World[axis];
		}
	
		psPara.last_rotatedVelX_robort = psPara.rotatedVelX_robort;
		psPara.last_rotatedVelY_robort = psPara.rotatedVelY_robort;
		
    psPara.last_RotatedPosX_robort =psPara.rotatedPosX_robort;
    psPara.last_RotatedPosY_robort =psPara.rotatedPosY_robort;
}


