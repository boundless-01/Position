/**
  ******************************************************************************
  * @file    rotate.c
  * @author  Lxy Zlq  XFR HJN/PLG/LXQ YQY
  * @version Action2.0
  * @date
  * @brief   坐标系转换，矩阵运算
  ******************************************************************************
**/

#include "rotate.h"
#include"math.h"
#include"string.h"
#include "my_usart.h"

#define Matrix							arm_matrix_instance_f32
#define MatrixInit					arm_mat_init_f32
#define MatrixAdd						arm_mat_add_f32

// 定义状态维度和测量维度
#define STATE_DIM 6  // 状态向量维度 [x, y, vx, vy, ax, ay]
#define MEAS_DIM 2   // 测量向量维度 [x_meas, y_meas]

// 卡尔曼滤波器结构体
typedef struct {
    float x[STATE_DIM];           // 状态向量 [x, y, vx, vy, ax, ay]
    float P[STATE_DIM][STATE_DIM]; // 误差协方差矩阵
    float F[STATE_DIM][STATE_DIM]; // 状态转移矩阵
    float Q[STATE_DIM][STATE_DIM]; // 过程噪声协方差矩阵
    float H[MEAS_DIM][STATE_DIM];  // 测量矩阵
    float R[MEAS_DIM][MEAS_DIM];   // 测量噪声协方差矩阵
    float K[STATE_DIM][MEAS_DIM];  // 卡尔曼增益
    float z[MEAS_DIM];            // 测量向量 [x_meas, y_meas]
} KalmanFilter;

// 初始化卡尔曼滤波器
void Kalman_Init(KalmanFilter *kf, float dt) 
{

    // 初始状态向量
		kf->x[0] = psPara.last_RotatedPosX_robort;
	  kf->x[1] = psPara.last_RotatedPosY_robort;
		kf->x[2] = psPara.last_rotatedVelX_robort;
		kf->x[3] = psPara.last_rotatedVelY_robort;
		kf->x[4] = psPara.imuPara.last_Acc_World[0];
		kf->x[5] = psPara.imuPara.last_Acc_World[1];
	
    // 状态转移矩阵 F
    for (int i = 0; i < STATE_DIM; i++) kf->F[i][i] = 1.0f;
    kf->F[0][2] = dt; kf->F[1][3] = dt;
		kf->F[2][4] = dt; kf->F[3][5] = dt;
    kf->F[0][4] = 0.5f * dt * dt; kf->F[1][5] = 0.5f * dt * dt;

    // 测量矩阵 H:将系统的状态向量x转换到与测量z相匹配的维度和形式
    kf->H[0][0] = 1.0f; kf->H[1][1] = 1.0f;

    // 过程噪声 Q
    float q = 0.1f;
    for (int i = 0; i < STATE_DIM; i++) kf->Q[i][i] = q;

    // 测量噪声 R
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

    // 初始化误差协方差 P：表示初始状态的 不确定性为1，且假设各个状态变量之间没有相关性（协方差为 0）。
    for (int i = 0; i < STATE_DIM; i++) kf->P[i][i] = 1.0f;
}

// 矩阵转置函数 (简化 H^T)
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

// 卡尔曼增益计算函数 K = P H^T (H P H^T + R)^(-1)
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

    // 简化计算 S 的逆 (假设为对角矩阵)
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

// 卡尔曼滤波器预测步骤
void Kalman_Predict(KalmanFilter *kf) 
{
    static float x_new[STATE_DIM] = {0};

    // 状态预测 x = F * x
    for (int i = 0; i < STATE_DIM; i++) 
		{
        for (int j = 0; j < STATE_DIM; j++) 
				{
            x_new[i] += kf->F[i][j] * kf->x[j];
        }
    }
    memcpy(kf->x, x_new, sizeof(x_new));
}

// 卡尔曼滤波器更新步骤
void Kalman_Update(KalmanFilter *kf, float z_meas[MEAS_DIM]) 
{
    static float y[MEAS_DIM] = {0};  // 测量残差 y = z - H * x

    // 计算测量残差
    for (int i = 0; i < MEAS_DIM; i++) 
		{
        y[i] = z_meas[i];
        for (int j = 0; j < STATE_DIM; j++) 
				{
            y[i] -= kf->H[i][j] * kf->x[j];
        }
    }

    // 计算卡尔曼增益 K
    Kalman_ComputeGain(kf);

    // 更新状态向量 x = x + K * y
    for (int i = 0; i < STATE_DIM; i++)
		{
        for (int j = 0; j < MEAS_DIM; j++)
				{
            kf->x[i] += kf->K[i][j] * y[j];
        }
    }

    // 更新误差协方差 P = (I - K H) P
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

    //因为求逆会改变原矩阵，所以这里建立一个原矩阵副本用作计算
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

    //因为求逆会改变原矩阵，所以这里建立一个原矩阵副本用作计算
    MatrixAdd(pSrcA,&matrixTempA,&matrixTempA);

    //因为求逆会改变原矩阵，所以这里建立一个原矩阵副本用作计算
    MatrixAdd(pSrcB,&matrixTempB,&matrixTempB);

    //在进行矩阵运算的时候，如果乘子和结果一样，就会有问题，因为两者共用一片内存
    arm_mat_mult_f32(&matrixTempA,&matrixTempB,pSrcC);
}

//是否要互换XY
static int isExchangeXY=1;

/*通过电机角速度更新坐标速度*/
void RotateLevel1(void)
{
    Matrix exchangeXY;
    Matrix rotate;
//Matrix是矩阵的意思
	
    float exchangeXY_Array[4]= {0,1,1,0};

		//旋转矩阵,将原坐标系顺时针旋转-135度
    float rotateArray[4]= {arm_cos_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f),
													 arm_sin_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f),
													 -arm_sin_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f),
													 arm_cos_f32(psPara.oPara.ROTATE_ANGLE*PI/180.0f)};

    MatrixInit(&exchangeXY,2,2,exchangeXY_Array);//初始化矩阵
		//exchangeXY是矩阵名； 2,2是矩阵尺寸； exchangeXY_Array用于填充矩阵的元素
		//第一个2是2行，第二个2是2列，所以是先给列赋值
    MatrixInit(&rotate,2,2,rotateArray);

    if(isExchangeXY) //检查是否要互换XY
        MatrixMuiltiple(&rotate,&exchangeXY,&rotate);	
		//将 exchangeXY 矩阵和 rotate 矩阵相乘，并将结果存储回 rotate 矩阵

    Matrix sendXY;
    Matrix selfXY;

    float sendXY_Array[2]= {0,0};
    float selfXY_Array[2]= {psPara.ePara.posx,psPara.ePara.posy};

    MatrixInit(&sendXY,2,1,sendXY_Array);
    MatrixInit(&selfXY,2,1,selfXY_Array);

    arm_mat_mult_f32(&rotate,&selfXY,&sendXY);//两矩阵相乘，结果存到第三个矩阵当中

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

    //若收到校正信息，上周期坐标与此周期会有较大差值，造成速度异常，故须对速度进行清零处理
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
*@brief 将定位系统坐标系选择为世界坐标系
*@para	pps2Center   定位系统坐标系原点与车中心的距离，单位：mm
*/
void adjustR()
{
		psPara.kalx = fabs(psPara.last_rotatedVelX_robort * 0.005) + 0.5 * fabs(psPara.imuPara.last_Acc_World[0] * 0.005 * 0.005);
		psPara.kaly = fabs(psPara.last_rotatedVelY_robort * 0.005) + 0.5 * fabs(psPara.imuPara.last_Acc_World[1] * 0.005 * 0.005);

		psPara.Modelength = sqrtf(psPara.delPos[0] * psPara.delPos[0] + psPara.delPos[1] * psPara.delPos[1]);//测量的瞬时模长
	  psPara.Kalmodelength = sqrtf(psPara.kalx * psPara.kalx + psPara.kaly * psPara.kaly);//预测的瞬时模长

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
    //将定位系统坐标系选择为世界坐标系
    psPara.rotatedPosX_robort = (psPara.rotatedPosX*cosf(POSEANGLE/180.f*PI)) - psPara.rotatedPosY*sinf(POSEANGLE/180.f*PI);
    psPara.rotatedPosY_robort = (psPara.rotatedPosX*sinf(POSEANGLE/180.f*PI)) + psPara.rotatedPosY*cosf(POSEANGLE/180.f*PI);

//    //将定位系统坐标转化为车中心坐标
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


