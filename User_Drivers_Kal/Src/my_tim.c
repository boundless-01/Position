/**
  ******************************************************************************
  * @file    my_tim.c
  * @author  Falonss
  * @version V1.0.0
  * @date    2023.03.04
  * @brief   �붨ʱ����صĴ���
  ******************************************************************************
**/
#include "my_tim.h"

//���д���
static uint32_t runTimeCnt = 0;

//5ms��ʱ
static uint8_t timeFlag = 0;

//1ms��ʱ
static uint8_t oneReadTimeFlag = 0;

/********************************************************
�������ܣ���ʱ����ʼ��                
��ڲ�����
����  ֵ��
*********************************************************/
void Tim_User_Start(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim3);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim4);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
}
/********************************************************
�������ܣ���ʱ���ص�����                
��ڲ�����
����  ֵ��
*********************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//��ʱ��2��ÿ��1ms�Բɼ���ԭʼ���ݽ��д���
	if(htim->Instance == htim2.Instance)
	{
		oneReadTimeFlag = 1;
	}
	
	//��ʱ��3��ÿ��1ms�ɼ�һ��ԭʼ����
	if(htim->Instance == htim3.Instance)
	{
		
	}
	
	//��ʱ��4���������м�ʱ
	if(htim->Instance == htim4.Instance)
	{
		//�������м�ʱ
		psPara.timeCounter++;
	}
}
/********************************************************
�������ܣ�ÿ1ms��ʱ 
��ڲ�����
����  ֵ��
*********************************************************/
uint8_t GetOneReadTimeFlag(void)
{
	if(oneReadTimeFlag)
	{
		oneReadTimeFlag = 0;
		runTimeCnt++;
		if(runTimeCnt >= RUN_PERIOD)
		{
			runTimeCnt = 0;
			timeFlag = 1;
			WORK_OFF;
			HAL_IWDG_Refresh(&hiwdg1);
		}
		return 1;
	}
	return 0;
}
/********************************************************
�������ܣ�ÿ5ms��ʱ                
��ڲ�����
����  ֵ��
*********************************************************/
uint8_t GetTimeFlag(void)
{
	if(timeFlag)
	{
		timeFlag = 0;
		return 1;
	}
	return 0;
}

