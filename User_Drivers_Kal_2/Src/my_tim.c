/**
  ******************************************************************************
  * @file    my_tim.c
  * @author  Falonss
  * @version V1.0.0
  * @date    2023.03.04
  * @brief   与定时器相关的代码
  ******************************************************************************
**/
#include "my_tim.h"

//运行次数
static uint32_t runTimeCnt = 0;

//5ms计时
static uint8_t timeFlag = 0;

//1ms计时
static uint8_t oneReadTimeFlag = 0;

/********************************************************
函数功能：定时器初始化                
入口参数：
返回  值：
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
函数功能：定时器回调函数                
入口参数：
返回  值：
*********************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//定时器2：每隔1ms对采集的原始数据进行处理
	if(htim->Instance == htim2.Instance)
	{
		oneReadTimeFlag = 1;
	}
	
	//定时器3：每隔1ms采集一次原始数据
	if(htim->Instance == htim3.Instance)
	{
		
	}
	
	//定时器4：程序运行计时
	if(htim->Instance == htim4.Instance)
	{
		//程序运行计时
		psPara.timeCounter++;
	}
}
/********************************************************
函数功能：每1ms计时 
入口参数：
返回  值：
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
函数功能：每5ms计时                
入口参数：
返回  值：
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

