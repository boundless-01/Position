#ifndef _MY_TIM_H
#define _MY_TIM_H

#include "tim.h"
#include "my_iwdg.h"
#include "my_gpio.h"

void Tim_User_Start(void);

uint8_t GetOneReadTimeFlag(void);

uint8_t GetTimeFlag(void);

#endif
