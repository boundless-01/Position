#ifndef _MY_MAIN_H
#define _MY_MAIN_H

#include "main.h"
#include "my_tim.h"
#include "my_usart.h"
#include "my_imu.h"
#include "my_gpio.h"
#include "figureAngle.h"
#include "mt6835.h"
#include "figurePos.h"

void main_loop(void);

void PsPallInit(void);

#endif
