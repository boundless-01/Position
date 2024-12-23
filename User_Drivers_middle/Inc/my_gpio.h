#ifndef _MY_GPIO_H
#define _MY_GPIO_H

#include "gpio.h"

/*GPIO*/
#define WORK_ON HAL_GPIO_WritePin(WORK_LED_GPIO_Port, WORK_LED_Pin, GPIO_PIN_SET)
#define WORK_OFF HAL_GPIO_WritePin(WORK_LED_GPIO_Port, WORK_LED_Pin, GPIO_PIN_RESET)
#define ERROR_ON HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET)
#define ERROR_OFF HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET)

#endif
