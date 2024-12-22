#ifndef _FIGUREANGLE_H
#define _FIGUREANGLE_H

#include "config.h"
#include "my_math.h"
#include "math.h"
#include "arm_math.h"
#include "quarternion.h"
#include "my_usart.h"

uint8_t RoughHandle(void);

uint8_t UpdateIMUZeroDrift(void);

uint8_t imuUpdateGyroOneAxisZeroDrift(uint8_t axis);

uint8_t UpdateAccelerometerOneZeroDrift(uint8_t axis);

void UpdateAngleByPixhawk(void);

void GyroCorrection(double gyroAngleCorrect[3] , double qCorrect[4]);

void UpdateAccByAngle(void);

uint8_t UpdateAccByAngleOneZeroDrift(uint8_t axis);

uint8_t UpdateAccByAngleZeroDrift(void);

#endif
