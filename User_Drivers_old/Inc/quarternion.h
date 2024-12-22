#ifndef QUARTERNION_H
#define QUARTERNION_H

#include "config.h"
#include "my_math.h"
#include "figureAngle.h"
#include "arm_math.h"

void Euler_to_Quaternion(const double Rad[3],double quaternion[4]);
void QuaternionInt(double quaternion[4],double data[3] );
void Quaternion_to_Euler(const double quaternion[4],double Rad[3]);

#endif
