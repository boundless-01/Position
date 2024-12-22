#ifndef __POS_H
#define __POS_H

#include "stdint.h"
#include "math.h"
#include "config.h"
#include "MT6835.h"

void CalculatePos(void);

void SetPosX(double in);

void SetPosY(double in);

void FigureVell(void);

void CorrectHandler(void);

#endif

