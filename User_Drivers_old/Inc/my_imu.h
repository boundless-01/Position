#ifndef _MY_IMU_H
#define _MY_IMU_H

#include "scha63x.h"
#include "my_usart.h"

extern scha63x_real_data scha63x_real_data_crc;

void IMU_Init(void);

void IMU_DataSampling(void);

void IMU_DataHandling(void);

#endif
