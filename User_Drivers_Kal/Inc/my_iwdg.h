#ifndef _MY_IWDG_H
#define _MY_IWDG_H

#include "iwdg.h"

void IWDG_ReloadNew(uint32_t Prescaler , uint32_t reload);

void IWDG_Reset(void);

#endif
