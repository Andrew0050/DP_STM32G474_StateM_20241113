#ifndef __CTLLOOP_H
#define __CTLLOOP_H	 

#include "stm32g4xx_it.h"
#include "function.h"

void BUCKVLoopCtlPID(void);

extern int32_t  VErr0,VErr1,VErr2;
extern int32_t	u0,u1;


//һ���������������� 
#define PERIOD 10240	 
#endif

