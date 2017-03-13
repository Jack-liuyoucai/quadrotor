#ifndef __HMC5883_H
#define __HMC5883_H	
#include "sys.h"

void Init_HMC5883(void);
void HMC5883_Muti_Read(short *xx,short *yy,short *zz);

#endif		
