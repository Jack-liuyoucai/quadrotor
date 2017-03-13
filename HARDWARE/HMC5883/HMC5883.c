#include "HMC5883.h"
#include "STM32_I2C.h"
#include "define.h"

short hxx=0,hyy=0,hzz=0;

void Init_HMC5883()
{
	init_HMC5883();
}

void HMC5883_Muti_Read(short *xx,short *yy,short *zz)
{
	HMC5883_read(xx,yy,zz);
}
