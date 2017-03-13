#ifndef __IMU_H
#define __IMU_H	
#include "sys.h"

void MahonyIMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void ACC_SMOOTH(u8 smooth_tms);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void ACC_Filter_with_FIR(void);
float ADXL345_Get_Angle(float x,float y,float z,u8 dir);
#endif		
