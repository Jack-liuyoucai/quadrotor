#ifndef __CONTROL_H
#define __CONTROL_H	
#include"sys.h"

void PID_INIT(void)	;
void COMP_TX_BUF(float pitch_or_rool,float gyrs,float comd,float exp_rate,u8*tx);
void READ_CONTROL_COMMAND(u8* tmp_buf);
void STABLE_WITH_PID(void);
void CRASH_LANDING(void);
void Limit_amplitude(int*m1,int* m2,int* m3,int* m4);
void Refresh_Or_STOP_MOTOR(u8 sel);
void EXP_SMOOTH(u8 tms);

#endif


