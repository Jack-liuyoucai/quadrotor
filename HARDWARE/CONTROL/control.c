#include "control.h"
#include "sys.h"
#include "define.h"
#include "timer.h"
#include "led.h"
struct DATA_XYZ_F GYR_F={0.0,0.0,0.0};
struct DATA_XYZ_F GYR_F_RATE={0.0,0.0,0.0};
struct DATA_XYZ_F GYR_F_AVG={0.0,0.0,0.0};
struct DATA_XYZ_F Current_ERROR={0.0,0.0,0.0};

struct DATA_XYZ ACC={0,0,0};
struct DATA_XYZ GYR_CTR={0,0,0};
//struct DATA_XYZ DATA[100]={0,0,0};
struct DATA_XYZ GYR={0,0,0};
struct DATA_XYZ GYR_RATE={0,0,0};
struct DATA_XYZ HMC={0,0,0};
struct DATA_XYZ ACC_OFFSET={0,0,0};
struct DATA_XYZ GYR_OFFSET={0,0,0};
struct DATA_XYZ ACC_AVG={0,0,0}; 
extern struct DATA_XYZ_F gyr_offset_f;
extern u8 wireless;
struct PID PID_DATA;
struct PID YAW_PID;
struct PID EXP_PID;
struct DATA_16 COMD={0,0,0,0};
float Pitch=0.0,Rool=0.0,Yaw=0.0;
float rool=0,pitch=0;
float EXP_YAW=0;
u8 ct=0;
u8 FLY_EN=0;
u8 ThreeD_EN=0;
u16  ThreeD_TMS=0;
u8 Motion=0,Last_Motion=0; 
float Dynamic_PID_rate=0.0;
float D2=0.0;
struct DATA_XYZ_F EXP_THTA={0.0,0.0,0.0};

float Exp_rate=0.012;   
#define MAX_ANGLE 20
float t1=0.0,t2=0.0,t3=0.0;
float NOW_GO_BACK_X=0.0,NOW_GO_BACK_Y=0.0;
float LAST_GO_BACK_X=0.0,LAST_GO_BACK_Y=0.0;
float GO_BACK_X_VAL=0.0,GO_BACK_Y_VAL=0.0;
u16 GO_BACK_X_CT=0,GO_BACK_Y_CT=0;
u8  GO_BACK_X_STR=0,GO_BACK_Y_STR=0;

void PID_INIT(void)	   
{
	PID_DATA.P = 2.85;	//2.682	   
	PID_DATA.I = 1;   //0.2
	PID_DATA.D = 0.7;	//0.521	 
	PID_DATA.SUM_ERROR_R=0.0;
	PID_DATA.SUM_ERROR_P=0.0;
	PID_DATA.IMAX = 300;

	YAW_PID.P = 7.5;	//7.5   
	YAW_PID.I = 0;   //0.2
	YAW_PID.D = 1.8;	//1.8	 
	YAW_PID.SUM_ERROR_R=0.0;
	YAW_PID.SUM_ERROR_P=0.0;
	YAW_PID.IMAX = 100;

	EXP_PID.P = 1;	    
	EXP_PID.I = 0;    
	EXP_PID.D = 1.8; 	 
	EXP_PID.SUM_ERROR_R=0.0;
	EXP_PID.SUM_ERROR_P=0.0;
	EXP_PID.IMAX = 100;
}

float EXP_BUF_x[50]={0};
float EXP_BUF_y[50]={0};
float avg_exp_x=0,avg_exp_y=0;
void EXP_SMOOTH(u8 tms)
{
	static u8 cnt=0; 
	u8 i=0;
	float temp1=0,temp2=0;
	EXP_BUF_x[cnt] = EXP_THTA.X;//更新滑动窗口数组
	EXP_BUF_y[cnt] = EXP_THTA.Y;//更新滑动窗口数组
	for(i=0;i<tms;i++)
	{
		temp1 += EXP_BUF_x[i]; 
		temp2 += EXP_BUF_y[i]; 
	}
	avg_exp_x = temp1 /(float)tms; 
	avg_exp_y = temp2 /(float)tms;
	cnt++;
	if(cnt==tms)	cnt=0;	 		  
}


float last_exp_x=0.0,last_exp_y=0.0;
float now_exp_x=0.0,now_exp_y=0.0;
float turn=0;
int tt1=0,tt2=0;
int maxx=30;
int goback=0;
float yaw_error=0,last_yaw_error=0.0;
float yaw_p=5,yaw_d=3.065;
float exp_yaw=0.0;
float yaw_tmp=0.0;
float turn_rate=0.00125;

void READ_CONTROL_COMMAND(u8* tmp_buf)			//读取摇杆无线信号方向
{
 
		COMD.THR=tmp_buf[4]*255+tmp_buf[5];
		COMD.THR*=0.25;
   	if(COMD.THR>850) COMD.THR=850;
 
	if(COMD.THR>100)
	{
		FLY_EN=1;
	}
	else
	{
		exp_yaw=Yaw;
		FLY_EN=0;
		PID_DATA.SUM_ERROR_R=0;
		PID_DATA.SUM_ERROR_P=0;
	}
	COMD.LR=	-(tmp_buf[2]*255+tmp_buf[3]-1917); // 
	COMD.FB=	  tmp_buf[0]*255+tmp_buf[1]-2146; //
	COMD.TURN= (tmp_buf[6]*255+tmp_buf[7])-2069; //	
	if(COMD.TURN<200&&COMD.TURN>-200)  COMD.TURN=0;

  turn=COMD.TURN*turn_rate;
	exp_yaw +=turn;	 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if(exp_yaw>360) exp_yaw=0;
	if(exp_yaw<0) exp_yaw=360;

	if(FLY_EN==0&&ThreeD_EN==0&&COMD.FB<-2000)
	{
		ThreeD_EN=1;
		PID_DATA.SUM_ERROR_P=0;
		PID_DATA.SUM_ERROR_R=0;
	}
	

 	if(COMD.FB<5&&COMD.FB>-5)	
		COMD.FB=0 ;
	else
		Motion=1;
	if(COMD.LR<5&&COMD.LR>-5) 
		COMD.LR=0;
	else
		Motion=1;
	if(COMD.FB==0&&COMD.LR==0)
		Motion=0;	 
	
	now_exp_x=COMD.FB*Exp_rate;
	now_exp_y=-COMD.LR*Exp_rate;
    
 	EXP_THTA.X= now_exp_x;
	EXP_THTA.Y= now_exp_y; 
	EXP_SMOOTH(6);
	if(avg_exp_x> MAX_ANGLE)	avg_exp_x= MAX_ANGLE;
	if(avg_exp_x<-MAX_ANGLE)	avg_exp_x=-MAX_ANGLE;	
	if(avg_exp_y> MAX_ANGLE)	avg_exp_y= MAX_ANGLE;
	if(avg_exp_y<-MAX_ANGLE)	avg_exp_y=-MAX_ANGLE;		   // 期望角度 限幅	
/*******************************-**************************************/
	Current_ERROR.X =   avg_exp_x- Rool;	
	Current_ERROR.Y =   avg_exp_y- Pitch;

	last_exp_x=now_exp_x;
	last_exp_y=now_exp_y;
}


float last_gyr_x=0.0,last_gyr_y=0.0;
float sum_gyr_x=0.0,sum_gyr_y=0.0;
int Motor1=0,Motor2=0,Motor3=0,Motor4=0; 	
int yaw=0; 	
float yaw_dt=0;

float RI_MAX=0,PI_MAX=0;
 
void STABLE_WITH_PID(void)
{
	Last_Motion=Motion;
////////////////////////////////////////////////////////////////////////////////PID计算
   	RI_MAX=PID_DATA.IMAX  ;
 	rool = PID_DATA.P * Current_ERROR.X;

	if(Current_ERROR.X<60&&Current_ERROR.X>-60)
		PID_DATA.SUM_ERROR_R+= PID_DATA.I*Current_ERROR.X/400.0 ;
	else
		PID_DATA.SUM_ERROR_R=0;

	if(PID_DATA.SUM_ERROR_R> RI_MAX)	PID_DATA.SUM_ERROR_R= RI_MAX;
	if(PID_DATA.SUM_ERROR_R<-RI_MAX)	PID_DATA.SUM_ERROR_R=-RI_MAX;
 
    rool+=	PID_DATA.SUM_ERROR_R;	  // 加上积分
 
   rool  +=	 - (PID_DATA.D * (GYR_F_RATE.X-gyr_offset_f.X))  ;

	PI_MAX=PID_DATA.IMAX;
	pitch = PID_DATA.P * Current_ERROR.Y ;

	if(Current_ERROR.Y<60&&Current_ERROR.Y>-60)
 		PID_DATA.SUM_ERROR_P+= PID_DATA.I*Current_ERROR.Y/400.0 ;
 	else
 		PID_DATA.SUM_ERROR_P=0;
	if(PID_DATA.SUM_ERROR_P> PI_MAX)	PID_DATA.SUM_ERROR_P= PI_MAX;
	if(PID_DATA.SUM_ERROR_P<-PI_MAX)	PID_DATA.SUM_ERROR_P=-PI_MAX;
   
 	pitch +=  PID_DATA.SUM_ERROR_P;	  // 加上积分  
 
	pitch +=	 - (PID_DATA.D * (GYR_F_RATE.Y-gyr_offset_f.Y)) ; 
/////////////
	yaw_tmp= exp_yaw-Yaw;
	if(yaw_tmp>-180)
	{
		if(yaw_tmp>180)
			yaw_error=( exp_yaw-Yaw-360);
		else
			yaw_error= exp_yaw-Yaw;
	}
	else
		yaw_error= (360+exp_yaw-Yaw);
	yaw =-yaw_p*yaw_error- yaw_d*GYR_F.Z;

	Motor1=(int)(COMD.THR + rool - pitch + yaw); //1///////////rool滚////pitch附仰///////yaw旋//////////////////////////////////////////////////////
	Motor2=(int)  (COMD.THR + rool + pitch - yaw);	//1
	Motor3=(int)(COMD.THR - rool + pitch + yaw); //1
	Motor4=(int)(COMD.THR - rool - pitch - yaw);
	Limit_amplitude(&Motor1,&Motor2,&Motor3,&Motor4);
  
	if(FLY_EN==1&&COMD.THR>200)  
	{
	    Refresh_Or_STOP_MOTOR(1);
		ThreeD_EN=0;			 
		ThreeD_TMS=0;
	}
	 else
	   Refresh_Or_STOP_MOTOR(2);

	if((Pitch>140||Pitch<(-140))||(Rool>140||Rool<(-140)))
	{ Refresh_Or_STOP_MOTOR(2);	 LED_RED_ON;	LED_GRE_ON;	   }
	 if(wireless==0)  	Refresh_Or_STOP_MOTOR(2);
}

void CRASH_LANDING()
{
	COMD.THR=  330;
	FLY_EN=1;
	COMD.LR=	0;
	COMD.FB=	0;
	COMD.TURN=  0;  
	Current_ERROR.X=0;
	Current_ERROR.Y=0;
	PID_DATA.SUM_ERROR_R=0;
	PID_DATA.SUM_ERROR_P=0; 	
}

void Limit_amplitude(int*m1,int* m2,int* m3,int* m4) 
{
	if(*m1>995) *m1=995;
	else if(*m1<0) *m1=0;
	if(*m2>995) *m2=995;
	else if(*m2<0) *m2=0;
	if(*m3>995) *m3=995;
	else if(*m3<0) *m3=0;
	if(*m4>995) *m4=995;
	else if(*m4<0) *m4=0;
}
void Refresh_Or_STOP_MOTOR(u8 sel)
{
	if(sel==1)
	{  
	
     	PWM_CH1=Motor4;	  //2
		PWM_CH2=Motor1;	  //3
		PWM_CH3=Motor2;	  //4
		PWM_CH4=Motor3;	  //1
	}
	else if(sel==2)	
	{
	   rool=0; pitch=0; yaw=0;
	   Rool=0;Pitch=0;Yaw=0;
		PWM_CH1=0;
		PWM_CH2=0;
		PWM_CH3=0;
		PWM_CH4=0;		
	}
	else if(sel==3)	
	{
	   rool=0; pitch=0; yaw=0;
	   Rool=0;Pitch=0;Yaw=0;
		PWM_CH1=330;
		PWM_CH2=330;
		PWM_CH3=330;
		PWM_CH4=330;		
	}
}


