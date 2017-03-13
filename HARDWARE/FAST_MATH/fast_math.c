#include "fast_math.h"

#define ONE_PI   (3.14159265)
#define TWO_PI   (6.2831853)
#define ANGLE_UNIT (0.62831853)

float my_abs(float f)
{
	if (f >= 0.0)
	{
		return f;
	}

	return -f;
}

/*
float my_atan(float x, float y)
{
	return my_atan2(y, x);
} */

//计算浮点数平方
float my_pow(float a)
{
	return a*a;
}

//快速平方根算法
float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}

double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad*(1.27323954 + 0.405284735 * rad);
	else
		sine = rad * (1.27323954 - 0.405284735 * rad);
	if (sine < 0)
		sine = sine*(-0.225 * (sine + 1) + 1);
	else
		sine = sine * (0.225 *( sine - 1) + 1);
	return sine;
}

double my_sin(double rad)
{
	char flag = 1;

	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -flag;
	}

	return mx_sin(rad) * flag;
}

float my_cos(double rad)
{
	char flag = 1;
	rad += ONE_PI/2.0;

	if (rad >= ONE_PI)
	{
		flag = -flag;
		rad -= ONE_PI;
	}

	return my_sin(rad)*flag;
}


// xatan 函数
float xatan(float x)
{
    float x2;
    x2=x*x;
    return x*(6.36918871*x2+1.26599861e1)/(x2*(x2+1.05891113e1)+1.26599865e1);
}
// yatan 函数
float yatan(float x)
{
    if (x<1.41421356-1.0) return xatan(x);
    if (x>1.41421356+1.0) return 0.5*3.1415926535898 -xatan(1.0/x);
    return 0.25*3.1415926535898 +xatan((x-1.0)/(x+1.0));
}


float my_asin(float x) 
{ 
    unsigned char neg=0; // 符号变量
    float t;             // 返回的角度值(范围是-PI/2 to PI/2)

    if ((x<-1.0) || (x>1.0)) return 0; // 如果x超过asin的定义域，返回HUGE_VAL
    if (x<0.0) {x=-x; neg=1;}; // 如果 x < 0，取其绝对值，判断角度为负(看单位圆) 
    t=my_sqrt(1.0-x*x); // 勾股定理，t为“点”在 x 轴上的投影长度(坐标)
    if (x>7.07106781e-1) t=0.5*3.1415926535898 -yatan(t/x); // 如果x>0.707，表明角度绝对值大于45度，90度减去用yatan(反正切)求出的角度即得到所要的值
    else t=yatan(x/t);  // 如果角度<45度，x/t，反正切直接求出角度
    if (neg) return -t; // 如果角度为负，返回 -t
    return t;           // 返回 t 
}


// Fast inverse square-root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
