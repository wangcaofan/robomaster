#ifndef PID_H
#define PID_H
#include "main.h"
typedef __packed struct{
   float Kp,Ki,Kd;
	 float out_MAX,iout_MAX;
	 float set,fdb;
	 float T_out,P_out,I_out,D_out;
	 float Dbuf[3],Err[3];
}PID_TypeDef;
typedef __packed struct{
  float A,B,T;
	float rin;
	float last_rin;
	float prrin;
	float result;
}FCC_TypeDef;

extern void PID_init(PID_TypeDef *pid,const float PID[3],float max_out, float max_iout);
extern void FCC_init(FCC_TypeDef *fcc,const float FCC[3]);
extern float PID_calc(PID_TypeDef *pid,float ref, float set);
extern void PID_clear(PID_TypeDef *pid);
















#endif
