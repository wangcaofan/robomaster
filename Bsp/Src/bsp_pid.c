#include "bsp_pid.h"
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_init(PID_TypeDef *pid,const float PID[3],float max_out, float max_iout){
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->out_MAX = max_out;
    pid->iout_MAX = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->Err[0] = pid->Err[1] = pid->Err[2] = pid->P_out = pid->I_out = pid->D_out = pid->T_out = 0.0f;

}
//void FCC_init(FCC_TypeDef *fcc,const float FCC[3]){
// fcc->A=FCC[0];
//	fcc->B=FCC[1];
//	fcc->T=FCC[2];
//  fcc->rin=0; 
//	fcc->last_rin=0;
//	fcc->prrin=0;
//}
float PID_calc(PID_TypeDef *pid,float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->Err[2] = pid->Err[1];
    pid->Err[1] = pid->Err[0];
    pid->set = set;
    pid->fdb = ref;
    pid->Err[0] = set - ref;
//		fcc->prrin=fcc->last_rin;
//		fcc->last_rin=fcc->rin;
//    fcc->rin=ref;
//    fcc->result=(fcc->A*(fcc->rin - fcc->last_rin))/fcc->B+(fcc->rin-2*fcc->last_rin+fcc->prrin)/(fcc->B*fcc->T*fcc->T);
        pid->P_out = pid->Kp * pid->Err[0];
        pid->I_out += pid->Ki * pid->Err[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->Err[0] - pid->Err[1]);
        pid->D_out = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->I_out, pid->iout_MAX);
        pid->T_out = pid->P_out + pid->I_out + pid->D_out;
        LimitMax(pid->T_out, pid->out_MAX);
		   return pid->T_out;
    }
void PID_clear(PID_TypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Err[0] = pid->Err[1] = pid->Err[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->T_out = pid->P_out = pid->I_out = pid->D_out = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
