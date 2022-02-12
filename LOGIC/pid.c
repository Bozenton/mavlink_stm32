#include "PID.h"

static void ParamInit(PID_t *pid,float p,float i,float d,float integral_limit,float max_output);
static void Calculate(PID_t *pid,float current_value);
 
void PID_StructInit(PID_t *pid)
{
    pid->f_ParamInit = ParamInit;
    pid->f_Calculate = Calculate;
    pid->LastError = 0;
    pid->TargetValue = 0;
    pid->CurrentValue = 0;
    pid->IValue = 0;
    pid->FirstTimeFlag = 1;
}

static void ParamInit(PID_t *pid,float p,float i,float d,float integral_limit,float max_output)
{
    pid->KP = p; 
    pid->KI = i; 
    pid->KD = d; 
    pid->integral_limit = integral_limit;
    pid->max_output = max_output; 
}

static void Calculate(
        PID_t *pid,
        float current_value)
{
    pid->CurrentValue = current_value;
    pid->Error = pid->TargetValue - pid->CurrentValue;
    float p_value = pid->KP * pid->Error; 
    float d_value = 0;
    if(pid->FirstTimeFlag == 1)pid->FirstTimeFlag = 0;
    else d_value = pid->KD * (pid->Error - pid->LastError);
    pid->IValue += pid->KI * pid->Error;
    if (pid->IValue > pid->integral_limit)
    {
        pid->IValue = pid->integral_limit;
    }
    if (pid->IValue < -pid->integral_limit)
    {
        pid->IValue = -pid->integral_limit;
    }
    pid->output = p_value + d_value + pid->IValue;
    pid->LastError = pid->Error;
    if (pid->output > pid->max_output){pid->output = pid->max_output;}
    if (pid->output < -pid->max_output){pid->output = -pid->max_output;}
}



