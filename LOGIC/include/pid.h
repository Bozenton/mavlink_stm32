#ifndef PID_H
#define PID_H

typedef struct _PID_t
{
    float KP;
    float KI;
    float KD;

    float TargetValue;  // 目标值
    float CurrentValue; // 当前值
    float Error;        // 误差
    float LastError;    // 上次误差
    float IValue;       // 积分值
    float integral_limit;           // 抗积分饱和限制

    float max_output;   // 输出上限
    float output;       // 输出值
    unsigned char FirstTimeFlag;    // 第一次计算时不要D项

    void (*f_ParamInit) // 结构体内的函数指针变量，赋函数指针后可进行调用
    (
       struct _PID_t *pid,  // 该类型结构体指针
       float p,
       float i,
       float d,
       float integral_limit,
       float max_output
    );

    void (*f_Calculate)
    (
        struct _PID_t *pid,
        float current_value
    );
}PID_t;

void PID_StructInit(PID_t *pid);

#endif


