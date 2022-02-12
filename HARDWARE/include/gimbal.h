#ifndef GIMBAL_H_
#define GIMBAL_H_

#include "stm32f10x.h"
#include "servo.h"
#include "console.h"
#include "delay.h"

#define SERVO_ANGULAR_VEL   2
#define SERVO_UPDATE_ITV    (1000*SERVO_ANGULAR_VEL/180)  // ms


typedef struct _Gimbal_t
{
    // 舵机参数
    uint8_t minServo[2];
    uint8_t maxServo[2];
    uint8_t nowServo[2];    // 当前角度
    uint8_t goalServo[2];   // 目标位置
    uint8_t biasServo[2];

    void (*f_UpdateAngular)
    (
        struct _Gimbal_t *gimbal,  
        uint8_t s
    );

    void (*f_setGoalAngle)
    (
        struct _Gimbal_t *gimbal, 
        uint8_t s,
        uint8_t goal
    );
    
    void (*f_move)
    (
        struct _Gimbal_t *gimbal,
        uint8_t s, 
        short v
    );
    
}Gimbal_t;

void Gimbal_StructInit(Gimbal_t *gimbal);

void task_updateServos(void * pvParameters);



#endif

