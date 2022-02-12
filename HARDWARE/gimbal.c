#include "gimbal.h"

static void updateAngular(struct _Gimbal_t *gimbal, uint8_t s);
static void setGoalAngle(struct _Gimbal_t *gimbal, uint8_t s, uint8_t goal);
static void move(struct _Gimbal_t *gimbal, uint8_t s, short v);

void Gimbal_StructInit(Gimbal_t *gimbal)
{
    gimbal->biasServo[0] = 18; // 108-90
    gimbal->minServo[0] = 0 + gimbal->biasServo[0];
    gimbal->maxServo[0] = 180 - gimbal->biasServo[0];
    gimbal->nowServo[0] = 90;
    gimbal->goalServo[0] = 90;
    
    gimbal->biasServo[1] = 0;
    gimbal->minServo[1] = 90;
    gimbal->maxServo[1] = 180;
    gimbal->nowServo[1] = 90;
    gimbal->goalServo[1] = 90;

    gimbal->f_UpdateAngular = updateAngular;
    gimbal->f_setGoalAngle = setGoalAngle;
    gimbal->f_move = move;

}



static void updateAngular(struct _Gimbal_t *gimbal, uint8_t s)
{
    if( (s!=0) && (s!=1) )
    {
        #if SWITCH_DEBUG
        // printf("Parameter servo should be 0 or 1\r\n");
        #endif
        return ;
    }

    short now = gimbal->nowServo[s];
    if( (short)gimbal->goalServo[s]-(short)gimbal->nowServo[s] > SERVO_ANGULAR_VEL )
    {
        if(now + SERVO_ANGULAR_VEL > gimbal->maxServo[s])
            gimbal->nowServo[s] = gimbal->maxServo[s];
        else
            gimbal->nowServo[s] += SERVO_ANGULAR_VEL;
    }
    else if((short)gimbal->goalServo[s]-(short)gimbal->nowServo[s] < -1*SERVO_ANGULAR_VEL)
    {
        if(now - SERVO_ANGULAR_VEL < gimbal->minServo[s])
            gimbal->nowServo[s] = gimbal->minServo[s];
        else
            gimbal->nowServo[s] -= SERVO_ANGULAR_VEL;
    }
    else 
        gimbal->nowServo[s] = gimbal->goalServo[s];

    if(s==0)
        servo1Angle(gimbal->nowServo[s] + gimbal->biasServo[0]);
    else
        servo2Angle(gimbal->nowServo[s] + gimbal->biasServo[1]);
}

static void setGoalAngle(struct _Gimbal_t *gimbal, uint8_t s, uint8_t goal)
{
    if(goal > gimbal->maxServo[s])
        gimbal->goalServo[s] = gimbal->maxServo[s];
    else if(goal < gimbal->minServo[s])
        gimbal->goalServo[s] = gimbal->minServo[s];
    else
        gimbal->goalServo[s] = goal;
}

static void move(struct _Gimbal_t *gimbal, uint8_t s, short v)
{
    short nowgoal = gimbal->goalServo[s];
    if( (short)nowgoal+v*SERVO_ANGULAR_VEL > gimbal->maxServo[s] )
        gimbal->goalServo[s] = gimbal->maxServo[s];
    else if( (short)nowgoal+v*SERVO_ANGULAR_VEL < gimbal->minServo[s] )
        gimbal->goalServo[s] = gimbal->minServo[s];
    else 
        gimbal->goalServo[s] += v*SERVO_ANGULAR_VEL; 
}

#if SWITCH_GIMBAL
extern Gimbal_t Gimbal;
#endif 

void task_updateServos(void * pvParameters)
{
    while(1)
    {
        #if SWITCH_GIMBAL
        Gimbal.f_UpdateAngular(&Gimbal, 0);
        Gimbal.f_UpdateAngular(&Gimbal, 1);
        #endif
        delay_ms(50);
    }
}


