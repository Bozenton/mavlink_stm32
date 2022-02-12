#ifndef FSM_CONDITION_H_
#define FSM_CONDITION_H_

#include <stdint.h>

#define CONDITION_TAKEOFF		((uint8_t)'t')
#define CONDITION_REACH			((uint8_t)'r')
#define CONDITION_FINISH        ((uint8_t)'f')
// #define CONDITION_DETECT		((uint8_t)'d')
// #define CONDITION_SCAN			((uint8_t)'s')
#define CONDITION_LAND			((uint8_t)'l')
#define CONDITION_ADJUST		((uint8_t)'a')

// typedef enum _FLY_STATE{
//     STATE_GROUND = 'g', 
//     STATE_TAKINGOFF = 't', 
//     STATE_CRUISE = 'c', 
//     STATE_NEAR = 'n', 
//     STATE_SCAN = 's', 
//     STATE_AWAY = 'a', 
//     STATE_LANDING = 'l'
// }FLY_STATE;

typedef enum _FLY_STATE{
    STATE_GROUND = 'g', 
    STATE_TAKINGOFF = 't', 
    STATE_XTOA = 'x', // 从十字飞向A点
	STATE_CRUISE = 'c', // 循迹各个格子 
    STATE_BACK = 'b', 
    STATE_LANDING = 'l'
}FLY_STATE;


#endif



