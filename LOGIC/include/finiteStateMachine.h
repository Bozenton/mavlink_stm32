#ifndef FINITE_STATE_MACHINE_H_
#define FINITE_STATE_MACHINE_H_

#include "fsmCondition.h"
#include "console.h"

/**
 * @brief 初始化状态机
 * 
 */
void fsmInit(void);

/**
 * @brief 查询当前状态
 * 
 * @return FLY_STATE 
 */
char queryCurrentState(void);

/**
 * @brief 状态更新
 * 
 * @param pvParameters 
 */
void task_fsmHandleEvent(void * pvParameters);

/**
 * @brief 各个状态中执行的动作
 * 			此任务的优先级应较低
 * 
 * @param pvParameters 
 */
void task_fsmAction(void * pvParameters);

enum eventType{
	Event_Fly, 
};

#endif


