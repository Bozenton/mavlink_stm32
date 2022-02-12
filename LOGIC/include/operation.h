#ifndef OPERATION_H_
#define OPERATION_H_

#include "mavInterface.h"
#include "mavMsg.h"
#include "console.h"

/**
 * @brief 起飞操作
 * 
 * @param api 接口指针
 * @param takeoffHeight 起飞高度（正数）
 */
void opTakeoff(Autopilot_Interface * api);

/**
 * @brief 降落操作（以匀速下降）
 * 
 * @param api 
 */
void opLand(Autopilot_Interface * api);


#endif


