#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "FreeRTOS.h"
#include "semphr.h"

#include "mavInterface.h"
#include "mavMsg.h"

#include "delay.h"
#include "console.h"
#include "led.h"

#define CTRL_NULL       0

#define CTRL_ENABLE     1

#define CTRL_UP             2
#define CTRL_DOWN           3
#define CTRL_FORWARD        4
#define CTRL_BACKWARD       5
#define CTRL_LEFT           6
#define CTRL_RIGHT          7
#define CTRL_LEFT_ROTATE    8
#define CTRL_RIGHT_ROTATE   9

#define VEL             0.3
#define TAKEOFF_HEIGHT  0.5

/**
 * @brief 启动控制
 * 
 * @param pvParameters 
 */
void task_ctrlStart(void* pvParameters);

/**
 * @brief 向autopilot写位置信息
 * 
 * @param pvParameters 
 */
void task_ctrlWritePos(void * pvParameters);

/**
 * @brief 接收来自蓝牙的控制信号，然后更新目标位置
 * 
 * @param pvParameters 
 */
void task_btCtrl(void * pvParameters);

#endif

