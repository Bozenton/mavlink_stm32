#ifndef TASKQUERY_H_
#define TASKQUERY_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "delay.h"
#include "console.h"

/**
 * @brief 查询各个任务的堆栈历史剩余最小值
 * 
 * @param pvParameters 
 */
void task_taskQuery(void *pvParameters);

#endif
