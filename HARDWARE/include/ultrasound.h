#ifndef ULTRASOUND_H_
#define ULTRASOUND_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "delay.h"
#include "queue.h"

#include "console.h"

#define TIM3_ARR    0xffff
#define TIM4_ARR    0xffff

#define REC_FINISHED    0x8000
#define REC_HIGH        0x4000
#define REC_OVERFLOW    0x3f


/**
 * @brief 超声波模块初始化
 * 
 */
void ultrasoundInit(void);

/**
 * @brief 进行一次超声波测距
 * 
 */
void ultrasoundTrigger(void);


/**
 * @brief 
 * 
 * @param intervalTime 两次测量的时间间隔
 * @return float 测量得到的距离，若为-1说明测量失败
 */
float ultrasoundGetDis(void);


/**
 * @brief 超声波测距任务
 * 
 * @param pvParameters 
 */
void task_ultrasound(void *pvParameters);


#endif
