#ifndef SERVO_H_
#define SERVO_H_

// TIM2_CH2: PB3(重映射)
// TIM5_CH2: PA1

#include "stm32f10x.h"
#include "stdio.h"
#include "console.h"

#define SERVO1_BIAS     0
#define SERVO2_BIAS     0

/**
 * @brief 初始化舵机
 * 
 */
void initServo(void);

/**
 * @brief 设置舵机1的角度
 * 
 * @param theta 
 */
void servo1Angle(uint8_t theta);

/**
 * @brief 设置舵机2角度
 * 
 * @param theta 
 */
void servo2Angle(uint8_t theta);

#endif

