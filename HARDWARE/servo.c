/**
 * @file servo.c
 * @author 小白哥 (you@domain.com)
 * @brief 舵机
 * @version 0.1
 * @date 2021-10-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "servo.h"
// TIM2_CH2: PB3(重映射)
// TIM5_CH2: PA1

#define ARR         2000
#define SCALER      720

void TIM2Servo1Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能时钟
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );

    // 重映射
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); //TIM2_CH2完全重映射到PB3，参考《stm32中文参考手册》8.3.7
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); // 禁用JTAG

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 初始化TIM2
    TIM_TimeBaseStructure.TIM_Period = ARR-1; // 装载到TIMx->ARR
    TIM_TimeBaseStructure.TIM_Prescaler = SCALER-1; // f=72MHz/720=100kHz, T=0.01ms
                                                // 因此PWM的周期为2000*0.01ms = 20ms, f=50Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 初始化TIM2_CH2 PWM模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
    // TIM_OCInitStructure.TIM_Pulse = 1500;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC2

    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR2上的预装载寄存器
    
    servo1Angle(90);

    TIM_Cmd(TIM2, ENABLE);
}

// TIM5_CH2: PA1
void TIM5Servo2Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能时钟
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 初始化TIM2
    TIM_TimeBaseStructure.TIM_Period = ARR-1; // 装载到TIMx->ARR
    TIM_TimeBaseStructure.TIM_Prescaler = SCALER-1; // f=72MHz/720=100kHz, T=0.01ms
                                                // 因此PWM的周期为2000*0.01ms = 20ms, f=50Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    // 初始化TIM2_CH2 PWM模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC2

    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM2在CCR2上的预装载寄存器
    
    // TIM_SetCompare2(TIM2, 1500); // 1500*1us=1.5ms 对应90度附近
    servo2Angle(90);

    TIM_Cmd(TIM5, ENABLE);
}


/**
 * @brief 初始化舵机
 * 
 */

static float period;
static float minv;
static float maxv;
void initServo(void)
{
    TIM2Servo1Init();
    TIM5Servo2Init();
    period = (float)SCALER/72.0/1000;
    minv = 0.5 / period;
    maxv = 2.5 / period;
}

/**
 * @brief 设置舵机1的角度
 * 
 * @param theta 
 */



void servo1Angle(uint8_t theta)
{
    // 当高电平的脉宽在0.5ms-2.5ms之间时舵机就可以对应旋转到不同的角度
    // 0.5ms/0.1ms = 5      0   degree
    // 2.5ms/0.1ms = 25     180 degree
    // if(theta > 180)
    // {
    //     theta = 90;
    //     #if SWITCH_DEBUG
    //         printf("The input of servo should be in 0,180 \r\n");
    //     #endif
    // }
    // uint16_t ccr = (2500.0-500.0)*theta/180.0+500.0+SERVO1_BIAS;

    uint16_t ccr = (maxv-minv)*(float)theta/180.0+minv+SERVO1_BIAS; 
    TIM_SetCompare2(TIM2, ccr);
}

/**
 * @brief 设置舵机2角度
 * 
 * @param theta 
 */
void servo2Angle(uint8_t theta)
{
    uint16_t ccr = (maxv-minv)*(float)theta/180.0+minv+SERVO2_BIAS; 
    TIM_SetCompare2(TIM5, ccr);
}





