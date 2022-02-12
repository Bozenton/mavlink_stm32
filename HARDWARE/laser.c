#include "laser.h"

/**
 * @brief 激光笔对应GPIO初始化
 *      使用PG8
 * 
 */
void initLaser(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);	 //使能PG端口时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PB.5 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOG, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
    GPIO_ResetBits(GPIOG, GPIO_Pin_8);
}

