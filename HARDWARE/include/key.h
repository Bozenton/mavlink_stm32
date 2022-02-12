#ifndef KEY_H_
#define KEY_H_
/**
 * @brief 按键
 * 
 *      KEY0:   PE4
 *      WK_UP:  PA0
 */

#include "stm32f10x.h"
#include "key.h"
#include "sys.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

#define READ_KEY0    GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)
#define READ_WKUP   GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)

#define PRES_KEY0   1
#define PRES_WKUP   2

void keyInit(void);


#endif
