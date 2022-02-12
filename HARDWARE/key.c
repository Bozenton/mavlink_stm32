#include "key.h"

void keyInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // NVIC_InitTypeDef NVIC_InitStructure;
    // EXTI_InitTypeDef EXTI_InitStructure;

    // KEY0:   PE4
    // WK_UP:  PA0
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 使能复用功能时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // 下拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // // GPIOE4 中断
    // GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource4);
    // EXTI_InitStructure.EXTI_Line = EXTI_Line4; 
    // EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    // EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    // EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    // EXTI_Init(&EXTI_InitStructure);

    // // GPIOA0
    // GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    // EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    // EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    // EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    // EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    // EXTI_Init(&EXTI_InitStructure);

    // // GPIOE4
    // NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);

    // // GPIOA0
    // NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);
}


/**
 * @brief   按键扫描函数
 *          由于机械按键需要去抖动，所以不宜用中断
 *          把它作为一个freertos任务或许更合适
 * 
 * @return  PRES_KEY0   1
 *          PRES_WKUP   2
 */

uint8_t keyScan()
{
    static uint8_t keyUp = 1;
    if(keyUp && (READ_KEY0==1||READ_WKUP==1))
    {
        delay_ms(10); // 防抖
        keyUp = 0;
        if(READ_KEY0 == 1)
            return PRES_KEY0;
        else if(READ_WKUP == 1)
            return PRES_WKUP;
    }
    else if(READ_KEY0 == 0 && READ_WKUP == 0)
        keyUp = 1;
    
    return 0;
}

void task_key(void* pvParameters)
{
    uint8_t flag;
    while(1)
    {
        flag = keyScan();
        if(flag == PRES_KEY0)
        {

        }
        else if(flag == PRES_WKUP)
        {
            
        }
    }
}






// /**
//  * @brief WK_UP 按键中断服务函数
//  * 
//  */

// void EXTI0_IRQHandler(void)
// {
// //    delay_ms(10); // 消除抖动
//     if( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1 )
//     {
//         // TaskStatus_t taskStatus;
// 		// TaskHandle_t taskHandle;
		
        
// 		// taskHandle = xTaskGetHandle("task_BtSend");
// 		// vTaskGetInfo(taskHandle, &taskStatus, pdTRUE, eInvalid);
// 		// printf("stack = %d\r\n", taskStatus.usStackHighWaterMark);
//     }
//     EXTI_ClearITPendingBit(EXTI_Line0);
// }

// /**
//  * @brief KEY0 按键中断服务函数
//  * 
//  */
// void EXTI4_IRQHandler(void)
// {
// //    delay_ms(10); 
//     if( GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 1)
//     {
//         // ...
//     }
//     EXTI_ClearITPendingBit(EXTI_Line4);
// }




