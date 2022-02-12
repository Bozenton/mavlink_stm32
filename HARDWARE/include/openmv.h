#ifndef OPENMV_H_
#define OPENMV_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "console.h"

/* 所使用的串口及其GPIO配置 */
#define OPMV_PIN_TX               GPIO_Pin_10              // TX: 
#define OPMV_PIN_RX               GPIO_Pin_11              // RX: 
#define OPMV_GPIO                 GPIOC
#define OPMV_IRQn                 UART4_IRQn
#define OPMV_USART                UART4
#define OPMV_RCC_USART            RCC_APB1Periph_UART4
#define OPMV_RCC_GPIO             RCC_APB2Periph_GPIOC
#define OPMV_PreemptionPriority   8                       // 抢占优先级
#define OPMV_SubPriority          0                       // 子优先级
#define OPMV_BaudRate             115200                   // 波特率


/* 水平方向OPENMV，加上后缀HOR */
// 与硬件无关的一些参数
// 使用UART5
// TX: PC12
// RX: PD2
#define OPMVHOR_PIN_TX              GPIO_Pin_12              // TX: 
#define OPMVHOR_PIN_RX              GPIO_Pin_2              // RX: 
#define OPMVHOR_GPIO_TX             GPIOC
#define OPMVHOR_GPIO_RX             GPIOD
#define OPMVHOR_IRQn                UART5_IRQn
#define OPMVHOR_USART               UART5
#define OPMVHOR_RCC_USART           RCC_APB1Periph_UART5
#define OPMVHOR_RCC_GPIO_TX         RCC_APB2Periph_GPIOC
#define OPMVHOR_RCC_GPIO_RX         RCC_APB2Periph_GPIOD
#define OPMVHOR_PreemptionPriority  9                       // 抢占优先级
#define OPMVHOR_SubPriority         0                       // 子优先级
#define OPMVHOR_BaudRate            115200                   // 波特率

// 缓冲区大小
// 通用
#define OPMV_RXBUFF_SIZE    64
#define OPMV_TXBUFF_SIZE    8

/* ------------------------------------- 数据包格式 -------------------------------------
字节数      数据        说明
1           0xFF       包头
1           0x         字节长度（数据部分） 0~254
1           0x         该字节用于表示数据类型
n           ...        data部分
1           0x         校验和，对数据部分累加取低八位

*/
// 通用
#define OPMV_PACK_HEAD  0xff
#define OPMV_RXPACK_UNFINISHED      0   // 开始了但是没结束
#define OPMV_RXPACK_FINISHED        1   // 成功完成
#define OPMV_RXPACK_CHECK_FAILED    2   // 出现错误
#define OPMV_RXPACK_UNKNOWN_FAILED  3   // 未知错误
#define OPMV_RXSTA_HEAD_BIT         0x8000 // bit15 标记收到包头
#define OPMV_RXSTA_LEN_BIT          0x4000 // bit14 标记收报数据长度位
#define OPMV_RXSTA_TYPE_BITS        0x3800 // bit13,12,11 标记接收到的数据类型
#define OPMV_RXSTA_DATA_BIT         0x400  // bit10 标记数据部分接收完成
#define OPMV_RXSTA_CNT_BITS         0xff   // bit7~0 对数据部分的计数

/* stm32 接收和发送openmv数据的ID */
typedef enum _OPMV_REC_ID{
    OPMV_REC_ID_1=1, 
    OPMV_REC_ID_2, 
    OPMV_REC_ID_3, 
    OPMV_REC_ID_4, 
}OPMV_REC_ID;

typedef enum _OPMV_SEND_ID{
    OPMV_SEND_ID_1=1,
    OPMV_SEND_ID_2,
    OPMV_SEND_ID_3, 
    OPMV_SEND_ID_4, 
}OPMV_SEND_ID;

/* stm32 接收和发送水平方向openmv数据的ID */
typedef enum _OPMVHOR_REC_ID{
    OPMVHOR_REC_ID_1 = 1, 
    OPMVHOR_REC_ID_2, 
    OPMVHOR_REC_ID_3,
    OPMVHOR_REC_ID_4, 
}OPMVHOR_REC_ID;

typedef enum _OPMVHOR_SEND_ID{
    OPMVHOR_SEND_ID_1 = 1, 
    OPMVHOR_SEND_ID_2, 
    OPMVHOR_SEND_ID_3, 
    OPMVHOR_SEND_ID_4, 
}OPMVHOR_SEND_ID;


/**
 * @brief 存放来自openmv的数据，用于外部调用
 * 
 */
typedef struct _opmvInterface{
    // REC_ID = 2
    float pkg2_flag; 
    float pkg2_x; 
    float pkg2_y;

    // REC_ID = 4
    float pkg4_recognition_result; 
    float pkg4_recognition_number; 

}opmvInterface;

/**
 * @brief 存放来自水平openmv的数据，用于外部调用
 * 
 */
typedef struct _opmvHorInterface{
    float pkg4_recognition_result; 
    float pkg4_recognition_number; 
}opmvHorInterface;


/**
 * @brief openmv所使用串口初始化
 * 
 */
void initOPMV(void);

/**
 * @brief 水平openmv所用串口初始化
 * 
 */
void initOPMVHOR(void);


/**
 * @brief openmv数据解析任务
 * 
 * @param pvParameters 
 */
void task_opmvHandle(void *pvParameters);

/**
 * @brief openmv数据解析任务
 * 
 * @param pvParameters 
 */
void task_opmvHorHandle(void *pvParameters);

/**
 * @brief 向openmv发送指令
 * 
 * @param sendID 
 */
void opmvSendPakage(OPMV_SEND_ID sendID);

/**
 * @brief 向水平openmv发送指令
 * 
 * @param sendID 
 */
void opmvHorSendPakage(OPMVHOR_SEND_ID sendID);

#endif
