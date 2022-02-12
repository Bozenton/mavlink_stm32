#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "console.h"
#include "delay.h"
#include "console.h"
#include "controller.h"

#if SWITCH_FSM
#include "finiteStateMachine.h"
#endif

#if SWITCH_GIMBAL
#include "gimbal.h"
#endif


/* 蓝牙所使用的串口及其GPIO配置 */
#define BT_PIN_TX               GPIO_Pin_2              // TX: PA2
#define BT_PIN_RX               GPIO_Pin_3              // RX: PA3
#define BT_GPIO                 GPIOA
#define BT_IRQn                 USART2_IRQn
#define BT_USART                USART2
#define BT_RCC_USART            RCC_APB1Periph_USART2
#define BT_RCC_GPIO             RCC_APB2Periph_GPIOA
#define BT_PreemptionPriority   7                       // 抢占优先级
#define BT_SubPriority          0                       // 子优先级
#define BT_BaudRate             38400                   // 波特率


/* 数据包及其解析与打包 */
// 本程序通过USART 配合接收中断 进行数据包的接收和发送
// 接收的数据在接收中断中写入到buffer中，通过定时调用readValuePack()函数来解析，定时间隔建议在10ms以内。


#define PACK_HEAD 0xa5   
#define PACK_TAIL 0x5a

/// 1.指定接收缓冲区的大小 ----------------------------------------------------------------------------------
//    一般需要512字节以上，需要根据实际接收数据的速度和proc函数的频率考虑。
#define VALUEPACK_BUFFER_SIZE 128

/// 2.指定发送数据包的结构--------------------------在发送时会自动额外在前后加上包头，包尾和校验和数据，因此会多出3个字节
///   
//    根据实际需要的变量，定义数据包中 bool byte short int float 五种类型的数目

#define TX_BOOL_NUM  0
#define TX_BYTE_NUM  1
#define TX_SHORT_NUM 0
#define TX_INT_NUM   0
#define TX_FLOAT_NUM 5

/// 3.指定接收数据包的结构-----------------------------------------------------------------------------------
//    根据实际需要的变量，定义数据包中 bool byte short int float 五种类型的数目

#define RX_BOOL_NUM  2
#define RX_BYTE_NUM  1
#define RX_SHORT_NUM 2
#define RX_INT_NUM   0
#define RX_FLOAT_NUM 1


/* 发送包结构体定义 */
typedef struct   
{	
	#if TX_BOOL_NUM > 0
	unsigned char bools[TX_BOOL_NUM];
	#endif
	
	#if TX_BYTE_NUM > 0
  char bytes[TX_BYTE_NUM];
  #endif
	
	#if TX_SHORT_NUM > 0
	short shorts[TX_SHORT_NUM];	
	#endif
	
	#if TX_INT_NUM > 0
	int  integers[TX_INT_NUM];
	#endif
	
	#if TX_FLOAT_NUM > 0
	float floats[TX_FLOAT_NUM];
	#endif
	char space; //只是为了占一个空，当所有变量数目都为0时确保编译成功
}TxPack;

/* 接收包结构体定义 */
typedef struct 
{	
#if RX_BOOL_NUM > 0
	unsigned char bools[RX_BOOL_NUM];
	#endif
	
	#if RX_BYTE_NUM > 0
  char bytes[RX_BYTE_NUM];
  #endif
	
	#if RX_SHORT_NUM > 0
	short shorts[RX_SHORT_NUM];	
	#endif
	
	#if RX_INT_NUM > 0
	int  integers[RX_INT_NUM];
	#endif
	
	#if RX_FLOAT_NUM > 0
	float floats[RX_FLOAT_NUM];
	#endif
	char space; //只是为了占一个空，当所有变量数目都为0时确保编译成功
}RxPack;



/* 函数声明 */

/**
 * @brief 蓝牙使用的串口初始化
 * 
 */
void initBT(void);

/**
 * @brief 从缓冲区中读取数据包
 *      尝试从缓冲区中读取数据包
 * 
 * @param rx_pack_ptr
 *      传入接收数据结构体的指针，从环形缓冲区中读取出数据包，
 *      并将各类数据存储到rx_pack_ptr指向的结构体中
 * @return 
 *      如果成功读取到数据包，则返回1，否则返回0
 */

unsigned char readValuePack(RxPack *rx_pack_ptr);

/**
 * @brief 将发送数据结构体中的变量打包，并发送出去
 * 
 * @param tx_pack_ptr 
 * 		待发送数据包的指针 
 */
void sendValuePack(TxPack *tx_pack_ptr);

/**
 * @brief 蓝牙传输任务
 * 
 * @param pvParameters 
 */
void task_BtSend(void *pvParameters);

/**
 * @brief 蓝牙接收任务
 * 
 * @param pvParameters 
 */
void task_BtReceive(void *pvParameters);


#endif
