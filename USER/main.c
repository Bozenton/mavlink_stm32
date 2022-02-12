#include "stm32f10x.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// 自定义文件
#include "mavInterface.h"
#include "ultrasound.h"
#include "bluetooth.h"
#include "openmv.h"
#include "key.h"
#include "led.h"
#include "console.h"
#include "controller.h"
#include "taskQuery.h"
#include "servo.h"
#include "gimbal.h"
#include "finiteStateMachine.h"
#include "laser.h"


/* ----------------------------------------- 任务 ----------------------------------------- */
// 开始任务
#define START_TASK_PRIO		1           //任务优先级
#define START_STK_SIZE 		256         //任务堆栈大小	
TaskHandle_t StartTask_Handler;         //任务句柄
void task_start(void *pvParameters);    //任务函数

// mavlink解析任务
#define MAVHANDLE_TASK_PRIO     3
#define MAVHANDLE_STK_SIZE      256
TaskHandle_t MavHandleTask_Handler;

// 超声波测距任务
#define UTS_TASK_PRIO    4
#define UTS_STK_SIZE     256
TaskHandle_t UtsHandleTask_Handler;

// 蓝牙发送任务
#define BTSEND_TASK_PRIO    4
#define BTSEND_STK_SIZE     256
TaskHandle_t BtSendTask_Handler;

// 蓝牙接收任务
#define BTREC_TASK_PRIO     3
#define BTREC_STK_SIZE      256
TaskHandle_t BtRecTask_Handler;

// openmv解析任务
#define OPMV_TASK_PRIO  3
#define OPMV_STK_SIZE   256
TaskHandle_t OpmvHandleTask_Handler;

// 水平openmv解析任务
#define OPMVHOR_TASK_PRIO  3
#define OPMVHOR_STK_SIZE   256
TaskHandle_t OpmvHorHandleTask_Handler;

#define GIMBAL_TASK_PRIO    3
#define GIMBAL_STK_SIZE     128
TaskHandle_t GimbalTask_Handler;

// -------------- 有关控制的一系列任务 --------------
// 启动控制
#define CTRLSTART_TASK_PRIO     7
#define CTRLSTART_STK_SIZE      256
TaskHandle_t CtrlStartTask_Handler;

// 写位置信息
#define CTRLWRITEPOS_TASK_PRIO  6
#define CTRLWRITEPOS_STK_SIZE   256
TaskHandle_t CtrlWritePosTask_Handler;

// 蓝牙控制
#define BTCTRL_TASK_PRIO        5
#define BTCTRL_STK_SIZE         256
TaskHandle_t BtCtrlTask_Handler;

// --------------- 调试任务 ---------------
// Debug任务，可以把各种测试指令放在这里面
#if SWITCH_DEBUG
#define DEBUG_TASK_PRIO     2
#define DEBUG_STK_SIZE      128
TaskHandle_t DebugTask_Handler;
void task_debugHandle(void* pvParameters);
#endif

// 查看各个任务的历史最小剩余堆栈值
#if SWITCH_QUERY
#define QUERY_TASK_PRIO     8
#define QUERY_STK_SIZE      256
TaskHandle_t QueryTask_Handler;
#endif

// ---------------- 有关状态机的一系列任务 ----------------
#if SWITCH_FSM
// 有限状态机状态更新
#define FSMHANDLEEVENT_TASK_PRIO    5
#define FSMHANDLEEVENT_STK_SIZE     512
TaskHandle_t FsmHandleEventTask_Handler;
// 各个状态中执行的动作
#define FSMACTION_TASK_PRIO     2
#define FSMACTION_STK_SIZE      512
TaskHandle_t FsmActionTask_Handler;

// ------------------ 飞行 ------------------
#define FIXHEIGHT_TASK_PRIO         5
#define FIXHEIGHT_STK_SIZE          256
TaskHandle_t FixHeightTask_Handler; 

#endif




/* ----------------------------------------- 信号量 ----------------------------------------- */
SemaphoreHandle_t mavRecSemphr;         // mavlink串口接收数据完成标志
SemaphoreHandle_t opmvRecSemphr;        // openmv串口接收数据完成标志
SemaphoreHandle_t opmvHorRecSemphr;     // 水平openmv串口接收数据完成标志
SemaphoreHandle_t opmvHandleSemphr;     // openmv解析完成标志
SemaphoreHandle_t opmvHorHandleSemphr;  // 水平openmv解析完成标志
SemaphoreHandle_t btRecSemphr;          // 蓝牙串口接收数据完成标志
SemaphoreHandle_t taskQuerySemphr;      // 查看任务信息
SemaphoreHandle_t ctrlStartSemphr;	    // 启动控制

SemaphoreHandle_t mutexSemphr;      // 互斥信号量，用于给autopilot传递位置信息
SemaphoreHandle_t ctrlMutexSemphr;  // 互斥信号量，用于给控制指令流发送消息

/* ----------------------------------------- 队列 ----------------------------------------- */
#define DISQUEUE_NUM    1
QueueHandle_t distanceQueue;

#define CTRLQUEUE_NUM 6
QueueHandle_t ctrlQueue;      // 控制指令流

// #define MAVAPIQUEUE_NUM 1
// QueueHandle_t mavApiQueue;          // mavlink api

#define FSMEVENTQUEUE_NUM 3
QueueHandle_t fsmEventQueue;         // 向FSM传递事件的condition

/* ----------------------------------------- 其他全局变量 ----------------------------------------- */
Autopilot_Interface api;
opmvInterface omi;
opmvHorInterface omhi;
RxPack rxpack;

#if SWITCH_GIMBAL
Gimbal_t Gimbal;
#endif

int main(void)
{
    delay_init();
	keyInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    uart_init(115200);
	LED_Init();
    initLaser();
	LASER = 1;
	
	#if SWITCH_UTS
    ultrasoundInit();
	#endif
	
	#if SWITCH_BT
    initBT();           // 蓝牙初始化
	#endif
	
	#if SWITCH_MAV
	mavlinkInit();
	apiInit(&api);
	#endif

    #if SWITCH_OPMV
    initOPMV();
    #endif

    #if SWITCH_OPMVHOR
    initOPMVHOR();
    #endif

    #if SWITCH_GIMBAL
    initServo();
    Gimbal_StructInit(&Gimbal);
    #endif

    #if SWITCH_FSM
    fsmInit();
    #endif


    
	#if SWITCH_TASK
	//创建开始任务
    xTaskCreate((TaskFunction_t )task_start,            //任务函数
                (const char*    )"task_start",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄 
    vTaskStartScheduler();          //开启任务调度
	
	#else
	while(1)
	{
		;
	}
	#endif
}


//开始任务任务函数
void task_start(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
    #if SWITCH_MAV
	// 创建二值信号量
    mavRecSemphr = xSemaphoreCreateBinary();
    // 创建mavlink解析任务
    xTaskCreate((TaskFunction_t )task_mavHandle,             
               (const char*    )"task_mavHandle",           
               (uint16_t       )MAVHANDLE_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )MAVHANDLE_TASK_PRIO,        
               (TaskHandle_t*  )&MavHandleTask_Handler);   
    #endif

    #if SWITCH_UTS
    // 创建队列
    distanceQueue = xQueueCreate(DISQUEUE_NUM, sizeof(float));
    // 创建超声波测距任务
    xTaskCreate((TaskFunction_t )task_ultrasound,             
                (const char*    )"task_ultrasound",           
                (uint16_t       )UTS_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )UTS_TASK_PRIO,        
                (TaskHandle_t*  )&UtsHandleTask_Handler);   
    #endif

    #if SWITCH_BT
    // 蓝牙串口接收数据完成标志
    btRecSemphr = xSemaphoreCreateBinary();
    // 创建蓝牙发送任务
    xTaskCreate((TaskFunction_t )task_BtSend,             
                (const char*    )"task_BtSend",           
                (uint16_t       )BTSEND_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )BTSEND_TASK_PRIO,        
                (TaskHandle_t*  )&BtSendTask_Handler);  

    // 创建蓝牙接收任务
    xTaskCreate((TaskFunction_t )task_BtReceive,             
                (const char*    )"task_BtReceive",           
                (uint16_t       )BTREC_STK_SIZE,        
                (void*          )NULL,
                (UBaseType_t    )BTREC_TASK_PRIO,        
                (TaskHandle_t*  )&BtRecTask_Handler);  
    #endif

    
    #if SWITCH_OPMV
	// 创建二值信号量
    opmvRecSemphr = xSemaphoreCreateBinary();
    // 创建openmv解析任务
    xTaskCreate((TaskFunction_t )task_opmvHandle,             
               (const char*    )"task_opmvHandle",           
               (uint16_t       )OPMV_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )OPMV_TASK_PRIO,        
               (TaskHandle_t*  )&OpmvHandleTask_Handler);  
    #endif

    #if SWITCH_OPMVHOR
	// 创建二值信号量
    opmvHorRecSemphr = xSemaphoreCreateBinary();
    // 创建openmv解析任务
    xTaskCreate((TaskFunction_t )task_opmvHorHandle,             
               (const char*    )"task_opmvHorHandle",           
               (uint16_t       )OPMVHOR_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )OPMVHOR_TASK_PRIO,        
               (TaskHandle_t*  )&OpmvHorHandleTask_Handler);  
    #endif

    #if SWITCH_CTRL
    // 创建互斥信号量
    mutexSemphr = xSemaphoreCreateMutex();
    xTaskCreate((TaskFunction_t )task_ctrlStart,             
               (const char*    )"task_ctrlStart",           
               (uint16_t       )CTRLSTART_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )CTRLSTART_TASK_PRIO,        
               (TaskHandle_t*  )&CtrlStartTask_Handler);  

    xTaskCreate((TaskFunction_t )task_ctrlWritePos,             
               (const char*    )"task_ctrlWritePos",           
               (uint16_t       )CTRLWRITEPOS_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )CTRLWRITEPOS_TASK_PRIO,        
               (TaskHandle_t*  )&CtrlWritePosTask_Handler);  

    #endif

    #if SWITCH_CTRL && SWITCH_BT
    ctrlQueue = xQueueCreate(CTRLQUEUE_NUM, sizeof(float));
    ctrlStartSemphr = xSemaphoreCreateBinary();
    xTaskCreate((TaskFunction_t )task_btCtrl,             
               (const char*    )"task_btCtrl",
               (uint16_t       )BTCTRL_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )BTCTRL_TASK_PRIO,        
               (TaskHandle_t*  )&BtCtrlTask_Handler);  
    #endif

    #if SWITCH_DEBUG
    xTaskCreate((TaskFunction_t )task_debugHandle,             
               (const char*    )"task_debugHandle",
               (uint16_t       )DEBUG_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )DEBUG_TASK_PRIO,        
               (TaskHandle_t*  )&DebugTask_Handler);  
    #endif

    #if SWITCH_QUERY
    taskQuerySemphr = xSemaphoreCreateBinary();
    xTaskCreate((TaskFunction_t )task_taskQuery,             
               (const char*    )"task_taskQuery",
               (uint16_t       )QUERY_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )QUERY_TASK_PRIO,        
               (TaskHandle_t*  )&QueryTask_Handler);  
    #endif

    #if SWITCH_GIMBAL
    xTaskCreate((TaskFunction_t )task_updateServos,             
               (const char*    )"task_updateServos",
               (uint16_t       )GIMBAL_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )GIMBAL_TASK_PRIO,        
               (TaskHandle_t*  )&GimbalTask_Handler);  
    #endif

    #if SWITCH_FSM
    fsmEventQueue = xQueueCreate(FSMEVENTQUEUE_NUM, sizeof(uint8_t));
	ctrlMutexSemphr = xSemaphoreCreateMutex();
    xTaskCreate((TaskFunction_t )task_fsmHandleEvent,             
               (const char*    )"task_fsmHandleEvent",
               (uint16_t       )FSMHANDLEEVENT_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )FSMHANDLEEVENT_TASK_PRIO,        
               (TaskHandle_t*  )&FsmHandleEventTask_Handler);  
    xTaskCreate((TaskFunction_t )task_fsmAction,             
               (const char*    )"task_fsmAction",
               (uint16_t       )FSMACTION_STK_SIZE,        
               (void*          )NULL,                  
               (UBaseType_t    )FSMACTION_TASK_PRIO,
               (TaskHandle_t*  )&FsmActionTask_Handler);  
    #endif
    
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}




#if SWITCH_DEBUG

void task_debugHandle(void* pvParameters)
{
    while(1)
    {
        // // opmvHorSendPakage(OPMVHOR_SEND_ID_4);
        // opmvSendPakage(OPMV_SEND_ID_2);
        // delay_ms(500);
        // printf("flag = %f, x = %f, y = %f \r\n", omi.pkg2_flag, omi.pkg2_x, omi.pkg2_y);
    }
}

#endif
