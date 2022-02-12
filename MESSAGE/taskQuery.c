#include "taskQuery.h"

extern SemaphoreHandle_t taskQuerySemphr;  // 查看任务信息

/**
 * @brief 查询各个任务的堆栈历史剩余最小值
 * 
 * @param pvParameters 
 */
void task_taskQuery(void *pvParameters)
{
    BaseType_t err;
    TaskHandle_t taskHandle;
    TaskStatus_t taskStatus;
    uint8_t taskenum = 0;

    while(1)
    {
        if( taskQuerySemphr != NULL )
        {
            #if SWITCH_DEBUG
            printf("wait for taks query\r\n");
            #endif
            err = xSemaphoreTake(taskQuerySemphr, portMAX_DELAY);
            if(err == pdTRUE)
            {
                err = pdFALSE;
                switch (taskenum)
                {
                #if SWITCH_MAV
                case 0:
                    taskHandle = xTaskGetHandle("task_mavHandle");
                    printf("task_mavHandle\r\n");
                    break;
                #endif
                #if SWITCH_BT
                case 1:
                    taskHandle = xTaskGetHandle("task_BtSend");
                    printf("task_BtSend\r\n");
                    break;
                case 2:
                    taskHandle = xTaskGetHandle("task_BtReceive");
                    printf("task_BtReceive\r\n");
                    break;
                #endif
                #if SWITCH_OPMV
                case 3:
                    taskHandle = xTaskGetHandle("task_opmvHandle");
                    printf("task_opmvHandle\r\n");
                    break;
                #endif
                #if SWITCH_CTRL
                case 4:
                    taskHandle = xTaskGetHandle("task_ctrlStart");
                    printf("task_ctrlStart\r\n");
                    break;
                case 5:
                    taskHandle = xTaskGetHandle("task_ctrlWritePos");
                    printf("task_ctrlWritePos\r\n");
                    break;
                #endif
                #if SWITCH_CTRL && SWITCH_BT
                case 6:
                    taskHandle = xTaskGetHandle("task_btCtrl");
                    printf("task_btCtrl\r\n");
                    break;
                #endif
                default:
                    break;
                }
                
                vTaskGetInfo((TaskHandle_t	)taskHandle, 		//任务句柄
				            (TaskStatus_t*	)&taskStatus, 		//任务信息结构体
				            (BaseType_t	)pdTRUE,			//允许统计任务堆栈历史最小剩余大小
			                (eTaskState	)eInvalid);			//函数自己获取任务运行壮态
                printf("任务堆栈历史剩余最小值:%d\r\n", taskStatus.usStackHighWaterMark);
                taskenum ++;
                if(taskenum>6)
                    taskenum = 0;
            }
            else
                delay_ms(10);
        }
        else
            delay_ms(10);
    }
}

