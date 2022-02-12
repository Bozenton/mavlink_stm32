#include "controller.h"
#include "fsmCondition.h"

extern Autopilot_Interface api;
extern TaskHandle_t CtrlStartTask_Handler;
extern SemaphoreHandle_t mutexSemphr;       // 互斥信号量，用于给autopilot传递位置信息
extern SemaphoreHandle_t ctrlStartSemphr;	// 启动控制
extern QueueHandle_t ctrlQueue;             // 控制指令流
extern QueueHandle_t fsmEventQueue;         // 向FSM传递事件的condition

/**
 * @brief 启动控制
 * 
 * @param pvParameters 
 */
void task_ctrlStart(void* pvParameters)
{
    // --------------------------- 启动光流 ---------------------------
    mavStartOpticalFlow();
    delay_ms(1000);

//    // -------------------------- 校准加速度计 ------------------------
//    mavAccelerometerCalibration(1);
//    delay_ms(1000);
//    mavAccelerometerCalibration(2);
//    delay_ms(1000);
    // mavAccelerometerCalibration(4);
    // delay_ms(200);

    // -------------------- 等待初始时的位置信息 --------------------
    #if SWITCH_DEBUG
    printf("waiting for the first pos\r\n");
    #endif
    while( api.current_messages.time_stamps.local_position_ned == 0 
        || api.current_messages.local_position_ned.z == 0 
        || api.current_messages.time_stamps.attitude == 0)
    {
        delay_ms(400); // 等待第一个位置信息的到来
        // 此处必须要有vTaskDelay（在delay_ms中）使得能够允许其他任务调度
    }
    // api.initial_position.x = api.current_messages.local_position_ned.x;
    // api.initial_position.y = api.current_messages.local_position_ned.y;
    // api.initial_position.z = api.current_messages.local_position_ned.z;
    // api.initial_position.vx = api.current_messages.local_position_ned.vx;
    // api.initial_position.vy = api.current_messages.local_position_ned.vy;
    // api.initial_position.vz = api.current_messages.local_position_ned.vz;
    // api.initial_position.yaw = api.current_messages.attitude.yaw;
    // api.initial_position.yaw_rate = api.current_messages.attitude.yawspeed;
	


    #if SWITCH_CTRL && SWITCH_BT
    // --------------------------- 等待蓝牙启动指令 ---------------------------
    BaseType_t err;
    while(1)
    {
        #if SWITCH_DEBUG
        printf("wait for the bt cmd\r\n");
        #endif
        // err_ctrl = xQueueReceive(ctrlQueue, &cmd, portMAX_DELAY);
        err = xSemaphoreTake(ctrlStartSemphr, portMAX_DELAY);
        if(err == pdTRUE)
        {
            break;
        }
    }


    mavlink_set_position_target_local_ned_t sp;
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY & 
                    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.vx = 0.0;
    sp.vy = 0.0;
    sp.vz = 1.0;
    sp.yaw_rate = 0.0;
    sp.target_system = api.target_sys_id;
    sp.target_component = api.target_comp_id;
    sp.time_boot_ms = xTaskGetTickCount();
    xSemaphoreTake(mutexSemphr, portMAX_DELAY);
    api.current_setpoint = sp;
    xSemaphoreGive(mutexSemphr);

	int i=0;
	for(i=0; i<10; i++)
	{
		mavWriteSetPoint();
		delay_ms(100);
	}


    // printf("send offboard commands\r\n");

    mavEnableOffboardCtrl();
    delay_us(100);
	
    mavArm();
    delay_us(100);
	
	mavSetOffbordMode();


    // printf("take off !\r\n");
    #if SWITCH_FSM
        // 向状态机发送起飞condition
        uint8_t cond = CONDITION_TAKEOFF;
        if(fsmEventQueue != NULL)
            xQueueSend(fsmEventQueue, &cond, portMAX_DELAY);
        
    #else
        // take off
        api.initial_position.x = api.current_messages.local_position_ned.x;
        api.initial_position.y = api.current_messages.local_position_ned.y;
        api.initial_position.z = api.current_messages.local_position_ned.z;
        api.initial_position.vx = api.current_messages.local_position_ned.vx;
        api.initial_position.vy = api.current_messages.local_position_ned.vy;
        api.initial_position.vz = api.current_messages.local_position_ned.vz;
        api.initial_position.yaw = api.current_messages.attitude.yaw;
        api.initial_position.yaw_rate = api.current_messages.attitude.yawspeed;

        mavlink_set_position_target_local_ned_t ip = api.initial_position;
        float takeoffHeight = api.targetHeight;
        mavSetPosition( ip.x ,       // [m]
                        ip.y ,       // [m]
                        ip.z - takeoffHeight , // [m]
                        &sp         );
        sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
        mavUpdateSetPoint(sp);
        mavWriteSetPoint();
    #endif
    

    #endif

    vTaskDelete(CtrlStartTask_Handler);
}

/**
 * @brief 向autopilot写位置信息
 * 
 * @param pvParameters 
 */
void task_ctrlWritePos(void * pvParameters)
{
    mavlink_set_position_target_local_ned_t sp;
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY & 
                    MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.vx = 0.0;
    sp.vy = 0.0;
    sp.vz = 0.0;
    sp.yaw_rate = 0.0;
    sp.target_system = api.target_sys_id;
    sp.target_component = api.target_comp_id;
    sp.time_boot_ms = xTaskGetTickCount();
    xSemaphoreTake(mutexSemphr, portMAX_DELAY);
    api.current_setpoint = sp;
    xSemaphoreGive(mutexSemphr);

    mavWriteSetPoint();

    while(1)
    {
        // px4要求在offboard控制时，以2Hz以上的速度更新控制信息
        delay_ms(100); // 以5Hz发送
        mavWriteSetPoint();
		
        #if SWITCH_DEBUG
//        printf("Current set point is:  \r\n\tx=%.4f, y=%.4f, z=%.4f, \r\n\tvx=%.4f, vy=%.4f, vz=%.4f \r\n", 
//                api.current_setpoint.x, api.current_setpoint.y, api.current_setpoint.z, 
//                api.current_setpoint.vx, api.current_setpoint.vy, api.current_setpoint.vz); 
        // printf("setting point \r\n");
        #endif
    }
}


/**
 * @brief 接收来自蓝牙的控制信号，然后更新目标位置
 * 
 * @param pvParameters 
 */
void task_btCtrl(void * pvParameters)
{
    BaseType_t err;
    uint8_t cmd = CTRL_NULL;
    mavlink_set_position_target_local_ned_t sp;
    while(1)
    {
        if(ctrlQueue != NULL)
        {
            err = xQueueReceive(ctrlQueue, &cmd, portMAX_DELAY);
            if(err == errQUEUE_EMPTY)
                printf("The ctrl queue is empty\r\n");
            switch (cmd)
            {
                case CTRL_UP:
                {
                    mavSetVelocity(0, 0, -VEL, &sp);
                    mavUpdateSetPoint(sp);
                    #if SWITCH_DEBUG
                        // printf("Flying up\r\n");
                    #endif
                    break;
                }
                case CTRL_DOWN:
                {
                    mavSetVelocity(0, 0, VEL, &sp);
                    mavUpdateSetPoint(sp);
                    #if SWITCH_DEBUG
                        // printf("Flying down\r\n");
                    #endif
                    break;
                }
                case CTRL_FORWARD:
                {
                    mavSetVelocity(-VEL, 0, 0, &sp);
                    mavUpdateSetPoint(sp);
                    #if SWITCH_DEBUG
                        // printf("Flying forward\r\n");
                    #endif
                    break;
                }
                case CTRL_BACKWARD:
                {
                    mavSetVelocity(VEL, 0, 0, &sp);
                    mavUpdateSetPoint(sp);
                    #if SWITCH_DEBUG
                        // printf("Flying backward\r\n");
                    #endif
                    break;
                }
                case CTRL_LEFT:
                {
                    mavSetVelocity(0, VEL, 0, &sp);
                    mavUpdateSetPoint(sp);
                    #if SWITCH_DEBUG
                        // printf("Flying left\r\n");
                    #endif
                    break;
                }
                case CTRL_RIGHT:
                {
                    mavSetVelocity(0, -VEL, 0, &sp);
                    mavUpdateSetPoint(sp);
                    #if SWITCH_DEBUG
                        // printf("Flying right\r\n");
                    #endif
                    break;
                }
                case CTRL_LEFT_ROTATE:
                {
                    mavSetAngleVelocity(-0.4, &sp);
                    mavUpdateSetPoint(sp);
                    #if SWITCH_DEBUG
                        // printf("Rotating left\r\n");
                    #endif 
                    break;
                }
                case CTRL_RIGHT_ROTATE:
                {
                    mavSetAngleVelocity(0.4, &sp);
                    mavUpdateSetPoint(sp);
                    #if SWITCH_DEBUG
                        // printf("Rotating right\r\n");
                    #endif 
                    break;
                }
                case CTRL_NULL:
                {
                    mavSetVelocity(0, 0, 0, &sp);
                    mavUpdateSetPoint(sp);
                    break;
                }
                default:
                    break;
            } // end switch 控制信号
            cmd = CTRL_NULL;
        }// end 检查控制队列是否创建成功
    }// end while循环
}







