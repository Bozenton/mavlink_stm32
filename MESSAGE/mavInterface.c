#include "mavInterface.h"


/**
 * @brief 初始化Autopilot_Interface
 * 
 * @param api 待初始化的Autopilot_Interface指针
 */
void apiInit(Autopilot_Interface * api)
{
    api->sys_id = SYS_ID;
    api->comp_id = COMP_ID;
    api->target_sys_id = TARGET_SYS_ID;
    api->target_comp_id = TARGET_COMP_ID;

    api->current_messages.local_position_ned.z = 0;
    api->current_messages.time_stamps.local_position_ned = 0;
    api->current_messages.time_stamps.optical_flow_rad = 0;
    api->current_messages.time_stamps.attitude = 0;

    api->current_setpoint.coordinate_frame = MAV_FRAME_LOCAL_NED;
}


void mavlinkInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(MAV_RCC_USART, ENABLE);
    RCC_APB2PeriphClockCmd(MAV_RCC_GPIO, ENABLE);

    // USART3_TX PB10
    GPIO_InitStructure.GPIO_Pin = MAV_PIN_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_Init(MAV_GPIO, &GPIO_InitStructure); // 初始化GPIOA.2

    // USART3_RX PB11
    GPIO_InitStructure.GPIO_Pin = MAV_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_Init(MAV_GPIO, &GPIO_InitStructure); // 初始化GPIOA.3

    // Usart3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = MAV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MAV_PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = MAV_SubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART 初始化设置
    USART_InitStructure.USART_BaudRate = MAV_BaudRate;           // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;         // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式

    USART_Init(MAV_USART, &USART_InitStructure);         // 初始化串口
    USART_ClearITPendingBit(MAV_USART, USART_IT_RXNE);
    USART_ITConfig(MAV_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(MAV_USART, ENABLE);                        // 使能串口
	
}



/**
 * @brief Mavlink串口中断服务函数
 * 
 */

uint8_t recbyte;
mavlink_message_t recMsg;
mavlink_status_t recStatus;
extern SemaphoreHandle_t mavRecSemphr;	//二值信号量句柄

void USART3_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken;

    if(USART_GetITStatus(MAV_USART, USART_IT_RXNE))
    {
        recbyte = USART_ReceiveData(MAV_USART);
        uint8_t flag = mavlink_parse_char(MAVLINK_COMM_0, recbyte, &recMsg, &recStatus);
        if(flag == MAVLINK_FRAMING_OK && mavRecSemphr != NULL)
        {
            xSemaphoreGiveFromISR(mavRecSemphr, &xHigherPriorityTaskWoken);	//释放二值信号量
		    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
        }
    }

}

/**
 * @brief mavlink数据包解析任务
 *          将来自autopilot的mavlink message解析后放入api
 * 
 * @param pvParameters Freertos任务参数
 */
void task_mavHandle(void *pvParameters)
{
    BaseType_t err = pdFALSE;

    while(1)
    {
        if(mavRecSemphr != NULL)
        {
            err = xSemaphoreTake(mavRecSemphr, portMAX_DELAY);
            if(err == pdTRUE) // 若是成功读取到了信号量，信号量自动变为无效，不需要手动清除
            {
                err = pdFALSE;
                mavHandle(&recMsg, &api);
                #if SWITCH_DEBUG
                // printf("x = %.4f, y = %.4f, z = %.4f\r\n", api.current_messages.local_position_ned.x, 
                //                                api.current_messages.local_position_ned.y,
                //                                api.current_messages.local_position_ned.z);
				// printf("attitute time = %d\r\n", api.current_messages.time_stamps.attitude);
				// printf("pos time = %d\r\n", api.current_messages.time_stamps.local_position_ned);
                #endif
            }
            else 
                vTaskDelay(10);
        }
        else
            vTaskDelay(10);
    }
}




/* ------------------------------------- 发送数据（驱动层） ------------------------------------- */
/**
 * @brief mavlink串口发送数据
 * 
 * @param p 传入指针
 * @param length 字节长度
 */
void mavSend(uint8_t *p, uint16_t length)
{
	for(uint16_t i=0;i<length;i++)
    {
        USART_SendData(MAV_USART, *p++); 
        while(USART_GetFlagStatus(MAV_USART, USART_FLAG_TXE) == RESET) 
        {
        } 
    }
}

/**
 * @brief 发送mavlink数据到autopilot
 * 
 * @param message 待发送的mavlink message
 */
void mavSendMessage(mavlink_message_t * message)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, message);
    mavSend(buf, len);
}


/* ------------------------------------- 发送各种数据包（应用层） ------------------------------------- */
/**
 * @brief 更新目标位置
 * 
 * @param sp 
 */
void mavUpdateSetPoint(mavlink_set_position_target_local_ned_t sp)
{
    xSemaphoreTake(mutexSemphr, portMAX_DELAY);
    api.current_setpoint = sp;
    xSemaphoreGive(mutexSemphr);
}



/**
 * @brief   向autopilot写目标位置与速度信息
 *          位置信息来源于api.current_setpoint
 * 
 */
void mavWriteSetPoint(void)
{
    /* pack */
    mavlink_set_position_target_local_ned_t sp;
    xSemaphoreTake(mutexSemphr, portMAX_DELAY);
    sp = api.current_setpoint;
    xSemaphoreGive(mutexSemphr);

    sp.time_boot_ms = xTaskGetTickCount();
    sp.target_component = api.target_comp_id;
    sp.target_system = api.target_sys_id;


    /* encode */
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_encode(SYS_ID, COMP_ID, &msg, &sp);

    /* write */
    mavSendMessage(&msg);
}



/**
 * @brief   使能offboard控制
 * 
 */
void mavEnableOffboardCtrl(void)
{
    mavlink_message_t msg;
    mav_cmd_toggle_offboard(&msg, SYS_ID, COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, 1);
    mavSendMessage(&msg);
}


/**
 * @brief   arm
 * 
 */
void mavArm(void)
{
    mavlink_message_t msg;
    mav_cmd_arm_disarm(&msg, SYS_ID, COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, true);
    mavSendMessage(&msg);
}

/**
 * @brief disarm 
 * 
 */
void mavDisArm(void)
{
    mavlink_message_t msg;
    mav_cmd_arm_disarm(&msg, SYS_ID, COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, false);
    mavSendMessage(&msg);
}

/**
 * @brief 将px4设置成offboard模式
 * 
 */
void mavSetOffbordMode(void)
{
    mavlink_message_t msg;
    mav_cmd_set_offboard_mode(&msg, SYS_ID, COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);
    mavSendMessage(&msg);
}

void mavDisableOffboard(void)
{
    mavlink_message_t msg;
    mav_cmd_set_offboard_mode(&msg, SYS_ID, COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);
    mavSendMessage(&msg);
}

/**
 * @brief 将px4设置成自动起飞模式
 * 
 */
void mavSetAutoTakeoff(void)
{
    mavlink_message_t msg;
    mav_cmd_set_autotakeoff_mode(&msg, SYS_ID, COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID);
    mavSendMessage(&msg);
}


/**
 * @brief       设置位置
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param sp 
 */
void mavSetPosition(float x, float y, float z, mavlink_set_position_target_local_ned_t *sp)
{
    mav_set_position(x, y, z, sp);
}

/**
 * @brief 设置速度
 * 
 * @param vx 
 * @param vy 
 * @param vz 
 * @param sp 
 */
void mavSetVelocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t *sp)
{
    mav_set_velocity(vx, vy, vz, sp);
}

/**
 * @brief 设置角速度
 * 
 * @param omega 
 * @param sp 
 */
void mavSetAngleVelocity(float omega, mavlink_set_position_target_local_ned_t *sp)
{
    mav_set_velocity(0, 0, 0, sp);
    mav_set_yaw_rate(omega, sp);
}

// /**
//  * @brief 设置姿态角和角速度
//  * 
//  * @param q1 
//  * @param q2 
//  * @param q3 
//  * @param q4 
//  * @param roll_rate 
//  * @param pitch_rate 
//  * @param yaw_rate 
//  * @param sa 
//  */
// void mavSetAttitude(float q1, float q2, float q3, float q4, float roll_rate, float pitch_rate, float yaw_rate,  mavlink_set_attitude_target_t *sa)
// {
//     mav_set_attitude_q_and_bodyrate(q1, q2, q3, q4, roll_rate, pitch_rate, yaw_rate,  sa);
// }

/**
 * @brief 设置姿态角和角速度为0
 * 
 */
void mavSetAttitudeZero(void)
{
    mavlink_set_attitude_target_t sa;
    mav_set_attitude_q_and_bodyrate(1, 0, 0, 0, 
									0, 0, 0, &sa);

    sa.time_boot_ms = xTaskGetTickCount();
    sa.target_system = api.target_sys_id;
    sa.target_component = api.target_comp_id;

    /* encode */
    mavlink_message_t msg;
    mavlink_msg_set_attitude_target_encode(SYS_ID, COMP_ID, &msg, &sa);

    /* write */
    mavSendMessage(&msg);

}

/**
 * @brief 打开光流
 * 
 */
void mavStartOpticalFlow(void)
{
    mavlink_message_t msg;
    mav_start_optical_flow(&msg);
    mavSendMessage(&msg);
}


void mavTakeoff(void)
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    cmd.command = MAV_CMD_NAV_TAKEOFF;
    cmd.target_system = TARGET_SYS_ID;
    cmd.target_component = TARGET_COMP_ID;
    mavlink_msg_command_long_encode(SYS_ID, COMP_ID, &msg, &cmd);
    mavSendMessage(&msg);
}

/**
 * @brief 校准加速度计
 * 
 * @param id    1: accelerometer calibration, 2: board level calibration, 
 *              3: accelerometer temperature calibration, 4: simple accelerometer calibration
 */
void mavAccelerometerCalibration(uint8_t id)
{
    if(id > 4)
        return ;
    mavlink_message_t msg;
    mav_cmd_calibration_accelerometer(&msg, SYS_ID, COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, id);
    mavSendMessage(&msg);
}
