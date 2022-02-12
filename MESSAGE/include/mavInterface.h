#ifndef MAVINTERFACE_H_
#define MAVINTERFACE_H_


#include "mavCommon.h"
#include "mavMsg.h"
#include "mavStartOpticalFlow.h"
#include "console.h"


/* mavlink所使用的串口及其GPIO配置 */
#define MAV_PIN_TX               GPIO_Pin_10             // TX: PA2
#define MAV_PIN_RX               GPIO_Pin_11             // RX: PA3
#define MAV_GPIO                 GPIOB
#define MAV_IRQn                 USART3_IRQn
#define MAV_USART                USART3
#define MAV_RCC_USART            RCC_APB1Periph_USART3
#define MAV_RCC_GPIO             RCC_APB2Periph_GPIOB
#define MAV_PreemptionPriority   6                       // 抢占优先级
#define MAV_SubPriority          0                       // 子优先级
#define MAV_BaudRate             57600                   // 波特率


#define MAV_BUFFER_SIZE         512                     // Mavlink数据缓冲区

/**
 * @brief 初始化mavlink
 *      初始化串口及其GPIO，以及mavlink通信接口
 * 
 */
void mavlinkInit(void);
void task_mavHandle(void *pvParameters);
void apiInit(Autopilot_Interface * api);


/**
 * @brief 发送mavlink数据到autopilot
 * 
 * @param message 待发送的mavlink message
 */
void mavSendMessage(mavlink_message_t * message);


/* ------------------------------- 应用层 ------------------------------- */

/**
 * @brief 更新目标位置
 * 
 * @param sp 
 */
void mavUpdateSetPoint(mavlink_set_position_target_local_ned_t sp);

/**
 * @brief 向autopilot写目标位置与速度信息
 * 
 */
void mavWriteSetPoint(void);

/**
 * @brief   使能offboard控制
 * 
 */
void mavEnableOffboardCtrl(void);

/**
 * @brief   arm
 * 
 */
void mavArm(void);

/**
 * @brief disarm 
 * 
 */
void mavDisArm(void);

/**
 * @brief 将px4设置成offboard模式
 * 
 */
void mavSetOffbordMode(void);

/**
 * @brief 将px4设置成自动起飞模式
 * 
 */
void mavSetAutoTakeoff(void);


/**
 * @brief       设置位置
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param sp 
 */
void mavSetPosition(float x, float y, float z, mavlink_set_position_target_local_ned_t *sp);

/**
 * @brief 设置速度
 * 
 * @param vx 
 * @param vy 
 * @param vz 
 * @param sp 
 */
void mavSetVelocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t *sp);

/**
 * @brief 设置角速度
 * 
 * @param omega 
 * @param sp 
 */
void mavSetAngleVelocity(float omega, mavlink_set_position_target_local_ned_t *sp);


/**
 * @brief 设置姿态角和角速度为0
 * 
 */
void mavSetAttitudeZero(void);


/**
 * @brief 打开光流
 * 
 */
void mavStartOpticalFlow(void);

/**
 * @brief 校准加速度计
 * 
 * @param id    1: accelerometer calibration, 2: board level calibration, 
 *              3: accelerometer temperature calibration, 4: simple accelerometer calibration
 */
void mavAccelerometerCalibration(uint8_t id);





extern Autopilot_Interface api;
extern SemaphoreHandle_t mutexSemphr;      // 互斥信号量，用于给autopilot传递位置信息


#endif
