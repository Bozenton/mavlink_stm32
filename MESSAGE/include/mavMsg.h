#ifndef MAVMSG_H_
#define MAVMSG_H_

#include "mavCommon.h"


/**
 * @brief 解析来自autopilot的mavlink数据
 * 
 * @param message 待解析的数据
 * @param api 存放解析后数据的结构体
 */
void mavHandle(mavlink_message_t * message, Autopilot_Interface* api);

/**
 * @brief 校准
 *      Trigger calibration. This command will be only accepted if in pre-flight mode.
 *      Except for Temperature Calibration, only one sensor should be set in a single message
 *      and all others should be zero.
 * @param msg               待打包的mavlink message
 * @param sys_id            本系统的id
 * @param comp_id           本component的id
 * @param target_sys_id     目标系统的id
 * @param target_comp_id    目标component的id
 */
void mav_cmd_calibration(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, 
                            uint8_t target_sys_id, uint8_t target_comp_id);

/**
 * @brief 校准加速度计
 * 
 * @param msg               待打包的mavlink message
 * @param sys_id            本系统的id
 * @param comp_id           本component的id
 * @param target_sys_id     目标系统的id
 * @param target_comp_id    目标component的id
 * @param id                1: accelerometer calibration, 2: board level calibration, 
 *                          3: accelerometer temperature calibration, 4: simple accelerometer calibration
 */
void mav_cmd_calibration_accelerometer(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint8_t id);


/**
 * @brief Request a data stream.
 * 
 * @param msg               待打包的mavlink message
 * @param sys_id            本系统的id
 * @param comp_id           本component的id
 * @param target_sys_id     目标系统的id
 * @param msg_stream_id     The ID of the requested data stream
 *                          0 to 12 corresponding to the group of messages (see MAV_DATA_STREAM). 
 *                          See the “Using SRx Parameters” section above to determine exactly 
 *                          which messages are in each group
 */
void mav_request_data_stream(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, 
                                uint8_t target_sys_id, uint8_t msg_stream_id);

/**
 * @brief Set the interval between messages for a particular MAVLink message ID.
 * 
 * @param msg               待打包的mavlink message
 * @param sys_id            本系统的id
 * @param comp_id           本component的id
 * @param target_sys_id     目标系统的id
 * @param target_comp_id    目标component的id
 * @param msgid             欲得到的message的id
 * @param itv               The interval between two messages. Set to -1 to disable and 0 to request default rate.
 */
void mav_cmd_set_msg_itv(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id,
                          uint8_t target_sys_id, uint8_t target_comp_id,
                          uint32_t msgid, uint32_t itv);


/**
 * @brief Arms / Disarms a component
 * 
 * @param msg               待打包的mavlink message
 * @param sys_id            本系统的id
 * @param comp_id           本component的id
 * @param target_sys_id     目标系统的id
 * @param target_comp_id    目标component的id
 * @param flag              0: disarm, 1: arm
 */
void mav_cmd_arm_disarm(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, 
                        uint8_t target_sys_id, uint8_t target_comp_id, uint8_t flag);

/**
 * @brief hand control over to an external controller
 * 
 * @param msg               待打包的mavlink message
 * @param sys_id            本系统的id
 * @param comp_id           本component的id
 * @param target_sys_id     目标系统的id
 * @param target_comp_id    目标component的id
 * @param flag              On / Off (> 0.5f on)
 */
void mav_cmd_toggle_offboard(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, 
                                uint8_t target_sys_id, uint8_t target_comp_id, bool flag);


/**
 * @brief 将px4设置为offboard模式
 * 
 * @param msg 
 * @param sys_id 
 * @param comp_id 
 * @param target_sys_id 
 * @param target_comp_id 
 */
void mav_cmd_set_offboard_mode(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t target_comp_id);

/**
 * @brief 将px4设置为自动起飞模式
 * 
 * @param msg 
 * @param sys_id 
 * @param comp_id 
 * @param target_sys_id 
 * @param target_comp_id 
 */
void mav_cmd_set_autotakeoff_mode(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t target_comp_id);


/**
 * @brief 设置位置
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param sp 
 */
void mav_set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t *sp);

/**
 * @brief 设置速度
 * 
 * @param vx 
 * @param vy 
 * @param vz 
 * @param sp 
 */
void mav_set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t *sp);

/**
 * @brief 设置转角
 * 
 * @param yaw 
 * @param sp 
 */
void mav_set_yaw(float yaw, mavlink_set_position_target_local_ned_t *sp);

/**
 * @brief 设置角速度
 * 
 * @param yaw_rate 
 * @param sp 
 */
void mav_set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t *sp);

/**
 * @brief 设置速度
 * 
 * @param vx 
 * @param vy 
 * @param vz 
 * @param sp 
 */
void mav_set_velocity_body(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t *sp);


/**
 * @brief 设置偏航角
 * 
 * @param yaw 
 * @param sp 
 */
void mav_set_yaw_body(float yaw, mavlink_set_position_target_local_ned_t *sp);


/**
 * @brief 设置姿态角和角速度
 * 
 * @param q1 
 * @param q2 
 * @param q3 
 * @param q4 
 * @param roll_rate 
 * @param pitch_rate 
 * @param yaw_rate 
 * @param sa 
 */
void mav_set_attitude_q_and_bodyrate(float q1, float q2, float q3, float q4, float roll_rate, float pitch_rate, float yaw_rate,  mavlink_set_attitude_target_t *sa);



#endif

