#include "mavMsg.h"

/**
 * @brief 解析来自autopilot的mavlink数据
 * 
 * @param message 待解析的数据
 * @param api 存放解析后数据的结构体
 */
void mavHandle(mavlink_message_t * message, Autopilot_Interface* api)
{
//    Time_Stamps this_timestamps;
    api->current_messages.sysid = message->sysid;
    api->current_messages.compid = message->compid;

    switch(message->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_msg_heartbeat_decode(message, &(api->current_messages.heartbeat));
           api->current_messages.time_stamps.heartbeat = xTaskGetTickCount();
//            this_timestamps.heartbeat = api->current_messages.time_stamps.heartbeat;
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_msg_sys_status_decode(message, &(api->current_messages.sys_status));
           api->current_messages.time_stamps.sys_status = xTaskGetTickCount();
//            this_timestamps.sys_status = api->current_messages.time_stamps.sys_status;
            break;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            mavlink_msg_local_position_ned_decode(message, &(api->current_messages.local_position_ned));
           api->current_messages.time_stamps.local_position_ned = xTaskGetTickCount();
//            this_timestamps.local_position_ned = api->current_messages.time_stamps.local_position_ned;
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            mavlink_msg_global_position_int_decode(message, &(api->current_messages.global_position_int));
           api->current_messages.time_stamps.global_position_int = xTaskGetTickCount();
        }
        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        {
            mavlink_msg_position_target_local_ned_decode(message, &(api->current_messages.position_target_local_ned));
           api->current_messages.time_stamps.position_target_local_ned = xTaskGetTickCount();
//            this_timestamps.position_target_local_ned = api->current_messages.time_stamps.position_target_local_ned;
            break;
        }
        case MAVLINK_MSG_ID_HIGHRES_IMU:
        {
            mavlink_msg_highres_imu_decode(message, &(api->current_messages.highres_imu));
           api->current_messages.time_stamps.highres_imu = xTaskGetTickCount();
//            this_timestamps.highres_imu = api->current_messages.time_stamps.highres_imu;
            break;
        }
        case MAVLINK_MSG_ID_ATTITUDE:
        {
            mavlink_msg_attitude_decode(message, &(api->current_messages.attitude));
           api->current_messages.time_stamps.attitude = xTaskGetTickCount();
//            this_timestamps.attitude = api->current_messages.time_stamps.attitude;
            break;
        }
       case MAVLINK_MSG_ID_OPTICAL_FLOW:
       {
           mavlink_msg_optical_flow_decode(message, &(api->current_messages.optical_flow));
           api->current_messages.time_stamps.optical_flow = xTaskGetTickCount();
           break;
       }

        case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
        {
            mavlink_msg_optical_flow_rad_decode(message, &(api->current_messages.optical_flow_rad));
           api->current_messages.time_stamps.optical_flow_rad = xTaskGetTickCount();
           break;
        }
        case MAVLINK_MSG_ID_ALTITUDE:
        {
            mavlink_msg_altitude_decode(message, &(api->current_messages.altitude));
            api->current_messages.time_stamps.altitude = xTaskGetTickCount();
            break;
        }
    }
}

/*
 * Trigger calibration. This command will be only accepted if in pre-flight mode.
 * Except for Temperature Calibration, only one sensor should be set in a single message
 * and all others should be zero.
 */

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
void mav_cmd_calibration(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    mavlink_command_long_t cmd;
    cmd.command = MAV_CMD_PREFLIGHT_CALIBRATION;
    cmd.target_system = target_sys_id;
    cmd.target_component = target_comp_id;
    cmd.confirmation = 0;
    cmd.param1 = 0;
    cmd.param2 = 0;
    cmd.param3 = 0;
    cmd.param4 = 0;
    cmd.param5 = 2; 
    cmd.param6 = 0;
    cmd.param7 = 0;
    mavlink_msg_command_long_encode(sys_id, comp_id, msg, &cmd);
}

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
void mav_cmd_calibration_accelerometer(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint8_t id)
{
    if(id>4)
        return ;
    mavlink_command_long_t cmd;
    cmd.command = MAV_CMD_PREFLIGHT_CALIBRATION;
    cmd.target_system = target_sys_id;
    cmd.target_component = target_comp_id;
    cmd.confirmation = 0;
    cmd.param1 = 0;
    cmd.param2 = 0;
    cmd.param3 = 0;
    cmd.param4 = 0;
    cmd.param5 = id; 
    cmd.param6 = 0;
    cmd.param7 = 0;
    mavlink_msg_command_long_encode(sys_id, comp_id, msg, &cmd);
}


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
void mav_request_data_stream(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t msg_stream_id)
{
    mavlink_request_data_stream_t rds;
    rds.target_system = target_sys_id;
    rds.target_component = 0;
    rds.req_stream_id = msg_stream_id;
    rds.req_message_rate = 10;   // [Hz]
    rds.start_stop = 1;         // "1" to start sending

    mavlink_msg_request_data_stream_encode(sys_id, comp_id, msg, &rds);
}

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
                          uint32_t msgid, uint32_t itv)
{
    mavlink_command_long_t cmd;
    cmd.target_system = target_sys_id;
    cmd.target_component = target_comp_id;
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd.confirmation = 0;
    cmd.param1 = (float)msgid;
    cmd.param2 = (float)itv;   // [us]
    cmd.param3 = 0;
    cmd.param4 = 0;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;     // Flight-stack default

    mavlink_msg_command_long_encode(sys_id, comp_id, msg, &cmd);
}

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
void mav_cmd_arm_disarm(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint8_t flag)
{
    mavlink_command_long_t cmd;
    cmd.target_system = target_sys_id;
    cmd.target_component = target_comp_id;
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    cmd.confirmation = 0;
    cmd.param1 = (float)flag;
    cmd.param2 = 0;
    // 0: arm-disarm unless prevented by safety checks (i.e. when landed)
    // 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)

    mavlink_msg_command_long_encode(sys_id, comp_id, msg, &cmd);
}

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
void mav_cmd_toggle_offboard(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t target_comp_id, bool flag)
{
    mavlink_command_long_t cmd = {0};
    cmd.target_system       = target_sys_id;
    cmd.target_component    = target_comp_id;
    cmd.command             = MAV_CMD_NAV_GUIDED_ENABLE;
    cmd.confirmation        = true;
    cmd.param1              = (float)flag;

    mavlink_msg_command_long_encode(sys_id, comp_id, msg, &cmd);
}

/**
 * @brief 将px4设置为offboard模式
 * 
 * @param msg 
 * @param sys_id 
 * @param comp_id 
 * @param target_sys_id 
 * @param target_comp_id 
 */
void mav_cmd_set_offboard_mode(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
//    uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | 
//                    MAV_MODE_FLAG_SAFETY_ARMED |
//                    MAV_MODE_FLAG_HIL_ENABLED;
	uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t custom_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
    uint8_t custom_sub_mode = 0;

    mavlink_command_long_t cmd = {0};
    cmd.target_system = target_sys_id;
    cmd.target_component = target_comp_id;
    cmd.command = MAV_CMD_DO_SET_MODE;
    cmd.confirmation = true;

    cmd.param1 = (float)mode;
    cmd.param2 = (float)custom_mode;
    cmd.param3 = (float)custom_sub_mode;

    mavlink_msg_command_long_encode(sys_id, comp_id, msg, &cmd);
}

void mav_cmd_set_autotakeoff_mode(mavlink_message_t *msg, uint8_t sys_id, uint8_t comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
	uint8_t mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t custom_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
    uint8_t custom_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;

    mavlink_command_long_t cmd = {0};
    cmd.target_system = target_sys_id;
    cmd.target_component = target_comp_id;
    cmd.command = MAV_CMD_DO_SET_MODE;
    cmd.confirmation = true;

    cmd.param1 = (float)mode;
    cmd.param2 = (float)custom_mode;
    cmd.param3 = (float)custom_sub_mode;

    mavlink_msg_command_long_encode(sys_id, comp_id, msg, &cmd);
}



/* ------------------------------ 位置 ------------------------------ */
/**
 * @brief 设置位置
 * 
 * @param x 
 * @param y 
 * @param z 
 * @param sp 
 */
void mav_set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t *sp)
{
    sp->type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

    sp->coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp->x   = x;
    sp->y   = y;
    sp->z   = z;
}

/**
 * @brief 设置速度
 * 
 * @param vx 
 * @param vy 
 * @param vz 
 * @param sp 
 */
void mav_set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t *sp)
{
    sp->type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp->coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp->vx  = vx;
    sp->vy  = vy;
    sp->vz  = vz;
}

/**
 * @brief 设置转角
 * 
 * @param yaw 
 * @param sp 
 */
void mav_set_yaw(float yaw, mavlink_set_position_target_local_ned_t *sp)
{
    sp->type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

    sp->yaw  = yaw;
}

/**
 * @brief 设置角速度
 * 
 * @param yaw_rate 
 * @param sp 
 */
void mav_set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t *sp)
{
    sp->type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

    sp->yaw_rate  = yaw_rate;
}

/* ------------------------- 姿态 -----------------------------*/

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
void mav_set_attitude_q_and_bodyrate(float q1, float q2, float q3, float q4, float roll_rate, float pitch_rate, float yaw_rate,  mavlink_set_attitude_target_t *sa)
{
    sa->type_mask = MAVLINK_MSG_SET_ATTITUDE_TARGET_Q_AND_BODYRATE;
    sa->q[0] = q1;
    sa->q[1] = q2;
    sa->q[2] = q3;
    sa->q[3] = q4;
    sa->body_roll_rate = roll_rate;
    sa->body_pitch_rate = pitch_rate;
    sa->body_yaw_rate = yaw_rate;
}
// MAVLINK_MSG_ID_SET_ATTITUDE_TARGET


/* ------------------------ 自身坐标系下 ------------------------ */

/**
 * @brief 设置速度
 * 
 * @param vx 
 * @param vy 
 * @param vz 
 * @param sp 
 */
void mav_set_velocity_body(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t *sp)
{
    sp->type_mask =
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp->coordinate_frame = MAV_FRAME_BODY_NED;

    sp->vx  = vx;
    sp->vy  = vy;
    sp->vz  = vz;
}

/**
 * @brief 设置偏航角
 * 
 * @param yaw 
 * @param sp 
 */
void mav_set_yaw_body(float yaw, mavlink_set_position_target_local_ned_t *sp)
{
    sp->type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

    sp->yaw  = yaw;
}

/**
 * @brief 设置偏航角速度
 * 
 * @param yaw_rate 
 * @param sp 
 */
void mav_set_yaw_rate_body(float yaw_rate, mavlink_set_position_target_local_ned_t *sp)
{
    sp->type_mask &=
        MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

    sp->yaw_rate  = yaw_rate;
}
