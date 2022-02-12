#ifndef MAVCOMMON_H_
#define MAVCOMMON_H_

#include "stm32f10x.h"
#include <stdint.h>
#include "common/mavlink.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define SYS_ID              1
#define COMP_ID             195
#define TARGET_SYS_ID       1
#define TARGET_COMP_ID      MAV_COMP_ID_AUTOPILOT1 // 0 for debug, MAV_COMP_ID_AUTOPILOT1 for normal use

/* ===================================================================
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 * 
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 * =================================================================== */
																// bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION		0xDF8     	// 0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     	0xDC7		// 0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 	0xC3F		// 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        	0xE3F		// 0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    	0x9FF		// 0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     	0x5FF		// 0b0000010111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION_Z	0xDFB     	// 0b0000110111111011

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POS_VEL		0xDC0     	// 0b0000110111000000

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF      0x1000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND         0x2000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER       0x3000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE         0x4000

//  MAVLINK_MSG_ID_SET_ATTITUDE_TARGET
																// bit number  0987654321
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_Q_AND_BODYRATE		    0X60     	// 0b01100000



/**
 * @brief 时间戳结构体
 * 
 */
typedef struct _Time_Stamps
{
    uint32_t heartbeat;
    uint32_t sys_status;
    uint32_t local_position_ned;
    uint32_t global_position_int;
    uint32_t position_target_local_ned;
    uint32_t highres_imu;
    uint32_t attitude;
    uint32_t optical_flow;
    uint32_t optical_flow_rad;
    uint32_t altitude;

}Time_Stamps;

/**
 * @brief mavlink消息结构体
 * 
 */
typedef struct _Mavlink_Messages{

    int sysid;
    int compid;

    mavlink_heartbeat_t heartbeat;                                      // Heartbeat
    mavlink_sys_status_t sys_status;                                    // System Status
    mavlink_local_position_ned_t local_position_ned;                    // Local Position
    mavlink_global_position_int_t global_position_int;
    mavlink_position_target_local_ned_t position_target_local_ned;      // Local Position Target
    mavlink_highres_imu_t highres_imu;                                  // HiRes IMU
    mavlink_attitude_t attitude;                                        // Attitude
    mavlink_optical_flow_t optical_flow;                                 // Optical Flow
    mavlink_optical_flow_rad_t optical_flow_rad;                        // Optical Flow Rad
    Time_Stamps time_stamps;                                            // Time Stamps
    mavlink_altitude_t altitude;                                        // 高度

}Mavlink_Messages;

typedef struct _Autopilot_Interface
{
    /* id of companion computer */
    uint8_t sys_id;
    uint8_t comp_id;

    /* id of autopilot */
    uint8_t target_sys_id;
    uint8_t target_comp_id;

    /* storage of messages from autopilot */
    Mavlink_Messages current_messages;

    /* setpoint to be sent to the autopilot */
    mavlink_set_position_target_local_ned_t current_setpoint;
    mavlink_set_position_target_local_ned_t initial_position;

    /* target height */
    float targetHeight;
}Autopilot_Interface;

enum PX4_CUSTOM_MAIN_MODE {
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
    PX4_CUSTOM_MAIN_MODE_ALTCTL,
    PX4_CUSTOM_MAIN_MODE_POSCTL,
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_ACRO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_MAIN_MODE_STABILIZED,
    PX4_CUSTOM_MAIN_MODE_RATTITUDE
};

enum PX4_CUSTOM_SUB_MODE_AUTO {
    PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
    PX4_CUSTOM_SUB_MODE_AUTO_RTL,
    PX4_CUSTOM_SUB_MODE_AUTO_LAND,
    PX4_CUSTOM_SUB_MODE_AUTO_RTGS,
    PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET
};

union px4_custom_mode {
    struct {
        uint16_t reserved;
        uint8_t main_mode;
        uint8_t sub_mode;
    };
    uint32_t data;
    float data_float;
};

#endif

