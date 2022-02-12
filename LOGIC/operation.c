#include "operation.h"

/**
 * @brief 起飞操作
 * 
 * @param api 接口指针
 */
void opTakeoff(Autopilot_Interface * api)
{
    api->initial_position.x = api->current_messages.local_position_ned.x;
    api->initial_position.y = api->current_messages.local_position_ned.y;
    api->initial_position.z = api->current_messages.local_position_ned.z;
    api->initial_position.vx = api->current_messages.local_position_ned.vx;
    api->initial_position.vy = api->current_messages.local_position_ned.vy;
    api->initial_position.vz = api->current_messages.local_position_ned.vz;
    api->initial_position.yaw = api->current_messages.attitude.yaw;
    api->initial_position.yaw_rate = api->current_messages.attitude.yawspeed;

    float takeoffHeight = 0.5;
    mavlink_set_position_target_local_ned_t sp;
    mavlink_set_position_target_local_ned_t ip = api->initial_position;
    mavSetPosition( ip.x ,       // [m]
			 	    ip.y ,       // [m]
				    ip.z - takeoffHeight , // [m]
				    &sp         );
    // sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION_Z;
    sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
    mavUpdateSetPoint(sp);
    mavWriteSetPoint();
}


/**
 * @brief 降落操作（以匀速下降）
 * 
 * @param api 
 */
void opLand(Autopilot_Interface * api)
{
    mavlink_local_position_ned_t cp = api->current_messages.local_position_ned;
    mavlink_set_position_target_local_ned_t sp; 
    mavSetVelocity( 0.0       , // [m/s]
					0.0       , // [m/s]
					0.3       , // [m/s]
					&sp       );
    sp.x = cp.x;
    sp.y = cp.y;
    sp.type_mask ^= POSITION_TARGET_TYPEMASK_X_IGNORE;
    sp.type_mask ^= POSITION_TARGET_TYPEMASK_Y_IGNORE;
    sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;
    mavUpdateSetPoint(sp);
    mavWriteSetPoint();
}



