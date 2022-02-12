/**
 * @file mavStartOpticalFlow.c
 * @author 小白哥 (you@domain.com)
 * @brief 光流不能自动启动，通过发送指令启动它
 * @version 0.1
 * @date 2021-10-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "mavStartOpticalFlow.h"

/**
 * @brief 开启光流
 * 
 * @param msg 
 */
void mav_start_optical_flow(mavlink_message_t *msg)
{
    char cmd[] = "px4flow start -X\n";
    mavlink_serial_control_t msc;
    msc.device = SERIAL_CONTROL_DEV_SHELL;
    msc.flags = SERIAL_CONTROL_FLAG_EXCLUSIVE | SERIAL_CONTROL_FLAG_RESPOND;
    msc.timeout = 0;
    msc.baudrate = 0;
    int idx=0;
    while(cmd[idx]!='\0')
    {
        msc.data[idx] = (uint8_t)cmd[idx];
        idx++;
    }
    msc.count = idx;
    mavlink_msg_serial_control_encode(SYS_ID, COMP_ID, msg, &msc);
}





