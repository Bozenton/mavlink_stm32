/**
 * @file laser.h
 * @author 小白哥 (you@domain.com)
 * @brief 通过继电器控制激光笔的开关，高电平开灯，低电平关灯
 * @version 0.1
 * @date 2021-11-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef LASER_H_
#define LASER_H_

#include "sys.h"

#define LASER   PGout(8)    // PG8

void initLaser(void);


#endif


