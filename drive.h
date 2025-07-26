#ifndef __DRIVE_H__
#define __DRIVE_H__

#include <stdint.h>

extern uint8_t PID_flag;
extern volatile uint16_t current_position_1; // 假设是左轮
extern volatile uint16_t current_position_2; // 假设是右轮

#endif