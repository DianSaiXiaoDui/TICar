#ifndef __DRIVE_H__
#define __DRIVE_H__

#include "DC.h"
#include "Angle_PID.h"
#include "Velocity_PID.h"
#include "CCD.h"
#include "CCD_ex.h"
#include "ti_msp_dl_config.h"

void MoveForward(uint32_t forwardDistance);
void openLoopTurning(int8_t clockwise,uint16_t angle);
void NewMoveAlongSquare(uint8_t start_idx,uint16_t period);//沿正方形轨道运动
void MoveAlongSquare(uint8_t start_idx,uint16_t period);//沿正方形轨道运动
uint32_t Delay_ms(uint32_t n);
#endif