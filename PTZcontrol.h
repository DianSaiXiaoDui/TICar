#ifndef __PTZCONTROL_H__
#define __PTZCONTROL_H__

#include <stdint.h>

extern float targetRads[]; // 目标弧度值
void calculateTargetRads(float x, float y);
float RadToDegree(float rad);
uint16_t DegreeToCCR(uint16_t MaxDegree,float Degree);
void SetDegree(float xDegree,float yDegree);


#endif //__PTZCONTROL_H__