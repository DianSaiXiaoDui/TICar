#include "PTZcontrol.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"


const float middleRadX = 0.75 * M_PI;
const float middleRadY = 0.5 * M_PI;
float dis = 50.0;
float targetRads[2] = {0.75 * M_PI, 0.5 * M_PI}; // 初始化目标角度（弧度）


void CalculateTargetRads(float x, float y)
{
    targetRads[0] = middleRadX - atan(x/dis);
    targetRads[1] = middleRadY + atan(y/dis);
}

float RadToDegree(float rad)
{
    return rad * (180.0 / M_PI);
}

uint16_t DegreeToCCR(uint16_t MaxDegree,float Degree)
{
   return (uint16_t)(Degree*1000/MaxDegree+250);
}

void SetDegree(float xDegree,float yDegree)
{
    DL_Timer_setCaptureCompareValue(PWM_PTZ_INST,1,GPIO_PWM_PTZ_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_PTZ_INST,1,GPIO_PWM_PTZ_C1_IDX); 
}