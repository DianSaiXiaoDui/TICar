#include "PTZcontrol.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "Velocity_PID.h"

const float middleRadX = 0.75 * M_PI;
const float middleRadY = 0.5 * M_PI;
float dis = 50.0;
float targetRads[2] = {0.75 * M_PI, 0.5 * M_PI}; // 初始化目标角度（弧度）

// uint16_t clip(int maxn, int minn, int val)
// {
//     val = val > maxn ? maxn : val;
//     val = val < minn ? minn : val;
//     return val;
// }

void PTZ_Init(void)
{
   SetDegree(90,90);
}

void calculateTargetRads(float x, float y)
{
    targetRads[0] = middleRadX - atan(x/dis);
    targetRads[1] = middleRadY + atan(y/dis);
}

float RadToDegree(float rad)
{
    return rad * (180.0 / M_PI);
}



uint16_t DegreeToCCRX(float Degree)
{
    int val = Degree * 1425 / 270 + 212.5;
    clip(&val, 250, 1250);
    return (uint16_t)(val);
}

uint16_t DegreeToCCRY(float Degree)
{
    int val = Degree * 675 / 180 + 400;
    clip(&val, 250, 1250);
    return (uint16_t)(val);
}

void SetDegree(float xDegree,float yDegree)
{
    DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, DegreeToCCRX(xDegree), GPIO_PWM_PTZ_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, DegreeToCCRY(yDegree), GPIO_PWM_PTZ_C1_IDX); 
}

void draw(float x, float y)
{
    CalculateTargetRads(x, y);
    float xDegree = RadToDegree(targetRads[0]);
    float yDegree = RadToDegree(targetRads[1]);
    SetDegree(xDegree, yDegree);
}