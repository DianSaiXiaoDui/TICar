#include "PTZcontrol.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "Velocity_PID.h"

const float middleRadX = 0.75 * M_PI;
const float middleRadY = 0.5 * M_PI;
float dis = 100.0; // cm
float targetRads[2] = {0.75 * M_PI, 0.5 * M_PI}; // 初始化目标角度（弧度）
float px, py;



struct Circle c = {.cx = 1, .cy = 1, .r = 10};


void SetCenter(float x, float y, struct Circle cir)
{
    cir.cx = x;
    cir.cy = y;
}

void SetRadius(float _r, struct Circle cir)
{
    cir.r = _r;
}


void CalculateTargetRads(float x, float y)
{
    targetRads[0] = middleRadX - atan(x/dis);
    targetRads[1] = middleRadY + atan(y/dis);
}

float RadToDegree(float rad)
{
    return rad * (180.0 / M_PI);
}



int DegreeToCCRX(float Degree)
{
    double val = Degree * 1425 / 270 + 212.5;
    clip(&val, 250, 1250);
    return (int)(val);
}

int DegreeToCCRY(float Degree)
{
    double val = Degree * 675 / 180 + 400;
    clip(&val, 250, 1250);
    return (int)(val);
}

void SetDegree(float xDegree,float yDegree)
{
    DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, DegreeToCCRX(xDegree), GPIO_PWM_PTZ_C1_IDX);
    DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, DegreeToCCRY(yDegree), GPIO_PWM_PTZ_C0_IDX); 
    // printf("DegreeX: %d, DegreeY: %d \n", DegreeToCCRX(xDegree), DegreeToCCRY(yDegree));
}

void FollowPoint(float x, float y)
{
    CalculateTargetRads(x, y);
    float xDegree = RadToDegree(targetRads[0]);
    float yDegree = RadToDegree(targetRads[1]);
    SetDegree(xDegree, yDegree);
}


void DrawCircle()
{
    for(float i = 0; i < 2 * M_PI; i += 0.05)
    {
        
        px = c.cx + c.r * cos(i);
        py = c.cx + c.r * sin(i);
        // printf("x: %f, y: %f \n", px, py);
        FollowPoint(px, py);
        delay_cycles(160000);
    }
}