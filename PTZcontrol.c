#include "PTZcontrol.h"
#include "CCR_PID.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "Velocity_PID.h"

const float middleRadX = 0.5 * M_PI;
const float middleRadY = 0.5 * M_PI;
float dis = 100.0; // cm
float targetRads[2] = {0.75 * M_PI, 0.5 * M_PI}; // 初始化目标角度（弧度）
float px, py;
extern int32_t CCR_PIDflag;
void PTZ_Init(void)
{
   SetDegree(90,90);
}


struct Circle c = {.cx = 1, .cy = 1, .r = 10};

struct PointFollower 
{
    float x, dis;
}follower;

void SetFollowerX(float x)
{
    follower.x = x;
}

void SetFollowerDis(float dis)
{
    follower.dis = dis;
}

void SetCenter(float x, float y)
{
    c.cx = x;
    c.cy = y;
}

void SetRadius(float _r)
{
    c.r = _r;
}


void CalculateTargetRads(float x, float y)
{
    targetRads[0] = middleRadX - atan(x/follower.dis);
    targetRads[1] = middleRadY + atan(y/follower.dis);
}

float RadToDegree(float rad)
{
    return rad * (180.0 / M_PI);
}

int DegreeToCCRX(float Degree)
{
    double val = Degree * 1000 / 180 + 250;
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
    x -= follower.x;
    CalculateTargetRads(x, y);
    float xDegree = RadToDegree(targetRads[0]);
    float yDegree = RadToDegree(targetRads[1]);
    SetDegree(xDegree, yDegree);
}


void DrawCircle()
{
    for(float i = 0; i < 2 * M_PI; i += 0.02)
    {
        
        px = c.cx + c.r * cos(i);
        py = c.cx + c.r * sin(i);
        // printf("x: %f, y: %f \n", px, py);
        FollowPoint(px, py);
        delay_cycles(80000);
    }
}

void StaticShooting(uint16_t clockwise)
{
    //云台旋转找靶
    uint8_t Step = 25; //转动CCR步长 
    float TimeInterval = 0.2;//单位:s
    DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, 750, GPIO_PWM_PTZ_C1_IDX);//将舵机转到180度处
    //DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, 750, GPIO_PWM_PTZ_C0_IDX);
    delay_cycles(TimeInterval*32000000);
    CCR_PIDflag = 0;
    if (clockwise)
    {
        for (int i = 750; i >= 250 ; i -= Step)
        {
        if(CCR_PIDflag) return; // 收到命令, 退出静止找点
        DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, i,GPIO_PWM_PTZ_C1_IDX);
        float per = 1.0 * i / 1250;
        // int count = 0;
        // while (count < TimeInterval*32000000)
        // {
        //     count++;
        //     if (CCR_PIDflag)
        //         return;
        // }
        // printf("percentage: %d \n", i);
        delay_cycles(TimeInterval*32000000);
        }
    }
    else
    {
        for (int i = 750; i <= 1250; i += Step)
        {
            if(CCR_PIDflag) return; // 收到命令, 退出静止找点
            DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, i,GPIO_PWM_PTZ_C1_IDX);
            float per = 1.0 * i / 1250;
            // int count = 0;
            // while (count < TimeInterval*32000000)
            // {
            //     count++;
            //     if (CCR_PIDflag)
            //         return;
            // }
            // printf("percentage: %d \n", i);
            delay_cycles(TimeInterval*32000000);
        }
    }
    
    // DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, 1500,GPIO_PWM_PTZ_C1_IDX);
    // delay_cycles(TimeInterval*32000000);
    // DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, 250,GPIO_PWM_PTZ_C1_IDX);
    // delay_cycles(TimeInterval*32000000);
}
