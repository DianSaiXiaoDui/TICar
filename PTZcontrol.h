#ifndef __PTZCONTROL_H__
#define __PTZCONTROL_H__

#include <stdint.h>
struct Circle
{
    float cx, cy;
    float r;
};
extern float targetRads[]; // 目标弧度值
void CalculateTargetRads(float x, float y);
float RadToDegree(float rad);
int DegreeToCCRX(float Degree);
int DegreeToCCRY(float Degree);

void SetDegree(float xDegree,float yDegree);
void FollowPoint(float x, float y);
void DrawCircle();

void SetCenter(float x, float y, struct Circle cir);


void SetRadius(float _r, struct Circle cir);

#endif //__PTZCONTROL_H__