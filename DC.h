#ifndef DC_H
#define DC_H

#include "ti_msp_dl_config.h"
#include "math.h" 

#define PI 3.1415926535897932382643383279 //圆周率
#define DD 65//轮直径：65mm
#define GMR_CPR_1 14000 //GMR编码电机转一圈对应的脉冲数(软件一倍频)
#define GMR_CPR_2 28000 //GMR编码电机转一圈对应的脉冲数(软件二倍频)
#define GMR_CPR_4 56000 //GMR编码电机转一圈对应的脉冲数(软件四倍频)
#define DC_RATIO_MAX 0.8 //最大电机转速比例
#define DC_ARR 1000//电机pwmARR
void DC_Init(void);//电机初始化
void SetVelocityL(float ratio);//设置左轮速度（左后）
void SetVelocityR(float ratio);//设置右轮速度（右后）
void SetVelocity(float ratioL,float ratioR);//设置左右轮速度（自动取决于移动模式）
void UpdateVelocity(void);//获取速度,计算一个测速周期内的车速度和整体速度，更新移动距离
void DC_Start(int8_t dir);//电机启动(前进/后退/左转/右转)，无距离限制
void DC_Forward(float Distance,uint8_t inf);//电机前进（可选择移动距离）
void DC_Backward(float Distance,uint8_t inf);//电机后退（可选择移动距离）
void DC_Turn(int8_t dir,uint16_t angle); //非堵塞式旋转一定角度停下
void DC_Stop(void);//电机停止
void DC_Start_Full(void);//2GPIO驱动电机全速驱动
void DC_Init_Full(void);//2GPIO驱动电机初始化
void DC_Stop_Full(void);//2GPIO驱动电机停止驱动
void UpdateVelocityDouble(void);

#endif /* "DC.h" */