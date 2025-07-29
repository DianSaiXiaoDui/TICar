#ifndef __CCR_PID_H
#define __CCR_PID_H

#include <stdint.h>


typedef struct{
	    float Kp;               // 比例系数
	    float Ki;               // 积分系数
	    float Kd;               // 微分系数
	    float P;                // 比例项
	    float I;                // 积分项
	    float D;                // 微分项
	    float IThresh;          // 积分限幅
	    float CurrentError;     // 当前误差
	    float LastError;        // 上一误差
	    float ErrorThresh;      // 抗积分饱和临界误差
	    float ErrorInt;         // 累计误差
	    float Current;          // 当前横/纵坐标
	    float Target;           // 目标横/纵坐标
	    int32_t DeltaCCR;         // pid输出量：电机CCR改变值
	    int32_t OutputThreshH;    // pid输出限幅（上界）
	    int32_t OutputThreshL;    // pid输出限幅（下界）
	    uint32_t Reset;        	// 切换目标标志
}CCR_PID_Struct;

extern CCR_PID_Struct CCRX_PID;
extern CCR_PID_Struct CCRY_PID;

void CCRX_PID_Init();
void CCRY_PID_Init();

void CCRX_PID_Update(void);
void CCRY_PID_Update(void);

void CCRX_PID_Reset(void);
void CCRY_PID_Reset(void);

#endif /* __CCR_PID_H */
