#include "CCR_PID.h"
#include <stdint.h>
#include <math.h>

CCR_PID_Struct CCRX_PID;
CCR_PID_Struct CCRY_PID;


void CCRX_PID_Init()
{
	CCRX_PID.Kp                     = 0.1;               // 比例系数
	CCRX_PID.Ki                     = 0;                 // 积分系数
	CCRX_PID.Kd                     = 0.25;                 // 微分系数
	CCRX_PID.P                      = 0;                 // 比例项
	CCRX_PID.I                      = 0;                 // 积分项
	CCRX_PID.D                      = 0;                 // 微分项
	CCRX_PID.IThresh                = 0.2;               // 积分限幅
	CCRX_PID.CurrentError           = 0;                 // 当前误差
	CCRX_PID.LastError              = 0;                 // 上一误差
	CCRX_PID.ErrorThresh            = 15;                // 抗积分饱和临界误差
	CCRX_PID.ErrorInt               = 0;                 // 累计误差
	CCRX_PID.Current                = 0;                 // 当前横/纵坐标
	CCRX_PID.Target                 = 0;                 // 目标横/纵坐标
	CCRX_PID.DeltaCCR               = 0;                 // pid输出量：电机CCR改变值
	CCRX_PID.OutputThreshH          = 100;                // pid输出限幅（上界）
	CCRX_PID.OutputThreshL          = -100;               // pid输出限幅（下界）
	CCRX_PID.Reset                  = 1;        	     // 切换目标标志
}

void CCRY_PID_Init()
{
    CCRY_PID.Kp                     = 0.1;               // 比例系数
    CCRY_PID.Ki                     = 0;                 // 积分系数
    CCRY_PID.Kd                     = 0;                 // 微分系数
    CCRY_PID.P                      = 0;                 // 比例项
    CCRY_PID.I                      = 0;                 // 积分项
    CCRY_PID.D                      = 0;                 // 微分项
    CCRY_PID.IThresh                = 0.2;               // 积分限幅
    CCRY_PID.CurrentError           = 0;                 // 当前误差
    CCRY_PID.LastError              = 0;                 // 上一误差
    CCRY_PID.ErrorThresh            = 15;                // 抗积分饱和临界误差
    CCRY_PID.ErrorInt               = 0;                 // 累计误差
    CCRY_PID.Current                = 0;                 // 当前横/纵坐标
    CCRY_PID.Target                 = 0;                 // 目标横/纵坐标
    CCRY_PID.DeltaCCR               = 0;                 // pid输出量：电机CCR改变值
    CCRY_PID.OutputThreshH          = 100;                // pid输出限幅（上界）
    CCRY_PID.OutputThreshL          = -100;               // pid输出限幅（下界）
    CCRY_PID.Reset                  = 1;                 // 切换目标标志
}



//全量式pid ,更新pid输出
void CCRX_PID_Update(void)
{
	CCRX_PID.CurrentError = CCRX_PID.Target - CCRX_PID.Current;
	if (CCRX_PID.Reset)
	{
		CCRX_PID.LastError = CCRX_PID.CurrentError;
		CCRX_PID.Reset = 0;
	}
	CCRX_PID.P = CCRX_PID.Kp * CCRX_PID.CurrentError;	// 计算P项
	if (fabs(CCRX_PID.CurrentError) > CCRX_PID.ErrorThresh)	//计算I项
	{
		CCRX_PID.ErrorInt = 0;
	}
	else
	{
		CCRX_PID.ErrorInt += CCRX_PID.CurrentError;  // 累计误差积分
	}
	CCRX_PID.I = CCRX_PID.ErrorInt * CCRX_PID.Ki;  // I项=累计误差积分*积分系数
	// 积分限幅
    CCRX_PID.I = (CCRX_PID.I > CCRX_PID.IThresh)? CCRX_PID.IThresh:CCRX_PID.I;
	CCRX_PID.I = (CCRX_PID.I < -CCRX_PID.IThresh)? -CCRX_PID.IThresh:CCRX_PID.I;

	CCRX_PID.D = CCRX_PID.Kd * (CCRX_PID.CurrentError - CCRX_PID.LastError);	// 计算D项

	CCRX_PID.DeltaCCR = (int32_t)(CCRX_PID.P + CCRX_PID.I + CCRX_PID.D);// PID输出=P项+I项+D项
	// 输出限幅
	CCRX_PID.DeltaCCR = (CCRX_PID.DeltaCCR > CCRX_PID.OutputThreshH)? CCRX_PID.OutputThreshH:CCRX_PID.DeltaCCR;
	CCRX_PID.DeltaCCR = (CCRX_PID.DeltaCCR < CCRX_PID.OutputThreshL)? CCRX_PID.OutputThreshL:CCRX_PID.DeltaCCR;

	CCRX_PID.LastError = CCRX_PID.CurrentError;// 更新上一次误差
}

void CCRY_PID_Update(void)
{
	CCRY_PID.CurrentError = CCRY_PID.Target - CCRY_PID.Current;
	if (CCRY_PID.Reset)
	{
		CCRY_PID.LastError = CCRY_PID.CurrentError;
		CCRY_PID.Reset = 0;
	}
	CCRY_PID.P = CCRY_PID.Kp * CCRY_PID.CurrentError;	// 计算P项
	if (fabs(CCRY_PID.CurrentError) > CCRY_PID.ErrorThresh)	//计算I项
	{
		CCRY_PID.ErrorInt = 0;
	}
	else
	{
		CCRY_PID.ErrorInt += CCRY_PID.CurrentError;  // 累计误差积分
	}
	CCRY_PID.I = CCRY_PID.ErrorInt * CCRY_PID.Ki;  // I项=累计误差积分*积分系数
	// 积分限幅
    CCRY_PID.I = (CCRY_PID.I > CCRY_PID.IThresh)? CCRY_PID.IThresh:CCRY_PID.I;
	CCRY_PID.I = (CCRY_PID.I < -CCRY_PID.IThresh)? -CCRY_PID.IThresh:CCRY_PID.I;

	CCRY_PID.D = CCRY_PID.Kd * (CCRY_PID.CurrentError - CCRY_PID.LastError);	// 计算D项

	CCRY_PID.DeltaCCR = (int32_t)(CCRY_PID.P + CCRY_PID.I + CCRY_PID.D);// PID输出=P项+I项+D项
	// 输出限幅
	CCRY_PID.DeltaCCR = (CCRY_PID.DeltaCCR > CCRY_PID.OutputThreshH)? CCRY_PID.OutputThreshH:CCRY_PID.DeltaCCR;
	CCRY_PID.DeltaCCR = (CCRY_PID.DeltaCCR < CCRY_PID.OutputThreshL)? CCRY_PID.OutputThreshL:CCRY_PID.DeltaCCR;

	CCRY_PID.LastError = CCRY_PID.CurrentError;// 更新上一次误差
}


void CCRX_PID_Reset(void)
{
	CCRX_PID.ErrorInt = 0;
	CCRX_PID.Reset = 1;
}

void CCRY_PID_Reset(void)
{
	CCRY_PID.ErrorInt = 0;
	CCRY_PID.Reset = 1;
}


