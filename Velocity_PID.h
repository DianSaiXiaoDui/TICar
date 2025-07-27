#ifndef INC_VELOCITY_PID_H_
#define INC_VELOCITY_PID_H_

#include "ti_msp_dl_config.h"

//左轮速度pid结构体
typedef struct{
	    double Kp;               // 左轮比例系数
	    double Ki;               // 左轮积分系数
	    double Kd;               // 左轮微分系数
	    double P;                // 左轮比例项
	    double I;                // 左轮积分项
	    double D;                // 左轮微分项
	    double IThresh;          // 左轮积分限幅
	    double Error0;           // 左轮当前速度误差
	    double Error1;           // 左轮上一速度误差
	    double ErrorThresh;      // 左轮抗积分饱和临界误差
	    double ErrorInt;         // 左轮累计速度误差
	    double CurVelocity;      // 左轮当前速度
	    double TargetVelocity;   // 左轮目标速度
	    double PwmDuty;          // 左轮pid输出量:左电机pwm占空比
	    double OutputThreshH;    // 左轮pid输出限幅（上界）
	    double OutputThreshL;    // 左轮pid输出限幅（下界）
	    uint8_t Reset;        //左轮切换目标速度标志
}  BL_Velocity_PID_Struct;

//右轮速度pid结构体
typedef struct{
	    double Kp;               // 右轮比例系数
	    double Ki;               // 右轮积分系数
	    double Kd;               // 右轮微分系数
	    double P;                // 右轮比例项
	    double I;                // 右轮积分项
	    double D;                // 右轮微分项
	    double IThresh;          // 右轮积分限幅
	    double Error0;           // 右轮当前速度误差
	    double Error1;           // 右轮上一速度误差
	    double ErrorInt;         // 右轮累计速度误差
	    double ErrorThresh;      // 右轮抗积分饱和临界误差
	    double CurVelocity;      // 右轮当前速度
	    double TargetVelocity;   // 右轮目标速度
	    double PwmDuty;          // 右轮pid输出量:右电机pwm占空比
	    double OutputThreshH;    // 右轮pid输出大小限幅（上界）
	    double OutputThreshL;    // 右轮pid输出大小限幅（下界）
	    uint8_t Reset;        //右轮切换目标速度标志
}  BR_Velocity_PID_Struct;


extern BL_Velocity_PID_Struct BL_Velocity_PID;
extern BR_Velocity_PID_Struct BR_Velocity_PID;

//速度pid初始化
void BL_Velocity_PID_Init(void);
void BR_Velocity_PID_Init(void);
void Velocity_PID_Init(void);

//设置速度pid目标速度
void Set_BL_TargetVelocity(double v);
void Set_BR_TargetVelocity(double v);
void Set_TargetVelocity(double vl,double vr);

//设置速度pid参数
void Set_BL_Kp(double kp);
void Set_BL_Ki(double ki);
void Set_BL_Kd(double kd);
void Set_BR_Kp(double kp);
void Set_BR_Ki(double ki);
void Set_BR_Kd(double kd);

//速度pid控制
void BL_Velocity_PID_Control(void);
void BR_Velocity_PID_Control(void);
void Velocity_PID_Control(void);

//速度pid更新左右轮转速
void Velocity_PID_Update(void);
//void Velocity_Update_A(void);

//速度pid复位
void Velocity_PID_Reset(void);
void BL_Velocity_PID_Reset(void);
void BR_Velocity_PID_Reset(void);


//获取当前目标速度
double Get_BL_TargetVelocity();
double Get_BR_TargetVelocity();

//限幅函数
void clip(double* val,double min,double max);

#endif /* INC_VELOCITY_PID_H_ */
