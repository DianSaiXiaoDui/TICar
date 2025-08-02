/*
 * Angle_PID.c
 *
 *  Created on: Jul 9, 2025
 */
#include "Angle_PID.h"
#include "DC.h"
#include "Velocity_PID.h"

extern volatile double V_Base;
extern volatile uint8_t MoveFlag;
extern volatile int8_t DriveMode;//驱动模式
extern volatile uint8_t TrackLineMode;
extern volatile double V_Ratio_Base_Straight;//基准直行速度比例
extern volatile double V_Ratio_Base_Turn;//基准转弯速度比例
extern volatile double K_Offset_FF;//左轮相对右轮的速度补偿系数(前驱前进)
extern volatile double K_Offset_FB;//左轮相对右轮的速度补偿系数(前驱后退)

Angle_PID_Struct Angle_PID;//转向pid结构体

//巡线模式1
void Angle_PID_Init(void)
{
  if(TrackLineMode==1)
  {
   //巡线小车
   Angle_PID.Kp=0.1;
   Angle_PID.Ki=0.0;
   Angle_PID.Kd=0.0;
   Angle_PID.P = 0;
   Angle_PID.I = 0;
   Angle_PID.D = 0;
   Angle_PID.Error0=0;
   Angle_PID.Error1=0;
   Angle_PID.ErrorThresh = 0;
   Angle_PID.ErrorInt=0;
   Angle_PID.IThresh=0;
   Angle_PID.CurX = 0;
   Angle_PID.TargetX = 63;
   Angle_PID.deltaVelocity = 0;
   Angle_PID.OutputThreshH=5;
   Angle_PID.OutputThreshL = 0;
   Angle_PID.Reset=0;
  }
  else if(TrackLineMode==2 && DriveMode==1)
  {
	Angle_PID.Kp=0.0008;
	Angle_PID.Kd=0.0003;
	Angle_PID.P = 0;
	Angle_PID.I = 0;
	Angle_PID.D = 0;
	Angle_PID.Error0=0;
	Angle_PID.Error1=0;
	Angle_PID.ErrorThresh = 0;
	Angle_PID.ErrorInt=0;
	Angle_PID.IThresh=0;
	Angle_PID.CurX = 0;
	Angle_PID.TargetX = 63;
	Angle_PID.deltaVelocityRatio = 0;
	Angle_PID.OutputThreshH=0.06;
	Angle_PID.OutputThreshL = 0;
	Angle_PID.Reset=0;
  }
  else if(TrackLineMode==2 && DriveMode==2)
  {
	Angle_PID.Kp=0.0008;
	Angle_PID.Kd=0.0003;
	Angle_PID.P = 0;
	Angle_PID.I = 0;
	Angle_PID.D = 0;
	Angle_PID.Error0=0;
	Angle_PID.Error1=0;
	Angle_PID.ErrorThresh = 0;
	Angle_PID.ErrorInt=0;
	Angle_PID.IThresh=0;
	Angle_PID.CurX = 0;
	Angle_PID.TargetX = 63;
	Angle_PID.deltaVelocityRatio = 0;
	Angle_PID.OutputThreshH=0.06;
	Angle_PID.OutputThreshL = 0;
	Angle_PID.Reset=0;
  }
}



/* Private PID functions ---------------------------------------------------------*/
//转向pid调控：输出和速度pid输出同数量级
void Angle_PID_Control()
{
	if(TrackLineMode==1)
	{
	    Angle_PID.Kp=0.06;
		Angle_PID.Kd=0.0;
		Angle_PID.OutputThreshH=5;
	}
	else if(TrackLineMode==2 && DriveMode==1)
	{
	    Angle_PID.Kp=0.0015;
		Angle_PID.OutputThreshH=0.06;
	}
	else if(TrackLineMode==2 && DriveMode==-1)
	{
	    Angle_PID.Kp=0.02;
		Angle_PID.OutputThreshH=0.06;
	}
	Angle_PID.Error0 = Angle_PID.TargetX - Angle_PID.CurX;//更新当前横向坐标误差

	//计算比例项
	Angle_PID.P=Angle_PID.Kp*Angle_PID.Error0;

	//计算积分项
	if(Angle_PID.Reset==1)//切换目标速度后，累计误差置0
	{
		Angle_PID.ErrorInt=0;
	}

	//当误差小于临界误差时，更新误差积分项
	if(MoveFlag == 1) {
		if(Angle_PID.Error0<Angle_PID.ErrorThresh)
			Angle_PID.ErrorInt += Angle_PID.Error0; // 小车运动时累加误差
	}

	Angle_PID.I=Angle_PID.Ki*Angle_PID.ErrorInt;

	// 积分限幅
	clip(&Angle_PID.I,0,Angle_PID.IThresh);

	//计算微分项
	if(Angle_PID.Reset==1)//切换目标速度后，第一次pid计算微分项为0，防止过冲
	{
	   Angle_PID.D=0;
	   Angle_PID.Reset=0;
	}
	else
	{
	   Angle_PID.D=Angle_PID.Kd*(Angle_PID.Error0-Angle_PID.Error1);
	}

	// 计算输出值 (PI控制)
	if(TrackLineMode==1)
	{
	  Angle_PID.deltaVelocity = Angle_PID.P + Angle_PID.I + Angle_PID.D;
	// 输出限幅
	  clip(&Angle_PID.deltaVelocity,Angle_PID.OutputThreshL,Angle_PID.OutputThreshH);
	}
    else if(TrackLineMode==2) 
	{
	   Angle_PID.deltaVelocityRatio = Angle_PID.P + Angle_PID.I + Angle_PID.D;
	// 输出限幅
	  clip(&Angle_PID.deltaVelocityRatio,Angle_PID.OutputThreshL,Angle_PID.OutputThreshH);
	}
	 


	// 更新上一误差
	Angle_PID.Error1 = Angle_PID.Error0;
}

void Angle_PID_Reset()
{
	Angle_PID.Reset=1;
}

void Angle_PID_SetCurX(uint16_t CurX)
{
	Angle_PID.CurX=CurX;
}


void Angle_PID_Update()
{
	float new_BL_Vel = 0;
    float new_BR_Vel = 0;
	float new_BL_Velocity_Ratio=0;
	float new_BR_Velocity_Ratio=0;
	if(TrackLineMode==1)
	{
		Angle_PID_Control();

		new_BL_Vel = V_Base - Angle_PID.deltaVelocity;
		new_BR_Vel = V_Base + Angle_PID.deltaVelocity;
		
	
		Set_TargetVelocity(new_BL_Vel, new_BR_Vel);
	}
	else if(TrackLineMode==2)
	{
        Angle_PID_Control();
		if(DriveMode==1)//前驱
		{
		   new_BL_Velocity_Ratio = V_Ratio_Base_Straight*K_Offset_FF - Angle_PID.deltaVelocityRatio;
		   new_BR_Velocity_Ratio = V_Ratio_Base_Straight + Angle_PID.deltaVelocityRatio;
		}
		else if(DriveMode==-1)//后驱
		{
           new_BL_Velocity_Ratio = (V_Ratio_Base_Straight + Angle_PID.deltaVelocityRatio)*0.7;
		   new_BR_Velocity_Ratio = V_Ratio_Base_Straight - Angle_PID.deltaVelocityRatio;
		}
		SetVelocity(new_BL_Velocity_Ratio, new_BR_Velocity_Ratio);
	}
}


/*
void Angle_PID_Update()
{
	Angle_PID_Control();
    float new_BL_ratio = V_Base - Angle_PID.deltaVelocity;
	float new_BR_ratio = V_Base + Angle_PID.deltaVelocity;
	Set_TargetVelocity(new_BL_Vel, new_RL_Vel);
}*/