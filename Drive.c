
#include "Drive.h"
#include "PTZcontrol.h"
#include "CCR_PID.h"
#include "ti_msp_dl_config.h"
#include "ti/driverlib/dl_timer.h"
extern volatile uint8_t MoveFlag;
extern volatile uint8_t EnableDistanceFlag;
extern volatile uint8_t StraightStopFlag;
extern volatile uint8_t DCStopFlag;
extern volatile uint8_t TurnFlag;
extern volatile uint16_t TurnPeriod;

extern int32_t CCR_PIDflag;
extern float targetX;
extern float currentX;
extern float targetY;
extern float currentY;
extern int16_t currentCCRX;
extern int16_t currentCCRY;
extern int16_t maxCCRX;
extern int16_t minCCRX;
extern int16_t maxCCRY;
extern int16_t minCCRY;
extern uint8_t K230_receive_completed;

//串口通信
extern uint8_t K230_Uart2_RxBuffer[128];
extern uint8_t K230_receive_len;
extern uint8_t K230_receive_completed;
extern uint8_t K230_receive;


extern volatile double V_L;
extern volatile double V_R;
extern volatile int8_t Dir_L;//左轮转动方向
extern volatile int8_t Dir_R;//右轮转动方向
extern volatile int8_t Dir;//前进方向
extern volatile double TotalDistance;//总移动距离
extern volatile double TargetDistance;//目标移动距离
extern volatile uint8_t T_Velocity_Update;//速度更新周期
extern volatile uint8_t Velocity_PID_UpdateFlag;//速度pid更新标志
extern volatile uint8_t Velocity_UpdateFlag;//速度更新标志
extern volatile uint8_t EnableEncoderFlag;
extern volatile int32_t  EncoderA_Cnt;
extern volatile int32_t  EncoderB_Cnt;
extern volatile int8_t DriveMode;
extern volatile double V_Base;
extern volatile uint8_t TrackLineMode;
extern volatile double V_Ratio_Base_Straight;//基准直行速度比例
extern volatile double V_Ratio_Base_Turn;//基准转弯速度比例
extern volatile double K_Offset_FF;//左轮相对右轮的速度补偿系数(前驱前进)
extern volatile double K_Offset_FB;//左轮相对右轮的速度补偿系数(前驱后退)

extern volatile uint8_t CCD_UpdateFlag;
extern volatile int16_t CCD_TargetIdx;

//直走一段距离停下（编码器测距闭环、角度pid巡线）
void MoveForward(uint32_t forwardDistance)
{
	if(MoveFlag==0)
	{
		DC_Start(0);//启动电机
	}
	TargetDistance=forwardDistance;
	EnableDistanceFlag=1;
	while(!StraightStopFlag)
	{
		if(K230_receive_completed)
		{
		ParseAndExecuteCommand((char*)K230_Uart2_RxBuffer);
		K230_receive_completed = 0;
		}
		if(CCR_PIDflag)
		{
			CCRX_PID_Update();
			CCRY_PID_Update();
			currentCCRX =  DL_Timer_getCaptureCompareValue(PWM_PTZ_INST,GPIO_PWM_PTZ_C1_IDX);
			currentCCRY =  DL_Timer_getCaptureCompareValue(PWM_PTZ_INST,GPIO_PWM_PTZ_C0_IDX);
			currentCCRX += CCRX_PID.DeltaCCR;
			currentCCRY -= CCRY_PID.DeltaCCR;
			currentCCRX = (currentCCRX < maxCCRX)? currentCCRX : maxCCRX;
			currentCCRX = (currentCCRX > minCCRX)? currentCCRX : minCCRX;
			currentCCRY = (currentCCRY < maxCCRY)? currentCCRY : maxCCRY;
			currentCCRY = (currentCCRY > minCCRY)? currentCCRY : minCCRY;
			DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, currentCCRX, GPIO_PWM_PTZ_C1_IDX);
			DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, currentCCRY, GPIO_PWM_PTZ_C0_IDX); 
			CCR_PIDflag = 0;
		}


		if( Velocity_UpdateFlag==1)//更新测速
		{
			UpdateVelocity();
			Velocity_UpdateFlag=0;
		}

		if(CCD_UpdateFlag==1 && TrackLineMode==1) //巡线模式1，角度pid更新
		{
			CCD_Read();
			CCD_DataProcess();
			if(CCD_TargetIdx!=-1)
			{ 
				Angle_PID_SetCurX(CCD_TargetIdx);
				Angle_PID_Update();
			}
			else{
				Set_TargetVelocity(V_Base,V_Base);
			}
			CCD_UpdateFlag=0;
		}

		if(CCD_UpdateFlag==1 && TrackLineMode==2) //巡线模式2，角度pid更新
		{
			CCD_Read();
			CCD_DataProcess();
			if(CCD_TargetIdx!=-1)
			{ 
				Angle_PID_SetCurX(CCD_TargetIdx);
				Angle_PID_Update();
			}
			CCD_UpdateFlag=0;
		}

		//速度pid更新（巡线模式1）
		if(MoveFlag==1 && Velocity_PID_UpdateFlag==1 && TrackLineMode==1)
		{
			Velocity_PID_UpdateFlag=0;
			Velocity_PID_Update();//速度PID控制
		}
		
	}
	TotalDistance=0;
	EnableDistanceFlag=0;
    StraightStopFlag=0;
	DC_Stop();
}

//堵塞式开环旋转一定角度停下
void openLoopTurning(int8_t clockwise,uint16_t angle)
{
    uint32_t TurnCnt=0;
    uint32_t TurnPeriod=0;
	TrackLineMode=1;//暂时切为模式1，启用速度pid转向
	if(clockwise > 0)//右转
	{
		if(angle == 90)
		{
            DC_Start(1);
			TurnPeriod=1000000;
		}
		if(angle == 180)
		{
			DC_Start(1);
			TurnPeriod=1800000;
		}
	}
	else
	{
		if(angle == 90)
		{
			DC_Start(-1);
			TurnPeriod=1000000;
		}
		if(angle == 180)
		{
			DC_Start(-1);
			TurnPeriod=1800000;
		}
	}
	while(TurnCnt++<=TurnPeriod)
	{
       if( Velocity_UpdateFlag==1)//更新测速
       {
        UpdateVelocity();
        Velocity_UpdateFlag=0;
       }
		 //速度pid更新
	    if(MoveFlag==1 && Velocity_PID_UpdateFlag==1)
	    {
	    	Velocity_PID_UpdateFlag=0;
	       Velocity_PID_Update();//速度PID控制
	    }
	}
	TrackLineMode=2;//切回模式2
	DC_Stop();
}

//闭环转向
void closeLoopTurning(int8_t clockwise,uint16_t angle)
{
    uint32_t TurnCnt=0;
    uint32_t TurnPeriod=0;
	TrackLineMode=1;//暂时切为模式1，启用速度pid转向
	if(clockwise > 0)//右转
	{
		if(angle == 90)
		{
            DC_Start(1);
			TurnPeriod=950000;
		}
		if(angle == 180)
		{
			DC_Start(1);
			TurnPeriod=1800000;
		}
	}
	else
	{
		if(angle == 90)
		{
			DC_Start(-1);
			TurnPeriod=950000;
		}
		if(angle == 180)
		{
			DC_Start(-1);
			TurnPeriod=1800000;
		}
	}
	while(TurnCnt++<=TurnPeriod)
	{ 
       if( Velocity_UpdateFlag==1)//更新测速
       {
        UpdateVelocity();
        Velocity_UpdateFlag=0;
       }
		 //速度pid更新
	    if(MoveFlag==1 && Velocity_PID_UpdateFlag==1)
	    {
	    	Velocity_PID_UpdateFlag=0;
	       Velocity_PID_Update();//速度PID控制
	    }
	}
	TrackLineMode=2;//切回模式2
	DC_Stop();
}

/*
沿正方形轨道运动：
AAAAAAAAAAAAAA




2------------3
|            |
|            |
|            |
1------------4 
*/

//行进过程中切换前驱/后驱模式
void NewMoveAlongSquare(uint8_t start_idx,uint16_t period)
{
 uint16_t cnt=0;
 while(cnt++<period)
 {
  //1->2
  MoveForward(100);//直行1m
  Delay_ms(500);
  openLoopTurning(-1,90);//左转90度
  SwitchDriveMode(-1);//切换为后驱模式
  Delay_ms(500);
  //2->3
  MoveForward(100);//直行1m
  Delay_ms(500);
  openLoopTurning(1,90);//右转90度
  Delay_ms(500);
//保持后驱模式
  //3->4
  MoveForward(100);//直行1m
  Delay_ms(500);
  openLoopTurning(-1,90);//左转90度
  SwitchDriveMode(1);//切换为前驱模式
  Delay_ms(500);
  //4->1
  MoveForward(100);//直行1m
  Delay_ms(500);
  openLoopTurning(1,90);//右转90度
  Delay_ms(500);
  //保持前驱模式
 }

}

//不切换驱动模式，一直绕圈
void MoveAlongSquare(uint8_t start_idx,uint16_t period)
{
 uint16_t cnt=0;
 while(cnt++<period*4)
 {
   MoveForward(100);//直行1m
   Delay_ms(500);
   openLoopTurning(1,90);//右转90度
   Delay_ms(500);
 }
}


uint32_t Delay_ms(uint32_t n)
{
    /*Debug1: uint32_t cycles=CPUCLK_FREQ*n/1000.0; 
    问题① 如果没有括号，CPUCLK_FREQ*n会溢出 
    问题② 1000->1000.0 保证浮点数运算，否则直接整除
    */
    uint32_t cycles=CPUCLK_FREQ*(n/1000.0);
    delay_cycles(cycles);
    return cycles;
}