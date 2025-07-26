#include "DC.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

extern enum DcDriveMode{
  FRONT,
  BACK
} ;


extern volatile uint8_t MoveFlag;
extern volatile uint8_t EnableDistanceFlag;
extern volatile uint8_t StraightStopFlag;
extern volatile uint8_t DCStopFlag;
extern volatile uint8_t TurnFlag;
extern volatile uint16_t TurnPeriod;

extern volatile uint8_t T_Velocity;
extern volatile double V_L;
extern volatile double V_R;
extern volatile int8_t Dir_L;//左轮转动方向
extern volatile int8_t Dir_R;//右轮转动方向
extern volatile int8_t Dir;//前进方向
extern volatile double TotalDistance;//总移动距离
extern volatile double TargetDistance;//目标移动距离

extern volatile uint8_t EnableEncoderFlag;
extern volatile uint16_t  EncoderA_Cnt;
extern volatile uint16_t  EncoderB_Cnt;
extern  enum DcDriveMode DriveMode;

//电机初始化
void DC_Init(void)
{
  //DL_Timer_setCaptureCompareValue(PWM_DC_INST,2,GPIO_PWM_DC_C0_IDX);//PWM(AIN1)
  //DL_Timer_setCaptureCompareValue(PWM_DC_INST,2,GPIO_PWM_DC_C1_IDX);//PWM(AIN3)
  //DL_GPIO_clearPins(GPIO_DC_PORT,GPIO_DC_AIN0_PIN|GPIO_DC_AIN2_PIN);   //Ain0=Ain2=0
  EnableEncoderFlag=1; //使能编码器计数
  EncoderA_Cnt=0;  //编码器计数值置0	
  EncoderB_Cnt=0;   //编码器计数值置0			
  Dir=0;//标识车前进/后退方向（不含旋转）
  Dir_L=0;
  Dir_R=0;     
  MoveFlag=0;
}

//左轮设置速度
void SetVelocityL(float ratio)
{
   //ratio:最大电机转速占比（范围0~1）
   uint8_t L_dir;//左后轮转动方向
    /*转速限制*/
   if(ratio>DC_RATIO_MAX)
      ratio=DC_RATIO_MAX;
   else if(ratio<-DC_RATIO_MAX)
      ratio=-DC_RATIO_MAX;

   /*方向判断*/
   if(ratio>0)
       L_dir=1;//正转
   else if(ratio<0)
       L_dir=0;//反转

    /*正转驱动*/
    if(L_dir>0)
    {
       DL_GPIO_setPins(GPIO_DC_PORT,GPIO_DC_AIN0_PIN);  //AIN0=1
		   DL_Timer_setCaptureCompareValue(PWM_DC_INST,(uint16_t)(1-ratio)*DC_ARR,GPIO_PWM_DC_C0_IDX );//PWM(AIN1)
    }

    /*反转驱动*/
    else
    {
        DL_GPIO_clearPins(GPIO_DC_PORT,GPIO_DC_AIN0_PIN);  //AIN0=0
        DL_Timer_setCaptureCompareValue(PWM_DC_INST,(uint16_t)(-ratio)*DC_ARR,GPIO_PWM_DC_C0_IDX);//PWM(AIN1)
    }


}

//右轮设置速度（默认右后）
void SetVelocityR(float ratio)
{
   //ratio:最大电机转速占比（范围0~1）
   uint8_t R_dir;//左后轮转动方向
    /*转速限制*/
   if(ratio>DC_RATIO_MAX)
      ratio=DC_RATIO_MAX;
   else if(ratio<-DC_RATIO_MAX)
      ratio=-DC_RATIO_MAX;

   /*方向判断*/
   if(ratio>0)
       R_dir=1;//正转
   else if(ratio<0)
       R_dir=0;//反转

    /*正转驱动*/
    if(R_dir>0)
    {
       DL_GPIO_setPins(GPIO_DC_PORT,GPIO_DC_AIN2_PIN);  //AIN2=1
	    	DL_Timer_setCaptureCompareValue(PWM_DC_INST,(uint16_t)(1-ratio)*DC_ARR,GPIO_PWM_DC_C1_IDX );//PWM(AIN3)
    }

    /*反转驱动*/
    else
    {
        DL_GPIO_clearPins(GPIO_DC_PORT,GPIO_DC_AIN2_PIN);  //AIN2=0
        DL_Timer_setCaptureCompareValue(PWM_DC_INST,(uint16_t)(-ratio)*DC_ARR,GPIO_PWM_DC_C1_IDX);//PWM(AIN3)、
    }

}


//两轮设置速度
void SetVelocity(float ratioL,float ratioR)
{
	//若电机未启动，启动电机
	if(MoveFlag==0)
	{
		
      //开启定时器(开启PWM输出)
      //DL_TimerG_startCounter(PWM_DC_INST );
		 //两路pwm占空比置0
       DL_Timer_setCaptureCompareValue(PWM_DC_INST,1,GPIO_PWM_DC_C0_IDX);//PWM(AIN1)
	  	  DL_Timer_setCaptureCompareValue(PWM_DC_INST,1,GPIO_PWM_DC_C1_IDX);//PWM(AIN3)
		  //编码器计数值置0
		  EncoderA_Cnt=0;
		  EncoderB_Cnt=0;
		  MoveFlag=1;//设置移动标志
	}
	if(MoveFlag==1)
	{
       if(DriveMode==BACK)
	   {
		  	SetVelocityL(ratioL);
		   	SetVelocityR(ratioR);
	   }
	   else if(DriveMode==FRONT)
	   {
        SetVelocityL(-ratioR);
		  	SetVelocityR(-ratioL);
	   }
	}
}

//获取速度,计算一个测速周期内的车速度和整体速度，更新移动距离
void UpdateVelocity(void)
{
   double V_Temp;//交互中介速度
   double V_C;//小车平均速度
   double circle=PI*DD/10;//轮子周长（cm）
   double dpp=circle/GMR_CPR_2;  //每个脉冲对应的距离（cm）[软件二倍频]
   int16_t delta_distance_L,delta_distance_R;//一个测速周期内计数器累加值
   double delta_distance_L_cm,delta_distance_R_cm;//将计数器累加值转化为cm

    //获取一个测速周期内左后轮/右后轮计数器值
   delta_distance_L =  EncoderA_Cnt;
   delta_distance_R =  EncoderB_Cnt;

    //将计时器清零，开始新一轮计数
   EncoderA_Cnt=0;
   EncoderB_Cnt=0;

   //将计数器值转化为移动距离（cm）
   delta_distance_L_cm=delta_distance_L*dpp;
   delta_distance_R_cm=delta_distance_R*dpp;

   //计算一个测速周期内的速度
   V_L=delta_distance_L_cm/(T_Velocity/1000.0);
   V_R=delta_distance_R_cm/(T_Velocity/1000.0);
  
   //前驱模式，交换轮速
   if(DriveMode==BACK)
   {
     V_L=delta_distance_L_cm/(T_Velocity/1000.0);
     V_R=delta_distance_R_cm/(T_Velocity/1000.0);
   }
   else if(DriveMode==FRONT)
   {
     V_L=-(delta_distance_R_cm/(T_Velocity/1000.0));
     V_R=-(delta_distance_L_cm/(T_Velocity/1000.0));
   }

   //计算小车平均速度
    V_C=(V_L+V_R)/2;

   //计算一个测速周期内小车移动距离,更新总距离
    if(MoveFlag==1 && EnableDistanceFlag==1)
    {
    	TotalDistance+=(fabs(V_C)*(T_Velocity/1000.0));//更新总距离
    	if(TotalDistance>=TargetDistance)//达到目标距离后，直行停止
    	{
    		StraightStopFlag=1; 
		   	EnableDistanceFlag=0;
    	}
    }

}

void DC_Start(int8_t dir)
{
	if(dir==0) //直走
	{
	  //Set_TargetVelocity(20,20);//设置pid目标速度20
	  SetVelocity(0.1,0.1);//启动电机
    Dir_L=1;
    Dir_R=1;
    Dir=1;//前进
	}
	else if(dir==1)//右转
	{
	  //Set_TargetVelocity(20,-20);//设置pid目标速度20
	  SetVelocity(0.1,-0.1);//启动电机
    Dir_L=1;
    Dir_R=-1;
    Dir=110;
	}
	else if(dir==-1)//左转
	{
	  //Set_TargetVelocity(-20,20);//设置pid目标速度20
	  SetVelocity(-0.1,0.1);//启动电机
    Dir_L=-1;
    Dir_R=1;
    Dir=101;
	}
	else if(dir==10) //后退
	{
	  //Set_TargetVelocity(-20,20);//设置pid目标速度20
	  SetVelocity(-0.1,-0.1);//启动电机
    Dir_L=-1;
    Dir_R=-1;
    Dir=-1;    
	}
}

//停止电机
void DC_Stop(void)
{
	//关停1号电机
  DL_Timer_setCaptureCompareValue(PWM_DC_INST,2,GPIO_PWM_DC_C0_IDX);//PWM(AIN1)
	DL_GPIO_clearPins(GPIO_DC_PORT,GPIO_DC_AIN0_PIN);    
	//关停2号电机
	DL_Timer_setCaptureCompareValue(PWM_DC_INST,2,GPIO_PWM_DC_C1_IDX);//PWM(AIN3)	
	DL_GPIO_clearPins(GPIO_DC_PORT,GPIO_DC_AIN0_PIN); 
	//Velocity_PID_Reset();//速度pid复位
	MoveFlag=0;
  Dir_L=0;
  Dir_R=0;
  Dir=0;   
}

//非堵塞式旋转一定角度停下,配合中断计数器和停止标志使用
void DC_Turn(int8_t dir,uint16_t angle)
{
	//顺时针转90度
	if(dir>0 && angle==90)
	{
    DC_Start(1);
		TurnFlag=1;
		TurnPeriod=650;
	}
	//逆时针转90度
	else if(dir<0 && angle==90)
	{
    DC_Start(-1);   
		TurnFlag=1;
		TurnPeriod=650;
	}
	//掉头
	if(angle==180)
	{
    DC_Start(1);
		TurnFlag=1;
		TurnPeriod=1100;
	}
}

//前进有限/无限距离
void DC_Forward(float Distance,uint8_t inf)
{
  DC_Start(0);
	if(inf==0)//走一定距离后停下
	{
		EnableDistanceFlag=1;
		TotalDistance=0;
		TargetDistance=Distance;
	}
}

void DC_Backward(float Distance,uint8_t inf)//后退
{
    
	DC_Start(10);
	if(inf==0)//走一定距离后停下
	{
		  EnableDistanceFlag=1;
		  TotalDistance=0;
		  TargetDistance=Distance;
	}
}




