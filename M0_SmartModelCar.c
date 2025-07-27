#include "DC.h"
#include "Velocity_PID.h"
#include "Angle_PID.h"
#include "CCD.h"
#include "CCD_ex.h"
#include "ti/driverlib/dl_adc12.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_timerg.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "ti/driverlib/dl_timer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define V_MAX 80
#define V_MIN 10

volatile uint8_t Time_Count_1mS_Index = 0x0;
/*
volatile uint16_t EncoderA_Cnt = 32500;
volatile uint16_t EncoderB_Cnt = 32500;
volatile uint16_t current_position_l = 0;
volatile uint16_t current_position_r = 0;
*/

//CCD
volatile uint8_t ADC_flag = 0;
volatile uint16_t CCD_ADV_Origin[128];//采集到的原始CCD像素序列
volatile uint16_t CCD_ADV_Filtered[128];//滤波后的CCD像素序列
volatile int16_t DxMax=0;//最大像素差分值
int16_t DxMin=0;//最小像素差分值
int16_t dX[128];//像素差分序列
uint16_t MaxIdx=0;//最大像素跳变点坐标
uint16_t MinIdx=0;//最小像素跳变点坐标
uint16_t CCD_TargetIdx;//目标中心点坐标
uint8_t CCD_UpdateFlag=0; //CCD更新标志
uint8_t T_Angle=100;//CCD调控周期（巡线）
volatile uint32_t Angle_PID_Cnt;//巡线差速pid计数器
extern Angle_PID_Struct Angle_PID;

//电机
volatile uint8_t MoveFlag;
volatile uint8_t EnableDistanceFlag;
volatile uint8_t StraightStopFlag;
volatile uint8_t MotorStopFlag;
volatile uint8_t TurnFlag;
volatile uint32_t TurnPeriod;
volatile uint32_t Velocity_PID_Cnt;//速度pid计数器
volatile uint8_t T_Velocity=20;//测速周期
volatile double V_L;
volatile double V_R;
volatile double NewVelocity;//测试时修改目标速度
volatile int8_t Dir;//标识车前进/后退方向（不含旋转）
volatile int8_t Dir_L;
volatile int8_t Dir_R;
volatile double TotalDistance;//总移动距离
volatile double TargetDistance;//目标移动距离
volatile double V_Base=30;//基准启动速度

//编码器
volatile uint8_t EnableEncoderFlag=0;
volatile uint8_t UpdateEncoderFlag=0;
volatile int32_t EncoderA_Cnt=0;//编码器A计数
volatile int32_t EncoderB_Cnt=0;//编码器B计数

//速度pid
extern BL_Velocity_PID_Struct BL_Velocity_PID;
extern BR_Velocity_PID_Struct BR_Velocity_PID;
volatile uint8_t Velocity_PID_UpdateFlag=0;
volatile char Velocity_Str[30];//串口查看速度波形 

//串口屏
// 串口屏通讯命令
volatile uint8_t StartSTR[3] = {0xff, 0xff, 0xff};
volatile uint8_t EndSTR[3] = {0xff, 0xff, 0xff};
volatile uint8_t HeaderTxBuffer1[] = "cls BLACK";      // 清屏命令
volatile uint8_t HeaderTxBuffer2[] = "page cube_aigc"; // 跳转页面命令
volatile uint8_t HeaderTxBuffer3[] = "n0.val=";        // 变量赋�?�命�????????????????????
volatile uint8_t HeaderTxBuffer4[20];                  // white line
volatile uint8_t HeaderTxBuffer5[] = "add 1,2,";       // real time data
volatile uint8_t HeaderTxBuffer6[20];                  // red line
volatile uint8_t HeaderTxBuffer7[20];                  // green line

// 接收串口屏信息
volatile uint8_t Touch_pannel_Uart0_RxBuffer[128];
volatile uint8_t Touch_pannel_receive_len = 0;
volatile uint8_t Touch_pannel_receive_completed = 0;
volatile uint8_t Touch_pannel_receive = 0;
volatile uint8_t Touch_pannel_data_receive_start = 0;

//驱动模式
volatile int8_t DriveMode=1;//前驱（-1：后驱）
void UART_SCREEN_TransmitString(const char* str);
uint32_t Delay_ms(uint32_t n)
{
    uint32_t cycles=CPUCLK_FREQ*(n/1000.0);
    delay_cycles(cycles);
    return cycles;
}

int main(void) {
  SYSCFG_DL_init();
  DC_Init();
  Velocity_PID_Init();//速度pid初始化
  NVIC_EnableIRQ(COMPARE_0_INST_INT_IRQN);
  NVIC_EnableIRQ(COMPARE_1_INST_INT_IRQN);
  NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
  //NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);
  NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
  NVIC_EnableIRQ(UART_SCREEN_INST_INT_IRQN);

  DL_TimerG_startCounter(COMPARE_0_INST);
  DL_TimerG_startCounter(COMPARE_1_INST);
  DL_TimerA_startCounter(TIMER_0_INST);
  DL_TimerG_startCounter(PWM_DC_INST);
  DL_TimerA_startCounter(PWM_PTZ_INST);


  // DC_Start(0);
  // 正转时， 左边置低， 右边占空比越小速度越快， 零为满速
  // 反转时， 左边置高， 右边占空比越小速度越快
 


  // for (int i = 250; i <= 1250; i += 25)
  // {
  //   DL_Timer_setCaptureCompareValue(PWM_PTZ_INST, i,GPIO_PWM_PTZ_C1_IDX);
  //   float per = 1.0 * i / 1250;
  //   printf("percentage: %d \n", i);
  //   delay_cycles(16000000);
  // }
  // while(1)
  //   DL_Timer_setCaptureCompareValue(PWM_PTZ_INST,(uint16_t)(740),GPIO_PWM_PTZ_C0_IDX );//PWM
  // delay_cycles(64000000);
  // DL_Timer_setCaptureCompareValue(PWM_PTZ_INST,(uint16_t)(800),GPIO_PWM_PTZ_C0_IDX );//PWM
  // delay_cycles(32000000);
  // DL_Timer_setCaptureCompareValue(PWM_PTZ_INST,(uint16_t)(400),GPIO_PWM_PTZ_C0_IDX );//PWM
  // delay_cycles(32000000);

  //delay_cycles(32000000);
  //DC_Stop();
  // set_pwm_left(20, 1);
  // set_pwm_right(20, 1);
  while (1) {


    //串口屏
        if (Touch_pannel_receive_completed == 1)
    {
        switch (Touch_pannel_Uart0_RxBuffer[1])
        {
        // 车前进
        case 0x11:
            DC_Start(0);
            Touch_pannel_Uart0_RxBuffer[1] = 0x0;
            break;    
        // 车后退    
        case 0x12:
            DC_Start(10);           
            Touch_pannel_Uart0_RxBuffer[1] = 0x0;
            break;
        // 车停止
        case 0x13:
            DC_Stop();
            Touch_pannel_Uart0_RxBuffer[1] = 0x0;
            break;
        // 调整车速
        case 0x14:
            
            //float TargetSpeed=V_MIN+(V_MAX-V_MIN)*Touch_pannel_Uart0_RxBuffer[2]/100.0;
				    //Set_TargetVelocity(TargetSpeed,TargetSpeed);
            V_Base=V_MIN+(V_MAX-V_MIN)*Touch_pannel_Uart0_RxBuffer[2]/100.0;
            Touch_pannel_Uart0_RxBuffer[1] = 0x0;
            break;
        // 车左转90度
        case 0x17:
            DC_Start(-1);
            Touch_pannel_Uart0_RxBuffer[1] = 0x0;
            break;
        // 车右转90度
        case 0x18:
            DC_Start(1);
            Touch_pannel_Uart0_RxBuffer[1] = 0x0;
            break;
        // 车掉头
        case 0x19:
            //Motor_Turn(-1, 180);
            // openLoopTurning(1, 180);
            Touch_pannel_Uart0_RxBuffer[1] = 0x0;
            break;            
        default:
            break;
        }
        Touch_pannel_receive_completed = 0;
    }

    //速度pid更新
    if(MoveFlag==1 && Velocity_PID_UpdateFlag==1)
    {
      UpdateVelocity();
      Velocity_PID_Update();
      Velocity_PID_UpdateFlag=0;
    }

    if(CCD_UpdateFlag==1)
    {
      CCD_Read();
      CCD_DataProcess();
     // Angle_PID_SetCurX(TargetIdx);
     // Angle_PID_Update();
      CCD_UpdateFlag=0;
    }

  }
}


void COMPARE_0_INST_IRQHandler()
{

  switch (DL_TimerG_getPendingInterrupt(COMPARE_0_INST)) 
  {
    case DL_TIMERG_IIDX_LOAD:
      if (!DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_1B_PIN)) 
      {
      // 反转
          EncoderA_Cnt++;
      } 
      else 
      {
      // 正转
         EncoderA_Cnt--;
      }
      break;
    default:
      break;
    }
}

void COMPARE_1_INST_IRQHandler()
{

  switch (DL_TimerG_getPendingInterrupt(COMPARE_1_INST)) 
  {
    case DL_TIMERG_IIDX_LOAD:
      if (!DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_2B_PIN)) 
      {
      // 反转
          EncoderB_Cnt++;
      } 
      else 
      {
      // 正转
         EncoderB_Cnt--;
      }
      break;
    default:
      break;
    }
}




void TIMER_0_INST_IRQHandler() {

  if(MoveFlag==1)
  {
    Velocity_PID_Cnt++;
     Angle_PID_Cnt++;

    if(Velocity_PID_Cnt % T_Velocity==0)
    {
      Velocity_PID_UpdateFlag=1;
      Velocity_PID_Cnt=0;
    }
    if(Angle_PID_Cnt & T_Angle==0)
    {
      Angle_PID_Cnt =0;
      CCD_UpdateFlag=1;
    }
  }

 
  DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);
}

void ADC12_0_INST_IRQHandler() {
  switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
  case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
    ADC_flag = 1;
    break;
  default:
    break;
  }
  DL_ADC12_clearInterruptStatus(ADC12_0_INST,
                                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
  DL_ADC12_enableConversions(ADC12_0_INST);
}

//串口屏接收处理命令
void UART_SCREEN_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_SCREEN_INST)) {
       case DL_UART_MAIN_IIDX_RX:
        /*串口屏命令接收*/
        Touch_pannel_receive = DL_UART_Main_receiveData(UART_SCREEN_INST); // 接收一个字节
        if (Touch_pannel_receive == 0x5A)                                  // 收到命令帧首
        {
            Touch_pannel_data_receive_start = 1;
        }
        if (Touch_pannel_data_receive_start) // 将字节放到接收数组中
        {
            Touch_pannel_Uart0_RxBuffer[Touch_pannel_receive_len] = Touch_pannel_receive;
            Touch_pannel_receive_len++;
            if (Touch_pannel_receive == 0xA5) // 收到帧尾
            {
                Touch_pannel_data_receive_start = 0; // 等待下一次命令接收
                Touch_pannel_receive_len = 0;        // 重置字节索引
                Touch_pannel_receive_completed = 1;  // 完成一次完整命令接收
            }
        }
           break;
        default: 
            break;
    }
}

/*串口发送字符串*/ 
void UART_SCREEN_TransmitString(const char* str)
{
   while(*str!='\0')
   {
      DL_UART_transmitDataBlocking(UART_SCREEN_INST , (uint8_t)*str);
      str++;
   }
}