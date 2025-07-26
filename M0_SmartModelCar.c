#include "DC.h"
#include "CCD.h"
#include "CCD_ex.h"
#include "motor_control.h"
#include "ti/driverlib/dl_adc12.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_timerg.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"


volatile uint8_t Time_Count_1mS_Index = 0x0;
/*
volatile uint16_t EncoderA_Cnt = 32500;
volatile uint16_t EncoderB_Cnt = 32500;
volatile uint16_t current_position_l = 0;
volatile uint16_t current_position_r = 0;
*/
float motor_speed_l = 0;
float motor_speed_r = 0;
uint8_t motor_direct_l = 0;
uint8_t motor_direct_f = 0;
volatile uint32_t la_level;
volatile uint32_t lb_level;
volatile uint32_t ra_level;
volatile uint32_t rb_level;

volatile uint8_t ADC_flag = 0;
volatile uint16_t ccd_result[128];
volatile uint16_t ccd_filtered[128];
volatile uint16_t TargetIdx = 404; // 404表示未找到黑线

uint8_t PID_flag = 0;
uint8_t PID_TIM = 0;

float duty1 = 0.0;

//电机
volatile uint8_t MoveFlag;
volatile uint8_t EnableDistanceFlag;
volatile uint8_t StraightStopFlag;
volatile uint8_t MotorStopFlag;
volatile uint8_t TurnFlag;
volatile uint32_t TurnPeriod;
volatile uint32_t Cnt_1ms;//1ms计数器
volatile uint8_t T_Velocity=20;//测速周期
volatile double V_L;
volatile double V_R;
volatile int8_t Dir;//标识车前进/后退方向（不含旋转）
volatile int8_t Dir_L;
volatile int8_t Dir_R;
volatile double TotalDistance;//总移动距离
volatile double TargetDistance;//目标移动距离

//编码器
volatile uint8_t EnableEncoderFlag=0;
volatile uint8_t UpdateEncoderFlag=0;
volatile int32_t EncoderA_Cnt=0;//编码器A计数
volatile int32_t EncoderB_Cnt=0;//编码器B计数


//驱动模式
enum DcDriveMode{
  FRONT,
  BACK
} ;

volatile enum DcDriveMode DriveMode=FRONT;

uint32_t Delay_ms(uint32_t n)
{
    uint32_t cycles=CPUCLK_FREQ*(n/1000.0);
    delay_cycles(cycles);
    return cycles;
}

int main(void) {
  SYSCFG_DL_init();
  DC_Init();
   DL_GPIO_clearInterruptStatus(
      GPIOB, GPIO_ENCODER_PIN_1A_PIN | GPIO_ENCODER_PIN_1B_PIN |
                 GPIO_ENCODER_PIN_2A_PIN | GPIO_ENCODER_PIN_2B_PIN);
  NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
  NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);
  NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

  DL_TimerA_startCounter(TIMER_0_INST);
  DL_TimerG_startCounter(PWM_DC_INST);

  //DC_Start(0);
  // 正转时， 左边置低， 右边占空比越小速度越快， 零为满速
  // 反转时，左边置高，右边占空比越小速度越快
 
  
  SetVelocity(0.1,0.1);

 // delay_cycles(32000000);
  //delay_cycles(32000000);
  //DC_Stop();
  // set_pwm_left(20, 1);
  // set_pwm_right(20, 1);

  while (1) {

    /*CCD_Read();
    CCD_MeanFilter();
    CCD_FindBlackLine();*/
    if(UpdateEncoderFlag==1)
    {
      UpdateVelocity();
      UpdateEncoderFlag=0;
    }
    //delay_cycles(32000000);
  }
}

void GROUP1_IRQHandler(void) {
  uint32_t iidx = DL_GPIO_getPendingInterrupt(GPIO_ENCODER_PORT);
  la_level = DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_1A_PIN);
  lb_level = DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_1B_PIN);
  ra_level = DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_2A_PIN);
  rb_level = DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_2B_PIN);

  switch (iidx) {
  case GPIO_ENCODER_PIN_1A_IIDX:        // 左轮 A 相引脚产生中断
    if (la_level != 0 && lb_level == 0) // A上升沿 B为低电平 正转
      EncoderA_Cnt++;
    else if (la_level != 0 && lb_level != 0) // A上升沿 B为高电平 反转
      EncoderA_Cnt--;
    else if (la_level == 0 && lb_level == 0) // A下降沿 B为低电平 反转
      EncoderA_Cnt--;
    else if (la_level == 0 && lb_level != 0) // A下降沿 B为高电平 正转
      EncoderA_Cnt++;
    break;

  case GPIO_ENCODER_PIN_1B_IIDX:        // 左轮 B 相引脚产生中断
    if (la_level != 0 && lb_level == 0) // B下降沿 A为高电平 反转
      EncoderA_Cnt--;
    else if (la_level != 0 && lb_level != 0) // B上升沿 A为高电平 正转
      EncoderA_Cnt++;
    else if (la_level == 0 && lb_level == 0) // B下降沿 A为低电平 正转
      EncoderA_Cnt++;
    else if (la_level == 0 && lb_level != 0) // B上升沿 A为低电平 反转
      EncoderA_Cnt--;
    break;

  case GPIO_ENCODER_PIN_2A_IIDX:        // 右轮 A 相引脚产生中断
    if (ra_level != 0 && rb_level == 0) // A上升沿 B为低电平 正转
      EncoderB_Cnt++;
    else if (ra_level != 0 && rb_level != 0) // A上升沿 B为高电平 反转
      EncoderB_Cnt--;
    else if (ra_level == 0 && rb_level == 0) // A下降沿 B为低电平 反转
      EncoderB_Cnt--;
    else if (ra_level == 0 && rb_level != 0) // A下降沿 B为高电平 正转
      EncoderB_Cnt++;
    break;

  case GPIO_ENCODER_PIN_2B_IIDX:        // 右轮 B 相引脚产生中断
    if (ra_level != 0 && rb_level == 0) // B下降沿 A为高电平 反转
      EncoderB_Cnt--;
    else if (ra_level != 0 && rb_level != 0) // B上升沿 A为高电平 正转
      EncoderB_Cnt++;
    else if (ra_level == 0 && rb_level == 0) // B下降沿 A为低电平 正转
      EncoderB_Cnt++;
    else if (ra_level == 0 && rb_level != 0) // B上升沿 A为低电平 反转
      EncoderB_Cnt--;
    break;
  }
  DL_GPIO_clearInterruptStatus(
      GPIOB, GPIO_ENCODER_PIN_1A_PIN | GPIO_ENCODER_PIN_1B_PIN |
                 GPIO_ENCODER_PIN_2A_PIN | GPIO_ENCODER_PIN_2B_PIN);
}

void TIMER_0_INST_IRQHandler() {

  Cnt_1ms++;
  if(Cnt_1ms%T_Velocity==0)
  {
     UpdateEncoderFlag=1;
     Cnt_1ms=0;
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