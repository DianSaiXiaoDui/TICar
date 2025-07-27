#include "CCD.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"

extern volatile uint8_t ADC_flag;
extern volatile uint16_t CCD_ADV_Origin[128];

void delay_us(uint8_t val)
{
    for(uint8_t i = 0; i<val; i++)
    {
        delay_cycles(32);
    }
}

void Linear_CCD_Flush(void)
{
  uint8_t index = 0;
  // TSL_SI=1;
  DL_GPIO_setPins(GPIO_CCD_SI_PORT, GPIO_CCD_SI_PIN);
  delay_us(1);
  // TSL_CLK=1;
  DL_GPIO_setPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
  delay_us(1);
  // TSL_SI=0;
  DL_GPIO_clearPins(GPIO_CCD_SI_PORT, GPIO_CCD_SI_PIN);
  delay_us(1);
  // TSL_CLK=0;
  DL_GPIO_clearPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
  delay_us(1);

  for (index = 0; index < 128; index++)
  {
    // TSL_CLK=1;
    DL_GPIO_setPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
    delay_us(2);
    // TSL_CLK=0;
    DL_GPIO_clearPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
    delay_us(2);
  }
}

void CCD_Read(void)
{
  uint8_t i = 0;
  Linear_CCD_Flush();
  delay_us(50);

  // TSL_SI=1;
  DL_GPIO_setPins(GPIO_CCD_SI_PORT, GPIO_CCD_SI_PIN);
  delay_us(1);
  // TSL_CLK=1;
  DL_GPIO_setPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
  delay_us(1);
  // TSL_SI=0;
  DL_GPIO_clearPins(GPIO_CCD_SI_PORT, GPIO_CCD_SI_PIN);
  delay_us(1);
  // TSL_CLK=0;
  DL_GPIO_clearPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
  delay_us(3);

  for (i = 0; i < 128; i++) // 128 DATA/LINE
  {

    DL_ADC12_startConversion(ADC12_0_INST);

    // // 等待 ADC 转换完成
    // while (ADC_flag != 1) {
    //     ; // 等待结束
    // }
    // ADC_flag = 0;

    CCD_ADV_Origin[i] = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);

    DL_GPIO_setPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
    delay_us(2);
    
    // TSL_CLK=0;
    DL_GPIO_clearPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
    delay_us(3);
    DL_ADC12_enableConversions(ADC12_0_INST);
  }
  DL_GPIO_setPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);// 129th pulse to terminate output of 128th pixel
  delay_us(2);
  DL_GPIO_clearPins(GPIO_CCD_CLK_PORT, GPIO_CCD_CLK_PIN);
}