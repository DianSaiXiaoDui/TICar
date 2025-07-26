#ifndef __CCD_H__
#define __CCD_H__

#include <stdint.h>

extern volatile uint8_t ADC_flag;
extern volatile uint16_t ccd_result[128];

void Dly_us(void);
void Linear_CCD_Flush(void);
void CCD_Read(void);

#endif