#ifndef __CCD_EX_H__
#define __CCD_EX_H__

#include <stdint.h>

extern volatile uint16_t ccd_filtered[128];
extern volatile uint16_t TargetIdx;

void CCD_MeanFilter(void);
void CCD_FindBlackLine(void);

#endif