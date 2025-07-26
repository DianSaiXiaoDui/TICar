#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "motor_control.h"

#define PeriodCount 1000
// 根据实际情况改左右、前后、通道等
extern float duty;
// duty:0-100

void set_pwm_left(float duty, uint8_t dir) // 1前 0后
{
    if(duty < 1) duty = 1;
    else if(duty > 70) duty = 70;

    if(dir == 1) duty = 100-duty;

    uint32_t val = (uint32_t)PeriodCount*(100.0-duty)/100.0;

    if(dir == 0)
        DL_GPIO_clearPins(GPIOB, GPIO_DC_AIN0_PIN);
    else
        DL_GPIO_setPins(GPIOB, GPIO_DC_AIN0_PIN);

    DL_TimerG_setCaptureCompareValue(PWM_DC_INST, val, 0);
}

void set_pwm_right(float duty, uint8_t dir)
{
    if(duty < 1) duty = 1;
    else if(duty > 70) duty = 70;

    if(dir == 1) duty = 100-duty;

    uint32_t val = (uint32_t)PeriodCount*(100.0-duty)/100.0;

    if(dir == 0)
        DL_GPIO_clearPins(GPIOB, GPIO_DC_AIN2_PIN);
    else
        DL_GPIO_setPins(GPIOB, GPIO_DC_AIN2_PIN);

    DL_TimerG_setCaptureCompareValue(PWM_DC_INST, val, 1);
}
/*
void stop_motors(void)
{
    set_pwm_left(0, 1);
    set_pwm_right(0, 1);
}
*/
