#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <stdint.h>

void set_pwm_left(float duty, uint8_t dir);
void set_pwm_right(float duty, uint8_t dir);
void stop_motors(void);

#endif