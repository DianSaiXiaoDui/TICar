#include "motor_control.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "motor_control.h"
#include "CCD.h"
#include "CCD_ex.h"
#include "drive.h"

#define MAX_I 100

uint16_t diff = 0;
float kp_s = 0.0f;

int error = 0;
int last_error = 0;
int error_div = 0;
int error_sum = 0;
uint8_t lost = 0;
float kp_f = 0.0f;
float ki_f = 0.0f;
float kd_f = 0.0f;
float pid_output = 0.0f;


void Drive_TurnInPlaceLeft(int speed)
{

}
void Drive_TurnInPlaceRight(int speed)
{

}
void Drive_Straight_PID(float base_pwm)
{
    set_pwm_left(base_pwm,1);
    set_pwm_right(base_pwm,1);

    while(1)
    {
        if(PID_flag == 1) // 每20ms置一次一
        {
            PID_flag = 0;
            diff = current_position_1 - current_position_2;
            float pwm_left = base_pwm - diff * kp_s;
            float pwm_right = base_pwm + diff * kp_s;
            set_pwm_left(pwm_left, 1);
            set_pwm_right(pwm_right, 1);
        }
        // 补充退出条件
    }
}
void Drive_LineFollow(float base_pwm)
{
    set_pwm_left(base_pwm,1);
    set_pwm_right(base_pwm,1);

    while(1)
    {
        if(PID_flag == 1) // 每20ms置一次一
        {
            PID_flag = 0;

            CCD_Read();
            CCD_MeanFilter();
            CCD_FindBlackLine(); // 更新TargetIdx

            if(TargetIdx != 404)
                error = TargetIdx - 63;
            else
                lost++;
            if(lost > 10) 
            {
                stop_motors();
                break;
            }
            else
            {
                lost = 0;
                error_sum += error;
                error_div = error - last_error;
                pid_output = kp_f*error + ki_f*error_sum + kd_f*error_div;
                last_error = error;

                if (error_sum > MAX_I) error_sum = MAX_I;
                if (error_sum < -MAX_I) error_sum = -MAX_I;

                float pwm_left = base_pwm - pid_output; // 确定符号
                float pwm_right = base_pwm + pid_output;
                set_pwm_left(pwm_left, 1); // 确定左右前后
                set_pwm_right(pwm_right, 1);
            }
            //补充退出条件
        }
    }
    error_sum = 0;
}