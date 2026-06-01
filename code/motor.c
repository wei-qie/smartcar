/*
//motor.c//
//电机模块函数代码//
#include "zf_common_headfile.h"
#include"motor.h"
#define MAX_DUTY            (50 )                                               // 最大 MAX_DUTY% 占空比
#define DIR_R1              (P02_4)
#define PWM_R1              (ATOM0_CH5_P02_5)
#define DIR_L1              (P02_6)
#define PWM_L1              (ATOM0_CH7_P02_7)

#define DIR_R2              (P21_2)
#define PWM_R2              (ATOM0_CH1_P21_3)
#define DIR_L2              (P21_4)
#define PWM_L2              (ATOM0_CH3_P21_5)
void Motor_init()
{
    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R1, 17000, 0);                         // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L1, 17000, 0);                         // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_R2, 17000, 0);                         // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL);   // GPIO 初始化为输出 默认上拉输出高
    pwm_init(PWM_L2, 17000, 0);                         // PWM 通道初始化频率 17KHz 占空比初始为 0
}
void Motor_Control(int16_t left_output, int16_t right_output)
{
    // 限幅到±10000
    if (left_output > 10000) left_output = 10000;
    if (left_output < -10000) left_output = -10000;
    if (right_output > 10000) right_output = 10000;
    if (right_output < -10000) right_output = -10000;

    // 左电机控制
    if (left_output >= 0)
    {
        // 正转
        gpio_set_level(DIR_L1, GPIO_HIGH);
        gpio_set_level(DIR_L2, GPIO_LOW);
        pwm_set_duty(PWM_L1, left_output);
        pwm_set_duty(PWM_L2, 0);
    }
    else
    {
        // 反转
        gpio_set_level(DIR_L1, GPIO_LOW);
        gpio_set_level(DIR_L2, GPIO_HIGH);
        pwm_set_duty(PWM_L1, 0);
        pwm_set_duty(PWM_L2, -left_output);  // 取绝对值
    }

    // 右电机控制（同上）
    if (right_output >= 0)
    {
        gpio_set_level(DIR_R1, GPIO_HIGH);
        gpio_set_level(DIR_R2, GPIO_LOW);
        pwm_set_duty(PWM_R1, right_output);
        pwm_set_duty(PWM_R2, 0);
    }
    else
    {
        gpio_set_level(DIR_R1, GPIO_LOW);
        gpio_set_level(DIR_R2, GPIO_HIGH);
        pwm_set_duty(PWM_R1, 0);
        pwm_set_duty(PWM_R2, -right_output);
    }
}


*/
// motor.c – 精简版（仅正转，无 L2/R2 引脚）
#include "zf_common_headfile.h"
#include "motor.h"

#define DIR_R1   P02_4              // 右电机方向引脚
#define PWM_R1   ATOM0_CH5_P02_5    // 右电机PWM

#define DIR_L1   P02_6              // 左电机方向引脚
#define PWM_L1   ATOM0_CH7_P02_7    // 左电机PWM
// 风扇 PWM 引脚 (使用 P21.2 的 ATOM0 通道0)
#define FAN_PWM_PIN     ATOM0_CH0_P21_2
// PWM 频率 20kHz
#define FAN_PWM_FREQ    20000
void Motor_init(void)
{
   /* gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);*/
    pwm_init(PWM_R1, 17000, 0);

  /*  gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);*/
    pwm_init(PWM_L1, 17000, 0);
}

void Motor_Control(int16_t left_output, int16_t right_output)
{
    // 只允许正转（0~10000）
    if (left_output < 0)   left_output = 0;
    if (left_output > 10000) left_output = 10000;
    if (right_output < 0)  right_output = 0;
    if (right_output > 10000) right_output = 10000;

    /*// 左电机正转*/
   /* gpio_set_level(DIR_L1, GPIO_HIGH);*/
    pwm_set_duty(PWM_L1, left_output);

    // 右电机正转
  /* gpio_set_level(DIR_R1, GPIO_HIGH);*/
    pwm_set_duty(PWM_R1, right_output);
}
// fan.c

/**
 * @brief 初始化负压风扇 PWM
 * @note  使用 P21.2 的 ATOM0 通道 0，频率 20kHz
 */
void fan_init(void)
{
    pwm_init(FAN_PWM_PIN, FAN_PWM_FREQ, 0);   // 初始占空比 0，风扇不转
}

/**
 * @brief 设置风扇占空比
 * @param duty 占空比，范围 0 ~ 10000，对应 0% ~ 100%
 */
void fan_set_duty(int16_t duty)
{
    if (duty < 0)       duty = 0;
    if (duty > 10000)   duty = 10000;
    pwm_set_duty(FAN_PWM_PIN, duty);
}

// ==================== 灵活负压控制参数 ====================
#define FAN_DUTY_OFF        0       // 关闭
#define FAN_DUTY_STRAIGHT   2500    // 直道基础负压 25%
#define FAN_DUTY_CURVE      5500    // 弯道循线负压 55%
#define FAN_DUTY_CORNER     8500    // 直角转弯负压 85%
#define GYRO_MAX_FAN        300.0f  // 角速度达到此值时满负压

/**
 * @brief 灵活负压控制：根据转弯强度动态调节风扇
 * @param corner_active 是否在执行直角转弯
 * @param gyro_intensity 当前目标角速度绝对值（deg/s），反映转弯强度
 */
void fan_control_update(uint8_t corner_active, float gyro_intensity)
{
    int16_t duty;

    if (corner_active)
    {
        // 直角转弯 → 最大负压，增加抓地力
        duty = FAN_DUTY_CORNER;
    }
    else
    {
        // 根据角速度大小线性插值：直道基础 → 弯道满负压
        float ratio = gyro_intensity / GYRO_MAX_FAN;
        if (ratio > 1.0f) ratio = 1.0f;
        duty = FAN_DUTY_STRAIGHT + (int16_t)((FAN_DUTY_CURVE - FAN_DUTY_STRAIGHT) * ratio);
    }

    fan_set_duty(duty);
}

/**
 * @brief 关闭风扇（电机停止时调用）
 */
void fan_off(void)
{
    fan_set_duty(0);
}

