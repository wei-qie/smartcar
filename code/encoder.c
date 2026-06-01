
// ========== encoder.c ==========
#include "zf_common_headfile.h"
#include "system_init.h"
#include "motor.h"
#include "pid.h"  // 新增：包含PID头文件
#include <math.h>

// ========== 配置参数 ==========
#define ENCODER_PPR           1024.0f     // 编码器每转脉冲数
#define CONTROL_PERIOD_S      0.01f       // 控制周期10ms
#define ENCODER_QUAD          1           // 4倍频
#define GEAR_RATIO            1           // 减速比

// ========== 计算常数 ==========
#define PULSES_PER_REV        (ENCODER_PPR * GEAR_RATIO * ENCODER_QUAD)
#define PULSE_TO_RPM_FACTOR   (60.0f / (PULSES_PER_REV * CONTROL_PERIOD_S))
#define SPEED_FILTER_ALPHA    0.3f        // 滤波系数：0.1-0.5，越小越平滑

// ========== 轮子参数（用于单位转换）==========
#define WHEEL_DIAMETER_M      0.065f      // 轮子直径（米）
#define MAX_MOTOR_RPM         150.0f      // 电机最大RPM
#define MAX_PWM_VALUE         10000.0f    // PWM最大值

// ========== 电机结构体（扩展）==========
typedef struct {
    // 编码器相关
    int32_t last_encoder_count;
    int32_t current_encoder_count;
    int32_t encoder_delta;
    int32_t total_encoder;

    // 速度相关（RPM单位）
    float speed_rpm;
    float raw_speed_rpm;
    float filter_alpha;

    // 新增：不同单位的速度
    float speed_mps;        // 米/秒
    float speed_pwm;        // PWM等效值（用于与PID输出比较）

    // 新增：校准参数
    float pwm_per_rpm;      // PWM/RPM转换系数
    float deadzone_pwm;     // 死区PWM值

} Motor_t;

// ========== 全局变量 ==========
Motor_t motor_l;    // 左轮电机
Motor_t motor_r;    // 右轮电机

// ========== 函数声明 ==========
static int32_t calculate_encoder_delta(int32_t current, int32_t last);
static void update_motor_speed(Motor_t* motor);
static float rpm_to_mps(float rpm);
static float rpm_to_pwm(float rpm);

// ========== 编码器初始化 ==========
void Encoder_Init(void)
{

    // 左编码器：CH1接脉冲(LSB)，CH2接方向(DIR)
       encoder_dir_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
       // 右编码器：CH1接脉冲(LSB)，CH2接方向(DIR)
       encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);
    // 初始化电机结构体0
    motor_l.last_encoder_count = 0;
    motor_l.current_encoder_count = 0;
    motor_l.encoder_delta = 0;
    motor_l.speed_rpm = 0.0f;
    motor_l.raw_speed_rpm = 0.0f;
    motor_l.speed_mps = 0.0f;
    motor_l.speed_pwm = 0.0f;
    motor_l.total_encoder = 0;
    motor_l.filter_alpha = SPEED_FILTER_ALPHA;
    motor_l.pwm_per_rpm = MAX_PWM_VALUE / MAX_MOTOR_RPM;  // 默认转换系数
    motor_l.deadzone_pwm = 500.0f;  // 默认死区

    motor_r.last_encoder_count = 0;
    motor_r.current_encoder_count = 0;
    motor_r.encoder_delta = 0;
    motor_r.speed_rpm = 0.0f;
    motor_r.raw_speed_rpm = 0.0f;
    motor_r.speed_mps = 0.0f;
    motor_r.speed_pwm = 0.0f;
    motor_r.total_encoder = 0;
    motor_r.filter_alpha = SPEED_FILTER_ALPHA;
    motor_r.pwm_per_rpm = MAX_PWM_VALUE / MAX_MOTOR_RPM;
    motor_r.deadzone_pwm = 500.0f;

    // 读取初始值，不清零
    motor_l.last_encoder_count = -encoder_get_count(TIM4_ENCODER);
    motor_r.last_encoder_count = encoder_get_count(TIM6_ENCODER);
}

// ========== 获取编码器数据（10ms调用一次） ==========
void Encoder_Data_Get(void)
{
    // ===== 左电机 =====
    motor_l.current_encoder_count = -encoder_get_count(TIM4_ENCODER);
    motor_l.encoder_delta = calculate_encoder_delta(
        motor_l.current_encoder_count,
        motor_l.last_encoder_count
    );
    motor_l.last_encoder_count = motor_l.current_encoder_count;
    update_motor_speed(&motor_l);

    // ===== 右电机 =====
    // 根据实际安装方向决定是否取反
    motor_r.current_encoder_count = encoder_get_count(TIM6_ENCODER); // 取反
    motor_r.encoder_delta = calculate_encoder_delta(
        motor_r.current_encoder_count,
        motor_r.last_encoder_count
    );
    motor_r.last_encoder_count = motor_r.current_encoder_count;
    update_motor_speed(&motor_r);

}

// ========== 计算编码器增量（处理16位计数器溢出） ==========
static int32_t calculate_encoder_delta(int32_t current, int32_t last)
{
    int32_t delta = current - last;

    // GPT12模块是16位计数器
    const int32_t COUNTER_MAX = 32767;
    const int32_t COUNTER_MIN = -32768;
    const int32_t COUNTER_RANGE = 65536; // 2^16

    if (delta > COUNTER_MAX) {
        delta -= COUNTER_RANGE;
    }
    else if (delta < COUNTER_MIN) {
        delta += COUNTER_RANGE;
    }

    return delta;
}

/*
// ========== RPM转米/秒 ==========
static float rpm_to_mps(float rpm)
{
    // 转速(rpm) -> 轮子周长(m) * 转速(rps)
    // rpm/60 = rps (转/秒)
    float rps = rpm / 60.0f;
    float circumference = 3.1415926f * WHEEL_DIAMETER_M;
    return rps * circumference;
}

// ========== RPM转PWM ==========
static float rpm_to_pwm(float rpm)
{
    // 简单线性转换：PWM = pwm_per_rpm * |rpm| + deadzone_pwm
    float pwm = motor_l.pwm_per_rpm * fabs(rpm) + motor_l.deadzone_pwm;

    // 保持符号
    if (rpm < 0) {
        pwm = -pwm;
    }

    // 限幅
    if (pwm > MAX_PWM_VALUE) pwm = MAX_PWM_VALUE;
    if (pwm < -MAX_PWM_VALUE) pwm = -MAX_PWM_VALUE;

    return pwm;
}
*/

// ========== 更新电机速度 ==========
static void update_motor_speed(Motor_t* motor)
{
    // 计算原始速度（RPM）
    motor->raw_speed_rpm = motor->encoder_delta * PULSE_TO_RPM_FACTOR;

    // 一阶低通滤波
    motor->speed_rpm = motor->speed_rpm * (1.0f - motor->filter_alpha)
                     + motor->raw_speed_rpm * motor->filter_alpha;

   /* // 转换为其他单位
    motor->speed_mps = rpm_to_mps(motor->speed_rpm);
    motor->speed_pwm = rpm_to_pwm(motor->speed_rpm);*/

    // 累积总脉冲（用于里程计算）
    motor->total_encoder += motor->encoder_delta;

    // 死区处理：非常低的速度视为0
    if (fabs(motor->speed_rpm) < 2.0f) {
        motor->speed_rpm = 0.0f;
        motor->speed_mps = 0.0f;
        motor->speed_pwm = 0.0f;
    }
}

// ========== 获取速度函数 ==========
float get_left_encoder_speed(void) { return motor_l.speed_rpm; }
float get_right_encoder_speed(void) { return motor_r.speed_rpm; }
float get_left_speed_mps(void) { return motor_l.speed_mps; }
float get_right_speed_mps(void) { return motor_r.speed_mps; }
float get_left_speed_pwm(void) { return motor_l.speed_pwm; }
float get_right_speed_pwm(void) { return motor_r.speed_pwm; }
float get_left_raw_speed(void) { return motor_l.raw_speed_rpm; }
float get_right_raw_speed(void) { return motor_r.raw_speed_rpm; }
int32_t get_left_encoder_delta(void) { return motor_l.encoder_delta; }
int32_t get_right_encoder_delta(void) { return motor_r.encoder_delta; }

// ========== 设置滤波系数 ==========
void set_speed_filter_alpha(float alpha_left, float alpha_right)
{
    if (alpha_left >= 0.0f && alpha_left <= 1.0f) {
        motor_l.filter_alpha = alpha_left;
    }
    if (alpha_right >= 0.0f && alpha_right <= 1.0f) {
        motor_r.filter_alpha = alpha_right;
    }
}

// ========== 设置校准参数 ==========
void set_motor_calibration(float pwm_per_rpm_left, float deadzone_left,
                          float pwm_per_rpm_right, float deadzone_right)
{
    motor_l.pwm_per_rpm = pwm_per_rpm_left;
    motor_l.deadzone_pwm = deadzone_left;
    motor_r.pwm_per_rpm = pwm_per_rpm_right;
    motor_r.deadzone_pwm = deadzone_right;
}

