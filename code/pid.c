/*
#include "zf_common_headfile.h"
#include "pid.h"
============= 方向外环PID控制器（位置环，PD控制）==============
*
 * @brief 方向外环（位置环）PD控制器
 * @param error 从摄像头图像计算出的横向偏差（单位：像素）
 * @note 此环输出目标角速度，而非直接控制量。这是串级控制的第一步。

============= 方向外环PID控制器（位置环，PD控制） =============
float place_error = 0.0f;
float place_last_error = 0.0f;
float place_out = 0.0f;
float place_last_out = 0.0f;
float place_kp = 1.5f;    // 比例系数示例值，需调试
float place_kd = 0.8f;    // 微分系数示例值，需调试
float place_target_gyro = 0.0f; // 外环输出的目标角速度
============= 角速度内环PID控制器（PI控制） =============
    float gyro_pid_out = 0.0f;
    float gyro_last_error = 0.0f;
    float gyro_error = 0.0f;
    float gyro_kp = 0.05f;   // 比例系数示例值
    float gyro_ki = 0.001f;  // 积分系数示例值
    float gyro_integral = 0.0f;
    const float GYRO_INTEGRAL_LIMIT = 100.0f; // 积分限幅
 ============= 左轮速度环PID控制器（PI控制） =============
      float l_speed_out = 0.0f;
      float l_speed_last_error = 0.0f;
      float l_speed_error = 0.0f;
      float l_speed_kp = 5.0f;   // 比例系数示例值
      float l_speed_ki = 0.05f;  // 积分系数示例值
      float l_speed_integral = 0.0f;
      const float L_SPEED_INTEGRAL_LIMIT = 500.0f;
      const float MAX_L_SPEED_OUT = 1000.0f; // 对应PWM最大值
  ============= 右轮速度环PID控制器（PI控制） =============
      float r_speed_out = 0.0f;
      float r_speed_last_error = 0.0f;
      float r_speed_error = 0.0f;
      float r_speed_kp = 5.0f;   // 比例系数示例值
      float r_speed_ki = 0.05f;  // 积分系数示例值
      float r_speed_integral = 0.0f;
      const float R_SPEED_INTEGRAL_LIMIT = 500.0f;
      const float MAX_R_SPEED_OUT = 1000.0f;
void place_pid(float error)
{
    // 1. 更新误差历史值（用于微分计算）
    place_last_error = place_error;
    place_error = error;

    // 2. PD控制计算：输出目标角速度（物理意义：为消除位置偏差，期望车身转动的速度）
    // place_target_gyro = Kp * e + Kd * (de/dt)
    // 这是完整的PD公式，其中 (place_error - place_last_error) / dt 近似为微分
    // 此处假设在固定周期中断中调用，dt已隐含在参数Kd中
    place_target_gyro = place_kp * place_error + place_kd * (place_error - place_last_error);

    // 3. 输出限幅：防止目标角速度指令过大导致失控
    // 假设最大安全目标角速度为 300 度/秒，需根据车辆实际性能调整
    const float MAX_TARGET_GYRO = 300.0f;
    if (place_target_gyro > MAX_TARGET_GYRO) {
        place_target_gyro = MAX_TARGET_GYRO;
    }
    else if (place_target_gyro < -MAX_TARGET_GYRO) {
        place_target_gyro = -MAX_TARGET_GYRO;
    }
}

============= 角速度内环PID控制器（PI控制）==============
*
 * @brief 角速度内环PI控制器（真正的串级内环）
 * @param target_gyro 方向外环给出的目标角速度
 * @param current_gyro 陀螺仪测量的当前角速度（单位：度/秒）
 * @return 转向控制量，将用于计算左右轮速差

float gyro_pid(float target_gyro, float current_gyro)
{
    // 1. 计算角速度偏差
    gyro_last_error = gyro_error;
    gyro_error = target_gyro - current_gyro;

    // 2. 积分项计算（带限幅，抗饱和）
    gyro_integral += gyro_error;
    // 积分限幅防止Windup
    if (gyro_integral > GYRO_INTEGRAL_LIMIT) {
        gyro_integral = GYRO_INTEGRAL_LIMIT;
    }
    else if (gyro_integral < -GYRO_INTEGRAL_LIMIT) {
        gyro_integral = -GYRO_INTEGRAL_LIMIT;
    }

    // 3. PI控制计算
    gyro_pid_out = gyro_kp * gyro_error + gyro_ki * gyro_integral;

    // 4. 输出限幅：此输出即为最终的转向调节量
    const float MAX_TURN_OUTPUT = 50.0f; // 最大转向调节量，影响差速大小
    if (gyro_pid_out > MAX_TURN_OUTPUT) {
        gyro_pid_out = MAX_TURN_OUTPUT;
    }
    else if (gyro_pid_out < -MAX_TURN_OUTPUT) {
        gyro_pid_out = -MAX_TURN_OUTPUT;
    }

    return gyro_pid_out;
}

============= 左轮速度环PID控制器（PI控制）==============

*
 * @brief 左轮速度环PI控制器
 * @param aim_speed 左轮目标速度（由基础速度和转向量合成）
 * @param now_speed 左编码器反馈的当前速度
 * @return 左电机PWM输出值

float l_speed_pid(float aim_speed, float now_speed)
{
    // 1. 计算速度偏差
    l_speed_last_error = l_speed_error;
    l_speed_error = aim_speed - now_speed;

    // 2. 积分项计算（带限幅）
    l_speed_integral += l_speed_error;
    if (l_speed_integral > L_SPEED_INTEGRAL_LIMIT) {
        l_speed_integral = L_SPEED_INTEGRAL_LIMIT;
    }
    else if (l_speed_integral < -L_SPEED_INTEGRAL_LIMIT) {
        l_speed_integral = -L_SPEED_INTEGRAL_LIMIT;
    }

    // 3. 标准PI控制：比例项 + 积分项
    // 注意：此处是位置式PI，比例项直接作用于当前误差
    l_speed_out = l_speed_kp * l_speed_error + l_speed_ki * l_speed_integral;

    // 4. 输出限幅
    if (l_speed_out > MAX_L_SPEED_OUT) {
        l_speed_out = MAX_L_SPEED_OUT;
    }
    else if (l_speed_out < 0) { // PWM通常不为负，反转由电机驱动方向引脚控制
        l_speed_out = 0;
    }

    return l_speed_out;
}

============= 右轮速度环PID控制器（PI控制）==============
*
 * @brief 右轮速度环PI控制器
 * @param aim_speed 右轮目标速度（由基础速度和转向量合成）
 * @param now_speed 右编码器反馈的当前速度
 * @return 右电机PWM输出值

float r_speed_pid(float aim_speed, float now_speed)
{
    // 实现与左轮对称，省略重复注释
    r_speed_last_error = r_speed_error;
    r_speed_error = aim_speed - now_speed;

    r_speed_integral += r_speed_error;
    if (r_speed_integral > R_SPEED_INTEGRAL_LIMIT) {
        r_speed_integral = R_SPEED_INTEGRAL_LIMIT;
    }
    else if (r_speed_integral < -R_SPEED_INTEGRAL_LIMIT) {
        r_speed_integral = -R_SPEED_INTEGRAL_LIMIT;
    }

    r_speed_out = r_speed_kp * r_speed_error + r_speed_ki * r_speed_integral;

    if (r_speed_out > MAX_R_SPEED_OUT) {
        r_speed_out = MAX_R_SPEED_OUT;
    }
    else if (r_speed_out < 0) {
        r_speed_out = 0;
    }

    return r_speed_out;
}
*/
#include "zf_common_headfile.h"
#include "pid.h"
#include <math.h>

// ========== 位置环（外环）PID ==========
PID_Controller_t place_pid = {
    .kp = 0.6f,        // 像素误差 -> 角速度：0.5 (deg/s)/像素
    .ki = 0.0f,        // 位置环一般只用PD
    .kd = 1.0f,        // 微分系数
    .integral = 0.0f,
    .last_error = 0.0f,
    .integral_limit = 5000.0f,
    .output_limit = 3000.0f,     // 最大目标角速度 200 deg/s
    .output_min = -3000.0f       // 最小目标角速度
};

static float place_target_gyro = 0.0f;  // 外环输出的目标角速度

// ========== 角速度环（内环）PID ==========
PID_Controller_t gyro_pid = {
    .kp = 4.0,        // 角速度误差 -> PWM：5 PWM/(deg/s)
    .ki = 0.5f,        // 积分系数
    .kd = 0.4f,        // 内环一般只用PI
    .integral = 0.0f,
    .last_error = 0.0f,
    .integral_limit = 6000.0f,
    .output_limit = 3000.0f,    // 最大PWM输出
    .output_min = -3000.0f      // 最小PWM输出（负值用于反转）
};

// ========== 左轮速度环PID ==========
PID_Controller_t speed_left_pid = {
    .kp = 0.8f,        // RPM误差 -> PWM：0.8 PWM/RPM
    .ki = 0.098f,       // 积分系数
    .kd = 0.0f,
    .integral = 0.0f,
    .last_error = 0.0f,
    .integral_limit = 1500.0f,
    .output_limit = 3000.0f,    // 最大PWM输出
    .output_min = -3000.0f      // 允许负值输出
};

// ========== 右轮速度环PID ==========
PID_Controller_t speed_right_pid = {
    .kp = 0.8f,        // RPM误差 -> PWM：0.8 PWM/RPM
    .ki = 0.098f,       // 积分系数
    .kd = 0.0f,
    .integral = 0.0f,
    .last_error = 0.0f,
    .integral_limit = 1500.0f,
    .output_limit = 3000.0f,    // 最大PWM输出
    .output_min = -3000.0f      // 允许负值输出
};

// ========== PID初始化函数 ==========
void pid_init_all(void)
{
    // 重置所有PID控制器
    place_pid.integral = 0.0f;
    place_pid.last_error = 0.0f;

    gyro_pid.integral = 0.0f;
    gyro_pid.last_error = 0.0f;

    speed_left_pid.integral = 0.0f;
    speed_left_pid.last_error = 0.0f;

    speed_right_pid.integral = 0.0f;
    speed_right_pid.last_error = 0.0f;

    place_target_gyro = 0.0f;
    pid_init_dual_mode();

}

// ========== 位置环PID计算 ==========
void place_pid_calculate(int16_t error_pixels)
{
    float error =- error_pixels;

    // 比例项
    float p_term = place_pid.kp * error;

    // 微分项（离散微分）
    float d_term = 0.0f;
    if (place_pid.kd > 0.0f) {
        float error_diff = error - place_pid.last_error;
        d_term = place_pid.kd * error_diff;
        place_pid.last_error = error;
    }

    // 位置环通常只用PD控制
    float output = p_term + d_term;

    // 输出限幅
    if (output > place_pid.output_limit) {
        output = place_pid.output_limit;
    } else if (output < place_pid.output_min) {
        output = place_pid.output_min;
    }

    // 保存为目标角速度
    place_target_gyro = output;  // 单位：deg/s
}

// ========== 角速度环PID计算（带微分项） ==========
float gyro_pid_calculate(float target_deg_s, float current_deg_s)
{
    float error = current_deg_s - target_deg_s;  // 你的误差定义

    // 比例项
    float p_term = gyro_pid.kp * error;

    // 积分项
    gyro_pid.integral += error;
    if (gyro_pid.integral > gyro_pid.integral_limit) {
        gyro_pid.integral = gyro_pid.integral_limit;
    } else if (gyro_pid.integral < -gyro_pid.integral_limit) {
        gyro_pid.integral = -gyro_pid.integral_limit;
    }
    float i_term = gyro_pid.ki * gyro_pid.integral;

    // 微分项（新增）
    float d_term = 0.0f;
    // 计算误差变化率
    float error_diff = error - gyro_pid.last_error;
    d_term = gyro_pid.kd * error_diff;
    // 更新上一次误差，供下次使用
    gyro_pid.last_error = error;

    // PID总输出
    float output = p_term + i_term + d_term;

    // 输出限幅
    if (output > gyro_pid.output_limit) {
        output = gyro_pid.output_limit;
    } else if (output < gyro_pid.output_min) {
        output = gyro_pid.output_min;
    }

    return output;
}

// ========== 通用速度环PID计算 ==========
// 速度环PID计算（加入积分分离）
float speed_pid_calculate(PID_Controller_t* pid,
                          float target_rpm,
                          float current_rpm,
                          float deadband_forward,
                          float deadband_backward)
{
    float error = target_rpm - current_rpm;

    // 比例项
    float p_term = pid->kp * error;

    // 积分项（带积分分离）
    float i_term = 0.0f;
    float integral_threshold = 100.0f;  // 误差小于此值时积分才工作
    if (fabsf(error) < integral_threshold)
    {
        pid->integral += error;
        // 积分限幅
        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        else if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;
        i_term = pid->ki * pid->integral;
    }
    else
    {
        // 误差大时清空积分，防止积分饱和
        pid->integral = 0.0f;
    }

    // PID基本输出
    float output = p_term + i_term;

    // ===== 死区补偿 =====
    if (output > 0)
    {
        // 正向：若输出小于死区阈值，则强制提升到死区阈值
        if (output < deadband_forward)
            output = deadband_forward;
    }
    else if (output < 0)
    {
        // 反向：若输出绝对值小于死区阈值，则强制设为负的死区阈值
        if (output > -deadband_backward)
            output = -deadband_backward;
    }
    // output == 0 时保持不变

    // 输出限幅（确保不超过电机最大允许PWM）
    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < pid->output_min)
        output = pid->output_min;

    return output;
}

// ========== 获取位置环目标角速度 ==========
float get_place_target_gyro(void)
{
    return place_target_gyro;
}
// 机械对称性测试
// ========== 双模PID参数 ==========
typedef struct {
    float kp, ki, kd;
    float integral_limit;
    float output_limit;
    float output_min;
} PID_Params_t;

// 直道参数组（平稳、不易震荡）
static const PID_Params_t straight_params[] = {
    // place_pid (位置外环)   kp    ki   kd   积分限幅  输出限幅  输出下限
    { 1.0f, 0.0f, 2.0f, 5000.0f, 2500.0f, -2500.0f },
    // gyro_pid  (角速度内环)
    { 3.0f, 0.3f, 1.2f, 6000.0f, 2500.0f, -2500.0f },
    // speed_left_pid
    { 0.8f, 0.10f, 0.0f, 1500.0f, 3000.0f, -3000.0f },
    // speed_right_pid
    { 0.8f, 0.10f, 0.0f, 1500.0f, 3000.0f, -3000.0f }
            // place_pid (位置外环
            // place_pid (位置外环)

};


// 弯道参数组（更激进的比例和积分，加快响应）
static const PID_Params_t curve_params[] = {
    // place_pid
        { 0.6f, 0.0f, 1.0f, 5000.0f, 3000.0f, -3000.0f },
          // gyro_pid
          { 6.0f, 0.5f, 0.6f, 6000.0f, 3000.0f, -3000.0f },
          // speed_left_pid
          { 0.8f, 0.098f, 0.0f, 1500.0f, 3000.0f, -3000.0f },
          // speed_right_pid
          { 0.8f, 0.098f, 0.0f, 1500.0f, 3000.0f, -3000.0f }
      };

/**
 * @brief 将参数写入PID控制器，并重置积分/历史误差
 */
static void apply_params(PID_Controller_t *pid, const PID_Params_t *params) {
    pid->kp = params->kp;
    pid->ki = params->ki;
    pid->kd = params->kd;
    pid->integral_limit = params->integral_limit;
    pid->output_limit = params->output_limit;
    pid->output_min = params->output_min;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

/**
 * @brief 切换PID模式（直道/弯道）
 * @param is_curve  0 = 直道，1 = 弯道
 */
void pid_switch_mode(uint8_t is_curve) {
    const PID_Params_t *params = is_curve ? curve_params : straight_params;
    apply_params(&place_pid,       &params[0]);
    apply_params(&gyro_pid,        &params[1]);
    apply_params(&speed_left_pid,  &params[2]);
    apply_params(&speed_right_pid, &params[3]);
}

/**
 * @brief 初始化双模参数（可放在上电时调用一次）
 */
void pid_init_dual_mode(void) {
    pid_switch_mode(0);   // 默认直道参数
}








