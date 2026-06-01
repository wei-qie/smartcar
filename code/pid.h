#include"zf_common_headfile.h"
/*
*
 * @file pid.h
 * @brief 智能车串级PID控制器头文件
 *
 * 本头文件声明了用于差速转向智能车的完整串级PID控制系统接口。
 * 系统包含：方向外环（位置环）、角速度内环、左/右轮速度环。


#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

	============= 方向外环PID控制器（位置环，PD控制） =============
	*
	 * @brief 方向外环（位置环）PD控制器
	 * @param error 从摄像头图像计算出的横向偏差（单位：像素）
	 * @note 此环根据位置偏差计算目标角速度，是串级控制的外环。

	void place_pid(float error);

	*
	 * @brief 获取方向外环计算得到的目标角速度
	 * @return 目标角速度（单位：度/秒）
	 * @note 该值为 place_pid() 函数的计算结果。

	float get_place_target_gyro(void);

	============= 角速度内环PID控制器（PI控制） =============

	*
	 * @brief 角速度内环PI控制器
	 * @param target_gyro 方向外环给出的目标角速度（单位：度/秒）
	 * @param current_gyro 陀螺仪测量的当前角速度（单位：度/秒）
	 * @return 转向控制量，将用于计算左右轮速差
	 * @note 这是串级控制的内环，负责快速、精准地跟踪目标角速度。

	float gyro_pid(float target_gyro, float current_gyro);

	============= 左轮速度环PID控制器（PI控制） =============
	*
	 * @brief 左轮速度环PI控制器
	 * @param aim_speed 左轮目标速度（单位：用户定义，需与编码器反馈单位一致）
	 * @param now_speed 左编码器反馈的当前速度
	 * @return 左电机PWM输出值

	float l_speed_pid(float aim_speed, float now_speed);
	*
	 * @brief 右轮速度环PI控制器
	 * @param aim_speed 右轮目标速度（单位：用户定义，需与编码器反馈单位一致）
	 * @param now_speed 右编码器反馈的当前速度
	 * @return 右电机PWM输出值

	float r_speed_pid(float aim_speed, float now_speed);


#ifdef __cplusplus
}
#endif

#endif  __PID_H
*/
#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h"

// PID参数结构体（更清晰）
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float integral_limit;
    float output_limit;
    float output_min;  // 新增：最小输出值（允许负值）
} PID_Controller_t;

// 声明PID控制器实例
extern PID_Controller_t place_pid;
extern PID_Controller_t gyro_pid;
extern PID_Controller_t speed_left_pid;
extern PID_Controller_t speed_right_pid;

// 函数声明
void pid_init_all(void);
void place_pid_calculate(int16_t error);
float gyro_pid_calculate(float target, float current);
float speed_pid_calculate(PID_Controller_t* pid, float target, float current,float deadband_forward,
        float deadband_backward);
float get_place_target_gyro(void);
// 双模PID切换
void pid_switch_mode(uint8_t is_curve);   // is_curve = 1 弯道，0 直道
void pid_init_dual_mode(void);            // 初始化双模参数（上电调用）

#endif


