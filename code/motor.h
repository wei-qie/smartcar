
//motor.h//
#define CODE MOTOR_H_
#define ENCODER_PPR 1024.0f//编码器每转脉冲数
#define CONTROL_PERIOD 0.01f//定时器控制周期10ms
#define RPM_CONVERSION_FACTOR (60.0f/(CONTROL_PERIOD* ENCODER_PPR))//比例系数
#include "zf_common_headfile.h"
#include "system_init.h"
void Motor_init();//电机初始化函数
void Motor_Control(int16 left_output, int16 right_output);//电机控制差速函数
uint8 gyro_init(void);//陀螺仪初始化
float get_filtered_gyro_x(void);//陀螺仪获取数据函数
void Encoder_Init();//编码器初始化函数
void Encoder_Data_Get(void);//编码器获取数据函数（在中断中调用）
float Convert_To_RPM(float encoder_speed);//编码器数值转换为RPM函数
float get_left_encoder_speed();//获取实际左轮速度
float get_right_encoder_speed();//获取实际右轮速度
// motor.h 中添加
extern uint8_t is_motor_running(void);
void fan_init(void);
void fan_set_duty(int16_t duty);
void fan_on(void);
void fan_off(void);
float get_imu_yaw(void);
