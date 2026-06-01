
//motor.h//
#define CODE MOTOR_H_
#define ENCODER_PPR 1024.0f//������ÿת������
#define CONTROL_PERIOD 0.01f//��ʱ����������10ms
#define RPM_CONVERSION_FACTOR (60.0f/(CONTROL_PERIOD* ENCODER_PPR))//����ϵ��
#include "zf_common_headfile.h"
#include "system_init.h"
void Motor_init();//�����ʼ������
void Motor_Control(int16 left_output, int16 right_output);//������Ʋ��ٺ���
uint8 gyro_init(void);//�����ǳ�ʼ��
float get_filtered_gyro_x(void);//�����ǻ�ȡ���ݺ���
void Encoder_Init();//��������ʼ������
void Encoder_Data_Get(void);//��������ȡ���ݺ��������ж��е��ã�
float Convert_To_RPM(float encoder_speed);//��������ֵת��ΪRPM����
float get_left_encoder_speed();//��ȡʵ�������ٶ�
float get_right_encoder_speed();//��ȡʵ�������ٶ�
// motor.h ������
extern uint8_t is_motor_running(void);
void fan_init(void);
void fan_set_duty(int16_t duty);
void fan_control_update(uint8_t corner_active, float gyro_intensity);
void fan_off(void);
float get_imu_yaw(void);
