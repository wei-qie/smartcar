#include"zf_common_headfile.h"
/*============= �����⻷PID��������λ�û���PD���ƣ�==============*/
extern float place_error;
extern float place_last_error;
extern float place_out;
extern float place_last_out;
extern float place_kp;    // ����ϵ��ʾ��ֵ�������
extern float place_kd;    // ΢��ϵ��ʾ��ֵ�������
extern float place_target_gyro; // �⻷�����Ŀ����ٶ�
/*============= ���ٶ��ڻ�PID��������PI���ƣ�==============*/
extern float gyro_pid_out;
extern float gyro_last_error;
extern float gyro_error;
extern float gyro_kp;   // ����ϵ��ʾ��ֵ
extern float gyro_ki;  // ����ϵ��ʾ��ֵ
extern float gyro_integral;
extern const float GYRO_INTEGRAL_LIMIT; // �����޷�
/*============= �����ٶȻ�PID��������PI���ƣ�==============*/
extern float l_speed_out;
extern float l_speed_last_error;
extern float l_speed_error;
extern float l_speed_kp;   // ����ϵ��ʾ��ֵ
extern float l_speed_ki;  // ����ϵ��ʾ��ֵ
extern  float l_speed_integral;
extern const float L_SPEED_INTEGRAL_LIMIT;
extern const float MAX_L_SPEED_OUT; // ��ӦPWM���ֵ
/*============= �����ٶȻ�PID��������PI���ƣ�==============*/
extern float r_speed_out;
extern float r_speed_last_error;
extern float r_speed_error;
extern float r_speed_kp;   // ����ϵ��ʾ��ֵ
extern float r_speed_ki;  // ����ϵ��ʾ��ֵ
extern float r_speed_integral;
extern const float R_SPEED_INTEGRAL_LIMIT;
extern const float MAX_R_SPEED_OUT;
/*=============��������ز���================*/
extern float filtered_gyro_z;   // �˲����Z����ٶ�
extern float gyro_z_offset;     // ���ƫ��У׼ֵ
extern float prev_gyro_z;       // �ϴν��ٶ�ֵ�������ƶ�ƽ����
extern const float GYRO_LPF_ALPHA;  // ��ͨ�˲�ϵ�� (0~1��ԽС�˲�Խǿ)
extern volatile uint8 gyro_calibrated;  // 0��ʾδУ׼��1��ʾ��У׼//���ڿ��������Ƿ��ʼ���ɹ�
/*=============����ͱ�������ز���===========*/
extern int target_speed;//Ŀ�����仯�ٶ�
extern int duty;//ռ�ձ�
extern int encoder_speed;//����仯�ٶ�
extern int encoder_raw;//��ʼ����ֵ
extern int32 total_encoder;//�ܱ���ֵ
/*===========���ƺ���������ʼ��===========*/
extern float turn_output;
extern float base_speed;
extern float left_target_speed;
extern float right_target_speed;
extern float left_now_speed;
extern float right_now_speed;
extern float left_pwm;
extern float right_pwm;
/*===========椋庢墖棰勮繍琛屾帶鍒�===========*/
extern uint8_t fan_prerun_active;
extern uint16_t fan_prerun_counter;
