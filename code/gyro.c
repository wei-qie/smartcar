#include "zf_common_headfile.h"
#include "motor.h"
#include "system_init.h"

const float GYRO_LPF_ALPHA = 0.2;
float filtered_gyro_x = 0.0;   // 滤波后的X轴角速度
float gyro_x_offset = 0.0;      // 零点偏移校准值
float prev_gyro_x = 0.0;        // 上次角速度值（用于移动平均）
volatile uint8 gyro_calibrated = 0;
#define DT_SECONDS           0.010f
float imu_yaw = 0.0f;           // 注意：如果用X轴做偏航，名称可保留但含义变，也可改为 imu_roll
uint8 gyro_init(void)
{
    uint8 ret;
    uint16 i;
    float sum = 0.0f;

    ret = mpu6050_init();
    if (ret != 0) {
        zf_log(0, "MPU6050 init failed!");
        return 1;
    }

    zf_log(1, "MPU6050 init success!");
    gyro_calibrated = 0;
    zf_log(1, "Gyro calibration start, keep car still...");
    system_delay_ms(1000);

    for (i = 0; i < 100; i++) {
        mpu6050_get_gyro();
        float current_gyro_x = mpu6050_gyro_transition(mpu6050_gyro_x);
        sum += current_gyro_x;
        system_delay_ms(10);
    }

    gyro_x_offset = sum / 100.0f;
    gyro_calibrated = 1;
    filtered_gyro_x = 0.0f;

    return 0;
}

float get_filtered_gyro_x(void)
{
    mpu6050_get_gyro();

    float raw_gyro_x = mpu6050_gyro_transition(mpu6050_gyro_x);
    float corrected_gyro_x = raw_gyro_x - gyro_x_offset;

    filtered_gyro_x = GYRO_LPF_ALPHA * corrected_gyro_x + (1.0f - GYRO_LPF_ALPHA) * prev_gyro_x;
    imu_yaw += filtered_gyro_x * DT_SECONDS;

    if (imu_yaw > 180.0f) imu_yaw -= 360.0f;
    else if (imu_yaw < -180.0f) imu_yaw += 360.0f;

    prev_gyro_x = filtered_gyro_x;

    return filtered_gyro_x;
}
float get_imu_yaw(void)
{
    return imu_yaw;
}
/*
#include "zf_common_headfile.h"
#include "motor.h"
#include "system_init.h"
#include "zf_device_imu660ra.h"        // 逐飞 IMU660RA 驱动

*
 * @brief 初始化 IMU660RA 并进行零点校准（Z轴）
 * @return 1-初始化失败 0-初始化成功

const float GYRO_LPF_ALPHA = 0.2f;
float filtered_gyro_z = 0.0f;   // 滤波后的 Z 轴角速度（°/s）
float gyro_z_offset = 0.0f;      // 零点偏移（°/s）
float prev_gyro_z = 0.0f;        // 上次滤波值（用于一阶滤波）
volatile uint8 gyro_calibrated = 0;
#define DT_SECONDS           0.010f   // 调用周期 10ms
float imu_yaw = 0.0f;                 // 偏航角（度，范围 ±180）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RA 并校准 Z 轴陀螺仪零点
// 参数说明     无
// 返回参数     0-成功   1-失败
// 使用示例     if(gyro_init()) while(1);
// 备注         需保持车模静止，采集 100 个点平均作为偏移
//-------------------------------------------------------------------------------------------------------------------
uint8 gyro_init(void)
{
    uint8 ret;
    uint16 i;
    float sum = 0.0f;

    // 1. 初始化 IMU660RA 硬件
    ret = imu660ra_init();
    if (ret != 0) {
        zf_log(0, "IMU660RA init failed!");
        return 1;
    }
    zf_log(1, "IMU660RA init success!");

    gyro_calibrated = 0;

    // 2. 零点校准：静止状态下采集 100 次 Z 轴角速度求平均
    zf_log(1, "Gyro calibration start, keep car still...");
    system_delay_ms(1000);   // 等待稳定

    for (i = 0; i < 100; i++) {
        imu660ra_get_gyro();   // 读取原始数据（三轴）
        // 使用驱动提供的转换宏，直接得到 °/s
        float current_gyro_z = imu660ra_gyro_transition(imu660ra_gyro_z);
        sum += current_gyro_z;
        system_delay_ms(10);   // 10ms 间隔
    }

    gyro_z_offset = sum / 100.0f;
    gyro_calibrated = 1;

    // 3. 清空滤波器和角度
    filtered_gyro_z = 0.0f;
    prev_gyro_z = 0.0f;
    imu_yaw = 0.0f;

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取并滤波 Z 轴角速度，同时积分得到偏航角
// 返回参数     滤波后的 Z 轴角速度（单位：度/秒）
// 使用示例     float omega = get_filtered_gyro_z();
// 备注         需保证以 DT_SECONDS (10ms) 的周期被调用
//-------------------------------------------------------------------------------------------------------------------
float get_filtered_gyro_z(void)
{
    // 1. 读取原始陀螺仪
    imu660ra_get_gyro();

    // 2. 转换为 °/s 并减去零点偏移
    float raw_gyro_z = imu660ra_gyro_transition(imu660ra_gyro_z);
    float corrected_gyro_z = raw_gyro_z - gyro_z_offset;

    // 3. 一阶低通滤波
    filtered_gyro_z = GYRO_LPF_ALPHA * corrected_gyro_z +
                      (1.0f - GYRO_LPF_ALPHA) * prev_gyro_z;

    // 4. 积分得到角度（偏航）
    imu_yaw += filtered_gyro_z * DT_SECONDS;

    // 5. 将角度限幅到 [-180, 180) 区间
    if (imu_yaw > 180.0f)
        imu_yaw -= 360.0f;
    else if (imu_yaw < -180.0f)
        imu_yaw += 360.0f;

    // 6. 更新上次滤波值
    prev_gyro_z = filtered_gyro_z;

    return filtered_gyro_z;
}
*/
