
#include "zf_common_headfile.h"
#include "motor.h"
#include "pid.h"
#include "system_init.h"
#include "img.h"


#define CENTER_INVALID 255
#define FIT_ROWS        40      // 参与拟合的行数
#define MIN_FIT_POINTS   8       // 最少点数
/**
 * @brief 获取摄像头转向误差（原始版本）
 * @return 转向误差（正值右转，负值左转）
 */
// 全局变量，用于节点转向时附加偏置（正值右转，负值左转）

/**
 * @brief 获取摄像头转向误差（原始版本 + 固定偏置）
 * @return 转向误差（正值右转，负值左转）
 */

int16_t get_camera_error(void)
{
    const uint8_t actual_width = MT9V03X_W;              // 原始图像宽度
    const uint8_t desired_center = actual_width / 2;     // 期望中心列
    const uint8_t start_row = 40;      // 拟合起始行（图像底部 FIT_ROWS 行）

    static int16_t last_valid_error = 0;                 // 保存上次有效误差

    float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;
    uint8_t count = 0;

    // 收集底部区域的有效点
    for (uint8_t i = 0; i < FIT_ROWS; i++)
    {
        uint8_t row = start_row + i;
        if (center_line[row] != CENTER_INVALID)
        {
            float x = (float)i;                     // 相对行号
            float y = (float)center_line[row];      // 原始列坐标（0 ~ actual_width-1）
            sum_x  += x;
            sum_y  += y;
            sum_xy += x * y;
            sum_xx += x * x;
            count++;
        }
    }

    // 有效点数不足时返回上次有效值 + 偏置，避免突变
    if (count < MIN_FIT_POINTS)
    {
        return last_valid_error + corner_error_bias;
    }

    // 最小二乘法拟合直线 y = a * x + b
    float denominator = count * sum_xx - sum_x * sum_x;
    if (fabsf(denominator) < 1e-6f)
    {
        return last_valid_error + corner_error_bias;
    }

    float a = (count * sum_xy - sum_x * sum_y) / denominator;  // 斜率
    float b = (sum_y - a * sum_x) / count;                     // 截距

    // 预测最底部（FIT_ROWS-1）的路径中心
    float pred_center = a * (FIT_ROWS - 1) + b;

    // 边界限制
    if (pred_center < 0.0f) pred_center = 0.0f;
    if (pred_center >= (float)actual_width) pred_center = (float)(actual_width - 1);

    // 计算误差：正值表示路径偏左（需右转），负值表示路径偏右（需左转）
    int16_t error = (int16_t)desired_center - (int16_t)pred_center;

    last_valid_error = error;

    // 返回最终误差（原始修正 + 节点偏置）
    int16_t final_error = -error+4+corner_error_bias;
    return final_error;
}

