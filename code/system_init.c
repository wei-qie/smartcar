#include"system_init.h"
#include "zf_common_headfile.h"
bool flag = 1;
int time = 0;
void system_init()
{

	//电机初始化
	Motor_init();
	//陀螺仪初始化
	gyro_init();
	//编码器初始化
	Encoder_Init();
	fan_init();
	//按健初始化
		
	//定时器中断初

	pit_ms_init(CCU60_CH0, 10);


}
