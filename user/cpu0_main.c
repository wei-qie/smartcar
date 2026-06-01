#include "zf_common_headfile.h"
#include "motor.h"
#include "pid.h"
#include "system_init.h"
#include "img.h"
#define LED1                    (P20_9)
#pragma section all "cpu0_dsram"
// 铏氭嫙绀烘尝鍣ㄩ�氶亾鏁版嵁
float oscilloscope_data[4];  // 0:宸︾洰鏍嘡PM  1:宸﹀疄闄匯PM  2:鍙崇洰鏍嘡PM  3:鍙冲疄闄匯PM*
float turn_pwm_adjust=0.0;
float current_gyro=0.0;
float target_gyro=0.0;
extern float corner_target_gyro;
extern uint8_t corner_gyro_active;
uint8_t exit_cnt = 0;
/*void Control1(void)
{
    // 4. 瑙掗�熷害鐜疨ID璁＄畻锛堝唴鐜級锛氳閫熷害璇樊 -> PWM璋冭妭閲�
    current_gyro = get_filtered_gyro_x();
    turn_pwm_adjust =gyro_pid_calculate(target_gyro, current_gyro);

}*/
void Control1(void)
{/*
    float current_gyro = get_filtered_gyro_x();

    float final_target_gyro;
    if (corner_gyro_active) {
        final_target_gyro = corner_target_gyro;   // 浼樺厛浣跨敤鎷愮偣鐩爣
    } else {
        final_target_gyro = target_gyro;           // 浣跨敤浣嶇疆鐜緭鍑�
    }

    turn_pwm_adjust = gyro_pid_calculate(final_target_gyro, current_gyro);*/
        float current_gyro = get_filtered_gyro_x();
        // 此时 target_gyro 已经被 Control3 按照当前模式设定好
        turn_pwm_adjust = gyro_pid_calculate(target_gyro, current_gyro);

}
void Control2(void)
{
   /* if(node_index>=20)
                   {
                       Motor_Control(0,0);
                   }*/
    // ===== 椋庢墖棰勮繍琛岄樁锛氬彧杞鎵囷紝涓嶅惎鍔ㄧ數鏈� =====
    if(fan_prerun_active)
    {
        if(--fan_prerun_counter == 0)
        {
            fan_prerun_active = 0;
            motor_start();  // 棰勮繍琛岀粨鏉燂紝鍏佽鐢垫満鍚姩
            printf("Motor START!\r\n");
        }
        Motor_Control(0, 0);
        return;
    }

    // 5. 鍚堟垚宸﹀彸杞洰鏍嘝WM鍊�
       // 娉ㄦ剰锛歵urn_pwm_adjust鏄痳pm鍗曚綅锛岀洿鎺ヤ笌base_pwm鐩稿姞
    // 鐢垫満鏈惎鍔ㄦ椂锛岀洿鎺ュ仠姝㈠苟杩斿洖
    if(!is_motor_running()) {
        Motor_Control(0, 0);
        fan_off();
        return;
    }
       float base_speed_rpm=2800.0f;     // 鍩虹閫熷害锛圧PM锛�
       float left_target_rpm = base_speed_rpm +turn_pwm_adjust;
       float right_target_rpm= base_speed_rpm -turn_pwm_adjust;

       // 6. 鑾峰彇褰撳墠閫熷害锛堢紪鐮佸櫒杩斿洖鐨勬槸RPM锛�
       Encoder_Data_Get();
       float left_now_rpm = get_left_encoder_speed();
       float right_now_rpm = get_right_encoder_speed();

       // 7. 灏嗙洰鏍嘝WM杞崲涓虹洰鏍嘡PM锛堢敤浜庨�熷害鐜級
       // 娉ㄦ剰锛氳繖閲岄渶瑕侀�嗚浆鎹紝浣嗕负浜嗙畝鍖栵紝鎴戜滑鍙互鐩存帴浣跨敤PWM浣滀负鐩爣
       // 鎴栬�呬娇鐢≧PM浣滀负鐩爣锛屼絾闇�瑕佺‘淇濆崟浣嶇粺涓�

       // 鏂规A锛氱洿鎺ヤ娇鐢≒WM浣滀负鐩爣锛堟帹鑽愶紝鍥犱负PID鍙傛暟鏄寜PWM鍗曚綅璋冭瘯鐨勶級
       float left_target_for_pid = left_target_rpm;
       float right_target_for_pid =right_target_rpm;


       // 8. 閫熷害鐜疨ID璁＄畻锛堣緭鍏ヨ緭鍑洪兘鏄疨WM鍗曚綅锛�
       float left_output_pwm = speed_pid_calculate(&speed_left_pid, left_target_for_pid, left_now_rpm,0.0,0.0);
       float right_output_pwm = speed_pid_calculate(&speed_right_pid, right_target_for_pid, right_now_rpm,0.0,0.0);

       // 9. 鐢垫満鎺у埗
       Motor_Control(left_output_pwm,right_output_pwm);
     /*  switch(current_track)
       {
           case 0:
               if(corner_index>=9)
               {
                 for(int pwm=left_output_pwm;pwm>=0;pwm-=100)
                 {
                     if(pwm<0)
                   {
                    pwm=0;
                   }
                    Motor_Control(pwm,pwm);
                 }
               }
           case 1:
               if(corner_index>=14)
               {
                  for(int pwm=left_output_pwm;pwm>=0;pwm-=100)
                 {
                    if(pwm<0)
                  {
                    pwm=0;
                  }
                    Motor_Control(pwm,pwm);
                 }

               }
           case 2:
               if(corner_index>=16)
               {
                  for(int pwm=left_output_pwm;pwm>=0;pwm-=100)
                 {
                    if(pwm<0)
                   {
                    pwm=0;
                   }
                    Motor_Control(pwm,pwm);
                 }
               }
       }*/

}
/*void Control3(void)
{
    if (!gyro_calibrated) {
        Motor_Control(500, 500);
        return;
    }

    int16_t pixel_error = get_camera_error();

    // ===== 新增：只在转弯状态变化时切换一次 PID =====
    static uint8_t last_gyro_active = 0;
    if (corner_gyro_active != last_gyro_active) {
        last_gyro_active = corner_gyro_active;
        pid_switch_mode(corner_gyro_active ? 1 : 0);
    }

    // ===== 以下完全保持你原来的逻辑 =====
    if (corner_gyro_active) {
        target_gyro = get_filtered_gyro_x();
    } else {
        place_pid_calculate(pixel_error);
        target_gyro = get_place_target_gyro();
    }
}*/
void Control3(void)
{
    if (!gyro_calibrated) {
        Motor_Control(500, 500);
        return;
    }

    // ===== PID 参数切换 =====
    static uint8_t last_turning = 0;
    static uint16_t turn_timeout = 0;   // 杞集甯ф暟瓒呮椂璁℃暟鍣�
    static uint8_t exit_cnt = 0;        // 杩炵画婊¤冻閫�鍑烘潯浠剁殑甯ф暟

    if (corner_turning_active != last_turning) {
        last_turning = corner_turning_active;
        pid_switch_mode(corner_turning_active ? 1 : 0);
        // 杩涘叆杞集鏃堕噸缃秴鏃跺拰閫�鍑鸿鏁板櫒
        if (corner_turning_active) {
            turn_timeout = 0;
            exit_cnt = 0;
        }
    }

    // ===== 鐩磋杞集妯″紡 =====
    if (corner_turning_active)
    {
        turn_timeout++;  // 姣忓抚璁℃暟

        float current_yaw = get_imu_yaw();
        float angle_error = corner_target_angle - current_yaw;

        // 瑙掑害褰掍竴鍖栧埌 [-180, 180]
        if (angle_error > 180.0f) angle_error -= 360.0f;
        if (angle_error < -180.0f) angle_error += 360.0f;

        // 瑙掑害澶栫幆锛堟瘮渚挎帶鍒讹級
        const float ANGLE_P = 8.0f;
        const float MAX_ANGULAR_SPEED = 350.0f;
        float target_angular_speed = angle_error * ANGLE_P;
        if (target_angular_speed > MAX_ANGULAR_SPEED)
            target_angular_speed = MAX_ANGULAR_SPEED;
        if (target_angular_speed < -MAX_ANGULAR_SPEED)
            target_angular_speed = -MAX_ANGULAR_SPEED;
        target_gyro = target_angular_speed;

        // ========== 閫�鍑烘潯浠讹細瑙掑害璇樊杈炬爣 鎴� 瓒呮椂寮哄埗閫�鍑� ==========
        const float EXIT_ANGLE_THRESH = 15.0f; // 瑙掑害璇樊闃堝�硷紙搴︼級
        const uint8_t EXIT_FRAMES = 1;        // 闇�瑕佽繛缁弧瓒崇殑甯ф暟
        const uint16_t MAX_TURN_TIMEOUT = 200; // 鏈�澶ц浆寮抚鏁帮紙绾�3-6绉掞級

        if (fabsf(angle_error) < EXIT_ANGLE_THRESH || turn_timeout >= MAX_TURN_TIMEOUT) {
            exit_cnt++;
            if (exit_cnt >= EXIT_FRAMES) {
                // 閫�鍑鸿浆寮ā寮�
                corner_turning_active = 0;
                // 澶嶄綅浣嶇疆鐜紝闃叉鍒囧洖寰嚎鏃剁獊鍙�
                place_pid.last_error = 0;
                place_pid.integral = 0;
                target_gyro = 0.0f;
                exit_cnt = 0;
                turn_timeout = 0;
            }
        } else {
            // 涓�鏃﹁宸繃澶э紝绔嬪澶嶄綅璁℃暟鍣�
            exit_cnt = 0;
        }
    }
    else
    {
        // ===== 正常循线模式 =====
        int16_t pixel_error = get_camera_error();
        place_pid_calculate(pixel_error);
        target_gyro = get_place_target_gyro();
    }
}
    // 璋冭瘯杈撳嚭
    // printf("%d\n", pixel_error);

/*void Control3(void)
{
    // 1. 妫�鏌ラ檧铻轰华鏄惁鏍″噯瀹屾垚
                        if (!gyro_calibrated)
                        {
                            // 鐢ㄥ浐瀹氫綆閫熺洿琛�
                            Motor_Control(500, 500);

                        }
           // 2. 鑾峰彇鎽勫儚澶磋宸紙鍍忕礌锛�
        int16_t pixel_error =get_camera_error();
       oscilloscope_data[0]=pixel_error;
        // 3. 浣嶇疆鐜疨ID璁＄畻锛堝鐜級锛氬儚绱犺宸� -> 鐩爣瑙掗�熷害锛坉eg/s锛�
        place_pid_calculate(pixel_error);
        printf("%d\n",pixel_error);
         target_gyro = get_place_target_gyro();
         if(node_index==20)
         {
             Motor_Control(0,0);
         }
}*/
// ========== 涓诲嚱鏁� ==========
int core0_main(void)
{
    clock_init();                   // 鑾峰彇鏃堕挓棰戠巼
    debug_init();                   // 鍒濆鍖栬皟璇曚覆鍙�
    // 姝ゅ缂栧啓鐢ㄦ埛浠ｇ爜 渚嬪澶栬鍒濆鍖栦唬鐮佺瓑
       while(1)
       {
           if(mt9v03x_init())
               gpio_toggle_level(LED1);                                            // 缈昏浆 LED 寮曡剼杈撳嚭鐢靛钩 鎺у埗 LED 浜伃 鍒濆鍖栧嚭閿欒繖涓伅浼氶棯鐨勫緢鎱�
           else
               break;
           system_delay_ms(500);                                                   // 闂伅琛ㄧず寮傚父
       }

    //鏄剧ず灞忓垵濮嬪寲
      ips114_init();                  // 鍒濆鍖朓PS114灞忓箷
       ips114_set_dir(0); // 璁剧疆涓烘í灞忔樉绀猴紙鏍规嵁鎮ㄧ殑闇�姹傞�夋嫨锛�   娌￠棶棰�
       ips114_set_color(RGB565_RED, RGB565_BLACK); // 璁剧疆鐢荤瑪棰滆壊鍜岃儗鏅壊
       init_track_selector();
       //ips114_clear();    // 娓呭睆
  // 初始化无线串口（逐飞无线转串口，115200波特率，关自动波特率）
      wireless_uart_init();
      // RTS 拉低：模块没有接 P10_2 时，让发送函数认为模块一直就绪
      gpio_init(P10_2, GPO, GPIO_LOW, GPO_PUSH_PULL);
   system_init();
   pid_init_all();

  // seekfree_assistant_interface_init(UART_2);
  // seekfree_assistant_oscilloscope_struct osc;
    //  osc.channel_num = 4;
   // 姝ゅ缂栧啓鐢ㄦ埛浠ｇ爜 渚嬪澶栬鍒濆鍖栦唬鐮佺瓑
       cpu_wait_event_ready();         // 绛夊緟鎵�鏈夋牳蹇冨垵濮嬪寲瀹屾瘯
       /*while (TRUE)
       {

          if(mt9v03x_finish_flag)
          {
               mt9v03x_finish_flag = 0;
               check_track_switch();
               image_process();
               Control3();
          }*/
       while (TRUE)
       {
           if(mt9v03x_finish_flag)
           {
               mt9v03x_finish_flag = 0;

               check_key();              // 涓�涓嚱鏁板鐞嗘墍鏈夋寜閿�

               image_process();
               Control3();
               // 运行时：根据转弯强度调节风扇负压
               if (is_motor_running()) {
                   fan_control_update(corner_turning_active, fabsf(target_gyro));
               }
               printf("%d\n",encoder_get_count(TIM6_ENCODER));
           }
       }
          /*if(node_index>=20)
                  {
                      Motor_Control(0,0);
                  }*/
         /*  osc.data[0] = oscilloscope_data[0];
           osc.data[1] = oscilloscope_data[1];
           osc.data[2] = oscilloscope_data[2];
           osc.data[3] = oscilloscope_data[3];
            // 閫氳繃鏃犵嚎涓插彛鍙戦�佽櫄鎷熺ず娉㈠櫒鏁版嵁
           seekfree_assistant_oscilloscope_send(&osc);*/
       }

#pragma section all restore*/
