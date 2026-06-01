/*
 * img.h - ͼ����ͷ�ļ�������·���
 *
 * ���ܣ���ֵ����ѭ�ߡ��յ��⡢ת��滮
 *//*

#ifndef CODE_IMG_H_
#define CODE_IMG_H_


#include "zf_common_headfile.h"
#include "zf_device_ips114.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_mpu6050.h"

// ==================== ͼ��ߴ� ====================
#define image_h             MT9V03X_H
#define image_w             MT9V03X_W

// ==================== ��ɫ���� ====================
#define COLOR_RED           0xF800
#define COLOR_GREEN         0x07E0
#define COLOR_BLUE          0x001F
#define COLOR_PURPLE        0xF81F
#define COLOR_YELLOW        0xFFE0
#define COLOR_WHITE         0xFFFF
#define COLOR_BLACK         0x0000

#define white_pixel         255
#define black_pixel         0

// ==================== �߽���� ====================
#define border_min          0
#define border_max          (image_w - 1)

// ==================== ѭ�߲��� ====================
#define SEARCH_OFFSET       12
#define SEARCH_OFFSET_EXPAND 40

// ����·���ר�ò�����ϸ������
#define MAX_SEARCH_WIDTH    80          // ��ͨ������������ȣ�ϸ������Ҫ����Χ��
#define BOTTOM_FULL_SEARCH  1           // �ײ�����ȫ��Χ������1=���ã�0=���ã�
#define MAX_SEARCH_ITER     120         // ����������������������Է���һ��
#define MIN_VALID_WHITE     5           // ��С��Ч��ɫ��������ϸ��������ֻ�м����׵㣩

// ==================== ��ֵ������ ====================
#define FIXED_THRESHOLD     100
#define MIN_THRESHOLD       70
#define MAX_THRESHOLD       180
#define TRACK_WHITE_RATIO   20
#define MIN_VALID_WHITE_RATIO 3         // ��С��Ч��ɫ������%����ϸ��������Ҫ��


// ==================== �յ������ ====================
#define CORNER_ANGLE_THRESH 35          // �յ��ж���ֵ
#define CORNER_ROW_RANGE    0          // �յ���������Χ
#define CORNER_HOLD_FRAMES  1           // �յ㱣��֡������������

// �յ������
#define CORNER_ANGLE_THRESH     12   // �߽�ͻ����ֵ�����أ�
#define CORNER_SCAN_RANGE       10      // �ӵײ�����ɨ���������ͼ���°벿�֣�
#define CORNER_HOLD_FRAMES      1       // ����֡��
#define CORNER_ROW_TOLERANCE    8       // �յ��к����̶ȣ���������
// ==================== ת������ ====================
typedef enum {
    TURN_LEFT=0,
    TURN_RIGHT=1,
    TURN_STRAIGHT=2,
} turn_dir_t;

// �յ�ṹ�壨�򻯰棩
typedef struct {
    uint8 detected;                      // �Ƿ��⵽�յ�
    uint8 row;                           // �յ�������
    uint8 col;                           // �յ�������
    uint8 turn_dir;                      // ת���򣨴ӹյ������ȡ��
} corner_point_t;



// ==================== ȫ�ֱ������� ====================
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];
extern uint16 l_border[image_h];
extern uint16 r_border[image_h];
extern uint16 center_line[image_h];

extern corner_point_t detected_corner;   // ��ǰ��⵽�Ĺյ�
extern int16_t corner_error_bias;
// ת��滮���飨�յ��Ӧ��ת����
extern const turn_dir_t corner_turn_plan[20];
extern uint8 corner_index;               // ��ǰ�������ڼ����յ�

// ״̬��־
extern uint8 cross_detection_enable;
// ==================== �������л� ====================
#define TRACK_COUNT         3

#define TRACK_1             0
#define TRACK_2             1
#define TRACK_3             2

// ���򿪹����Ŷ���
#define KEY_UP_PIN          P33_12
#define KEY_DOWN_PIN        P20_7
#define KEY_LEFT_PIN        P20_6
#define KEY_RIGHT_PIN       P11_3
#define KEY_ENTER_PIN       P11_2

// �ⲿ�ӿ�
extern uint8 current_track;
void init_track_selector(void);
void check_track_switch(void);
void switch_to_track(uint8 track_idx);
uint8 get_current_track(void);

// ==================== �������� ====================
int my_abs(int value);
int16 limit_a_b(int16 x, int a, int b);
void Get_image(uint8(*mt9v03x_image)[MT9V03X_W]);
void turn_to_bin_fixed(void);
void turn_to_bin_track(void);
void turn_to_bin(void);
void image_filter_simple(void);
void find_left_right_boundary(void);
void calculate_center_line(void);
void detect_corner(void);
int image_process(void);
void draw_boundary_and_center(void);
void draw_corner_marker(void);

// �ⲿ���ƽӿ�
uint8 get_corner_detected(void);
uint8 get_corner_turn_dir(void);
uint8 get_corner_row(void);
uint8 get_corner_col(void);
void reset_corner_state(void);
// �� img.h ĩβ����
// �滻ԭ��������
void check_key(void);
uint8_t is_motor_running(void);
// ɾ�� check_track_switch �� check_motor_start ������
#endif
*/
/*
 * img.h - ͼ����ģ��ͷ�ļ������ϰ棬�������Ǻ�����
 *
 * �����˶�ֵ�����߽���ȡ�������߼��㡢�յ�����ת����ƵȽӿ�
 * �����ڶ������л�������·���
 */

#ifndef CODE_IMG_H_
#define CODE_IMG_H_

#include "zf_common_headfile.h"
#include "zf_device_ips114.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_mpu6050.h"

// ==================== ͼ��ߴ� ====================
#define image_h             MT9V03X_H
#define image_w             MT9V03X_W

// ==================== ��ɫ���� ====================
#define COLOR_RED           0xF800
#define COLOR_GREEN         0x07E0
#define COLOR_BLUE          0x001F
#define COLOR_PURPLE        0xF81F
#define COLOR_YELLOW        0xFFE0
#define COLOR_WHITE         0xFFFF
#define COLOR_BLACK         0x0000
#define COLOR_CYAN          0x07FF

#define white_pixel         255
#define black_pixel         0

// ==================== �߽�������Χ ====================
#define border_min          0
#define border_max          (image_w - 1)

// ==================== ����������������ϸ������ ====================
#define SEARCH_OFFSET           12
#define SEARCH_OFFSET_EXPAND    40
#define MAX_SEARCH_WIDTH        80
#define MAX_SEARCH_ITER         120
#define MIN_VALID_WHITE         8

// ==================== ��ֵ����ֵ���� ====================
#define FIXED_THRESHOLD         100
#define MIN_THRESHOLD           70
#define MAX_THRESHOLD           180
#define TRACK_WHITE_RATIO       20
#define MIN_VALID_WHITE_RATIO   3

// ==================== �յ������ ====================
#define CORNER_ANGLE_THRESH     12
#define CORNER_HOLD_FRAMES      1
#define CORNER_ROW_TOLERANCE    8       // �к����̶�
#define CORNER_ROW_RANGE        0
#define NORMAL_START_ROW        8
#define TRIPOD_START_ROW        60
#define TRIPOD_NODE_INDEX       6

// ��չ���������ã�
#define WHITE_SURGE_THRESHOLD   55

// ==================== ת�䷽��ö�� ====================
typedef enum {
    TURN_LEFT = 0,
    TURN_RIGHT = 1,
    TURN_STRAIGHT = 2,
} turn_dir_t;

// �յ���Ϣ�ṹ��
typedef struct {
    uint8 detected;
    uint8 row;
    uint8 col;
    uint8 turn_dir;
} corner_point_t;

// ==================== ȫ�ֱ������� ====================
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];
extern uint16 l_border[image_h];
extern uint16 r_border[image_h];
extern uint16 center_line[image_h];

extern corner_point_t detected_corner;
extern uint8 corner_index;
extern int16_t corner_error_bias;
extern float corner_target_gyro;
extern uint8_t corner_gyro_active;
extern uint8_t corner_gyro_timer;
extern uint8 current_track;
// ֱ��ת��ջ����Ʊ�����������
extern uint8_t corner_turning_active;   // �Ƿ�����ִ��ֱ��ת��
extern float   corner_target_angle;     // Ŀ��ƫ���ǣ��ȣ�
extern float   corner_start_angle;      // ��ʼƫ���ǣ������ã�
// ==================== ����ѡ����غ� ====================
#define TRACK_COUNT     3
#define TRACK_1         0
#define TRACK_2         1
#define TRACK_3         2

// �������Ŷ���
#define KEY_UP_PIN      P33_12
#define KEY_DOWN_PIN    P20_7
#define KEY_LEFT_PIN    P20_6
#define KEY_RIGHT_PIN   P11_3
#define KEY_ENTER_PIN   P11_2

// ==================== �������� ====================
int image_process(void);

void Get_image(uint8(*mt9v03x_image)[MT9V03X_W]);
void turn_to_bin(void);
void turn_to_bin_fixed(void);
void turn_to_bin_track(void);
void image_filter_simple(void);
void find_left_right_boundary(void);
void calculate_center_line(void);
void detect_corner(void);
void handle_corner_turn(void);
void reset_corner_state(void);
void reset_corner_turn_state(void);

uint8 get_corner_detected(void);
uint8 get_corner_turn_dir(void);
uint8 get_corner_row(void);
uint8 get_corner_col(void);

void draw_boundary_and_center(void);
void draw_corner_marker(void);
void display_track_info(void);
void display_white_pixel_count(void);

void init_track_selector(void);
void check_key(void);
void switch_to_track(uint8 track_idx);
uint8_t is_motor_running(void);
void motor_start(void);

// ���ߺ���
int my_abs(int value);
int16 limit_a_b(int16 x, int a, int b);

#endif /* CODE_IMG_H_ */
