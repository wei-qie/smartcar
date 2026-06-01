/**
 * img.c - 鍥惧儚澶勭悊妯″潡锛堟櫤鑳借溅璧涢亾妫�娴嬶級
 *
 * 鍔熻兘锛氫簩鍊煎寲銆佸惊绾裤�佹嫄鐐规娴嬨�佽浆鍚戞帶鍒�
 */

#include "img.h"
#include "motor.h"
#include <string.h>

// ==================== 鍏ㄥ眬鍙橀噺瀹氫箟 ====================
uint8 original_image[image_h][image_w];
uint8 bin_image[image_h][image_w];

uint16 l_border[image_h];
uint16 r_border[image_h];
uint16 center_line[image_h];

corner_point_t detected_corner;
uint8 corner_index = 0;
float corner_target_gyro = 0.0f;      // 鎷愮偣鐩爣瑙掗�熷害锛堝害/绉掞級
uint8_t corner_gyro_active = 0;        // 鎷愮偣闄�铻轰华杞悜婵�娲绘爣蹇�
uint8_t corner_gyro_timer = 0;         // 鎷愮偣闄�铻轰华杞悜璁℃椂鍣�

// ==================== 璧涢亾杞悜瑙勫垝琛� ====================
// 璧涢亾1 - 杞悜瑙勫垝
/*static const turn_dir_t track1_turn_plan[30] = {
    TURN_RIGHT,TURN_RIGHT,TURN_STRAIGHT , TURN_LEFT,
    TURN_LEFT, TURN_RIGHT, TURN_RIGHT,TURN_STRAIGHT,
    TURN_RIGHT,TURN_LEFT, TURN_LEFT,TURN_LEFT, TURN_STRAIGHT, TURN_STRAIGHT,
    TURN_STRAIGHT, TURN_RIGHT, TURN_STRAIGHT, TURN_RIGHT,
    TURN_STRAIGHT, TURN_RIGHT,TURN_STRAIGHT,TURN_STRAIGHT,TURN_RIGHT
};*/
static const turn_dir_t track1_turn_plan[30] = {
        TURN_RIGHT,TURN_RIGHT,TURN_LEFT,TURN_LEFT, TURN_RIGHT, TURN_RIGHT,TURN_RIGHT,
        TURN_LEFT,TURN_RIGHT,TURN_LEFT,TURN_LEFT,TURN_LEFT,TURN_LEFT,TURN_LEFT,TURN_RIGHT,TURN_STRAIGHT,TURN_STRAIGHT
};
// 璧涢亾2 - 杞悜瑙勫垝
static const turn_dir_t track2_turn_plan[25] = {
    TURN_RIGHT, TURN_STRAIGHT, TURN_RIGHT, TURN_STRAIGHT, TURN_LEFT,
    TURN_LEFT, TURN_STRAIGHT, TURN_RIGHT, TURN_RIGHT, TURN_STRAIGHT,
    TURN_RIGHT, TURN_STRAIGHT, TURN_STRAIGHT, TURN_RIGHT, TURN_STRAIGHT
};

// 璧涢亾3 - 杞悜瑙勫垝
static const turn_dir_t track3_turn_plan[25] = {
    TURN_STRAIGHT,TURN_RIGHT,TURN_RIGHT,TURN_LEFT,TURN_LEFT,TURN_RIGHT,TURN_RIGHT,TURN_RIGHT,TURN_LEFT,TURN_LEFT,TURN_STRAIGHT,
    TURN_STRAIGHT, TURN_STRAIGHT, TURN_STRAIGHT,TURN_STRAIGHT,TURN_STRAIGHT,TURN_STRAIGHT,TURN_STRAIGHT
};

static const turn_dir_t* active_turn_plan = track1_turn_plan;
uint8 current_track = TRACK_2;

static uint32_t last_key_time = 0;
static uint8_t last_key_value = 0;

// 榛樿鎷愮偣杞悜瑙勫垝锛堝鐢級
const turn_dir_t corner_turn_plan[20] = {
    TURN_RIGHT, TURN_STRAIGHT, TURN_STRAIGHT, TURN_RIGHT, TURN_STRAIGHT,
    TURN_RIGHT, TURN_STRAIGHT, TURN_STRAIGHT, TURN_RIGHT, TURN_STRAIGHT,
    TURN_STRAIGHT, TURN_STRAIGHT, TURN_STRAIGHT, TURN_STRAIGHT,
    TURN_STRAIGHT, TURN_STRAIGHT, TURN_STRAIGHT, TURN_STRAIGHT,
    TURN_STRAIGHT, TURN_STRAIGHT
};

uint8 cross_detection_enable = 1;

static uint8 temp_image[image_h][image_w];

static uint8 corner_hold_counter = 0;
static uint8 last_corner_detected = 0;
static uint8 last_corner_row = 0;
uint8_t corner_turning_active = 0;
float corner_target_angle = 0.0f;
float corner_start_angle = 0.0f;
// ==================== 鎷愮偣杞悜鐘舵�佸彉閲� ====================
uint8_t corner_active = 0;           // 鎷愮偣杞悜婵�娲绘爣蹇�
uint8_t corner_turn_timer = 0;       // 鎷愮偣杞悜璁℃椂鍣�
uint8_t corner_active_flag = 0;      // 鎷愮偣婵�娲绘爣蹇楋紙鐢ㄤ簬闃查噸澶嶈Е鍙戯級
uint8_t corner_turn_dir = TURN_STRAIGHT;  // 鎷愮偣杞悜鏂瑰悜
uint8_t corner_target_row = 0;       // 鎷愮偣鐩爣琛�
int16_t corner_error_bias = 0;       // 鎷愮偣杞悜璇樊鍋忕疆

// ==================== 椋庢墖棰勮繍琛屾帶鍒� ====================
uint8_t fan_prerun_active = 0;      // 椋庢墖棰勮繍琛屾爣蹇楋紙绛夊緟璐熷帇寤虹珛锛�
uint16_t fan_prerun_counter = 0;    // 椋庢墖棰勮繍琛岃鏁板櫒锛�10ms姝ヨ繘锛�

// ==================== 宸ュ叿鍑芥暟 ====================
int my_abs(int value)
{
    return (value >= 0) ? value : -value;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if(x < a) x = a;
    if(x > b) x = b;
    return x;
}

// ==================== 鎷愮偣鐘舵�佽幏鍙栧嚱鏁� ====================
uint8 get_corner_detected(void)
{
    return detected_corner.detected;
}

uint8 get_corner_turn_dir(void)
{
    return detected_corner.turn_dir;
}

uint8 get_corner_row(void)
{
    return detected_corner.row;
}

uint8 get_corner_col(void)
{
    return detected_corner.col;
}

void reset_corner_state(void)
{
    detected_corner.detected = 0;
    corner_hold_counter = 0;
    last_corner_detected = 0;
    last_corner_row = 0;
}

// ==================== 鍥惧儚閲囬泦 ====================
void Get_image(uint8(*mt9v03x_image)[MT9V03X_W])
{
    uint16 i, j;
    for(i = 0; i < image_h; i++)
    {
        for(j = 0; j < image_w; j++)
        {
            original_image[i][j] = mt9v03x_image[i][j];
        }
    }
}

// ==================== 浜屽�煎寲鍑芥暟 ====================
void turn_to_bin_fixed(void)
{
    uint8 i, j;
    uint8 threshold = FIXED_THRESHOLD;

    for (i = 0; i < image_h; i++)
    {
        for (j = 0; j < image_w; j++)
        {
            bin_image[i][j] = (original_image[i][j] > threshold) ? white_pixel : black_pixel;
        }
    }
}

void image_filter_simple(void)
{
    uint8 i, j;

    memcpy(temp_image, bin_image, sizeof(bin_image));

    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < image_w - 1; j++)
        {
            if (temp_image[i][j] == white_pixel)
            {
                uint8 white_count = 0;
                int8 di, dj;
                for (di = -1; di <= 1; di++)
                {
                    for (dj = -1; dj <= 1; dj++)
                    {
                        if (di == 0 && dj == 0) continue;
                        if (temp_image[i+di][j+dj] == white_pixel)
                        {
                            white_count++;
                        }
                    }
                }
                if (white_count < 2)
                {
                    bin_image[i][j] = black_pixel;
                }
            }
            else if (temp_image[i][j] == black_pixel)
            {
                uint8 black_count = 0;
                int8 di, dj;
                for (di = -1; di <= 1; di++)
                {
                    for (dj = -1; dj <= 1; dj++)
                    {
                        if (di == 0 && dj == 0) continue;
                        if (temp_image[i+di][j+dj] == black_pixel)
                        {
                            black_count++;
                        }
                    }
                }
                if (black_count < 2)
                {
                    bin_image[i][j] = white_pixel;
                }
            }
        }
    }
}

static void calculate_histogram(uint8 *image, uint32 *hist)
{
    uint16 i, j;

    for (i = 0; i < 256; i++)
    {
        hist[i] = 0;
    }

    for (i = 0; i < image_h; i++)
    {
        for (j = 0; j < image_w; j++)
        {
            hist[image[i * image_w + j]]++;
        }
    }
}

static uint8 find_adaptive_threshold(void)
{
    uint32 hist[256];
    uint32 total_pixels = (uint32)image_h * (uint32)image_w;
    uint32 target_white = total_pixels * TRACK_WHITE_RATIO / 100;
    uint32 white_count = 0;
    int i;

    calculate_histogram(original_image[0], hist);

    for (i = 255; i > 0; i--)
    {
        white_count += hist[i];
        if (white_count >= target_white)
        {
            return (uint8)i;
        }
    }

    return FIXED_THRESHOLD;
}

void turn_to_bin_track(void)
{
    uint8 i, j;
    uint8 threshold = find_adaptive_threshold() + 50;
    uint32 white_count = 0;
    uint32 total_pixels = image_h * image_w;
    uint32 white_ratio;

    if (threshold < MIN_THRESHOLD) threshold = MIN_THRESHOLD;
    if (threshold > MAX_THRESHOLD) threshold = MAX_THRESHOLD;

    for (i = 0; i < image_h; i++)
    {
        for (j = 0; j < image_w; j++)
        {
            bin_image[i][j] = (original_image[i][j] > threshold) ? white_pixel : black_pixel;
            if(bin_image[i][j] == white_pixel) white_count++;
        }
    }

    white_ratio = white_count * 100 / total_pixels;

    // 鐧界偣姣斾緥鏈夋晥鎬ф鏌�
    if(white_ratio < MIN_VALID_WHITE_RATIO)
    {
        return;
    }

    image_filter_simple();
}

void turn_to_bin(void)
{
    turn_to_bin_track();
}

// ==================== 杈圭晫妫�娴嬪嚱鏁� ====================
static inline uint8 is_left_boundary(uint8 *row, uint16 x)
{
    return (x + 2 < image_w) && (row[x] == black_pixel && row[x+1] == white_pixel && row[x+2] == white_pixel);
}

static inline uint8 is_right_boundary(uint8 *row, uint16 x)
{
    return (x >= 2) && (row[x] == black_pixel && row[x-1] == white_pixel && row[x-2] == white_pixel);
}

// 鍏ㄨ寖鍥存悳绱㈠乏杈圭晫
static uint8 search_left_boundary_full_range(uint8 *row, uint16 *found_col)
{
    uint16 x;
    uint16 search_count = 0;
    uint16 max_search_count = border_max - border_min + 1;

    for(x = border_min; x <= border_max; x++)
    {
        search_count++;
        if(search_count > max_search_count) break;

        if(is_left_boundary(row, x))
        {
            *found_col = x;
            return 1;
        }
    }
    return 0;
}

// 鍏ㄨ寖鍥存悳绱㈠彸杈圭晫
static uint8 search_right_boundary_full_range(uint8 *row, uint16 *found_col)
{
    int16 x;
    uint16 search_count = 0;
    uint16 max_search_count = border_max - border_min + 1;

    for(x = (int16)border_max; x >= (int16)border_min; x--)
    {
        search_count++;
        if(search_count > max_search_count) break;

        if(is_right_boundary(row, (uint16)x))
        {
            *found_col = (uint16)x;
            return 1;
        }
    }
    return 0;
}

// 闄愬畾鑼冨洿鎼滅储宸﹁竟鐣�
static uint8 search_left_boundary_limited(uint8 *row, uint16 start_col, uint16 *found_col)
{
    uint16 x;
    uint16 search_count = 0;

    if(start_col < border_min) start_col = border_min;
    if(start_col > border_max) start_col = border_max;

    uint16 max_search = start_col + MAX_SEARCH_WIDTH;
    if(max_search > border_max) max_search = border_max;

    for(x = start_col; x <= max_search; x++)
    {
        search_count++;
        if(search_count > MAX_SEARCH_ITER) break;

        if(is_left_boundary(row, x))
        {
            *found_col = x;
            return 1;
        }
    }
    return 0;
}

// 闄愬畾鑼冨洿鎼滅储鍙宠竟鐣�
static uint8 search_right_boundary_limited(uint8 *row, uint16 start_col, uint16 *found_col)
{
    int16 x;
    uint16 search_count = 0;

    if(start_col > border_max) start_col = border_max;
    if(start_col < border_min) start_col = border_min;

    uint16 min_search = (start_col > MAX_SEARCH_WIDTH) ? (start_col - MAX_SEARCH_WIDTH) : border_min;

    for(x = (int16)start_col; x >= (int16)min_search; x--)
    {
        search_count++;
        if(search_count > MAX_SEARCH_ITER) break;

        if(is_right_boundary(row, (uint16)x))
        {
            *found_col = (uint16)x;
            return 1;
        }
    }
    return 0;
}

// 蹇�熺粺璁′竴琛屼腑鐧界偣鏁伴噺锛堜粎鍦ㄤ腑蹇冨尯鍩熼檮杩戠粺璁★級
static uint16 count_white_pixels_in_row_quick(uint8 *row)
{
    uint16 count = 0;
    uint16 center = image_w / 2;
    uint16 start = (center > MAX_SEARCH_WIDTH) ? (center - MAX_SEARCH_WIDTH) : border_min;
    uint16 end = (center + MAX_SEARCH_WIDTH < border_max) ? (center + MAX_SEARCH_WIDTH) : border_max;

    for(uint16 x = start; x <= end; x++)
    {
        if(row[x] == white_pixel) count++;
    }
    return count;
}

// 涓�兼护娉㈠钩婊戣竟鐣�
void smooth_boundary_median(void)
{
    uint16 y;
    uint16 temp_l[image_h], temp_r[image_h];

    // 鎷疯礉鍘熷鏁版嵁
    for(y = 0; y < image_h; y++)
    {
        temp_l[y] = l_border[y];
        temp_r[y] = r_border[y];
    }

    // 3鐐逛腑鍊兼护娉�
    for(y = 1; y < image_h - 1; y++)
    {
        // 宸﹁竟鐣屼腑鍊兼护娉�
        uint16 a = temp_l[y-1], b = temp_l[y], c = temp_l[y+1];
        if((a <= b && b <= c) || (c <= b && b <= a))
            l_border[y] = b;
        else if((b <= a && a <= c) || (c <= a && a <= b))
            l_border[y] = a;
        else
            l_border[y] = c;

        // 鍙宠竟鐣屼腑鍊兼护娉�
        a = temp_r[y-1]; b = temp_r[y]; c = temp_r[y+1];
        if((a <= b && b <= c) || (c <= b && b <= a))
            r_border[y] = b;
        else if((b <= a && a <= c) || (c <= a && a <= b))
            r_border[y] = a;
        else
            r_border[y] = c;
    }
}

// ==================== 瀵绘壘宸﹀彸杈圭晫 ====================
void find_left_right_boundary(void)
{
    int16 y;
    uint16 last_left = image_w / 2;
    uint16 last_right = image_w / 2;
    uint16 offset = SEARCH_OFFSET;
    uint8 boundary_valid = 0;
    uint16 valid_rows_count = 0;
    uint8 consecutive_no_boundary = 0;

    // 鍒濆鍖栬竟鐣�
    for(y = 0; y < (int16)image_h; y++)
    {
        l_border[y] = image_w / 2;
        r_border[y] = image_w / 2;
    }

    for(y = (int16)image_h - 1; y >= 0; y--)
    {
        if(y < 0 || y >= (int16)image_h) break;

        uint8 *row = bin_image[y];
        uint16 left = 0, right = 0;
        uint8 left_found = 0, right_found = 0;

        // 缁熻褰撳墠琛岀櫧鐐规暟閲�
        uint16 white_count = count_white_pixels_in_row_quick(row);

        // 鐧界偣澶皯锛岃涓烘棤鏁堣
        if(white_count < MIN_VALID_WHITE)
        {
            consecutive_no_boundary++;
            l_border[y] = border_min;
            r_border[y] = border_max;

            // 杩炵画鏃犳晥琛屽お澶氾紝閲嶇疆杩借釜浣嶇疆
            if(consecutive_no_boundary > 20)
            {
                last_left = image_w / 2;
                last_right = image_w / 2;
                consecutive_no_boundary = 0;
            }
            continue;
        }

        consecutive_no_boundary = 0;
        valid_rows_count++;
        if(valid_rows_count > 5) boundary_valid = 1;

        // ==================== 搴曢儴涓夎鍏ㄨ寖鍥存悳绱� ====================
        if(y == (int16)image_h - 1 || y == (int16)image_h - 2 || y == (int16)image_h - 3)
        {
            left_found = search_left_boundary_full_range(row, &left);
            right_found = search_right_boundary_full_range(row, &right);

            if(y == (int16)image_h - 1)
            {
                if(left_found) last_left = left;
                if(right_found) last_right = right;
            }

            if(left_found)
                l_border[y] = left;
            else
                l_border[y] = border_min;

            if(right_found)
                r_border[y] = right;
            else
                r_border[y] = border_max;
        }
        // ==================== 鍏朵粬琛岄檺瀹氳寖鍥存悳绱� ====================
        else
        {
            // ----- 鎼滅储宸﹁竟鐣� -----
            uint16 left_start;
            if(last_left > offset) left_start = last_left - offset;
            else left_start = border_min;
            if(left_start > border_max) left_start = border_max;
            if(left_start < border_min) left_start = border_min;

            left_found = search_left_boundary_limited(row, left_start, &left);

            // 鎵╁ぇ鎼滅储鑼冨洿
            if(!left_found && last_left > SEARCH_OFFSET_EXPAND)
            {
                uint16 left_start_expand = (last_left > SEARCH_OFFSET_EXPAND) ?
                                          (last_left - SEARCH_OFFSET_EXPAND) : border_min;
                if(left_start_expand > border_max) left_start_expand = border_max;
                left_found = search_left_boundary_limited(row, left_start_expand, &left);
            }

            // 浠嶆湭鎵惧埌锛屽叏鑼冨洿鎼滅储
            if(!left_found)
            {
                left_found = search_left_boundary_full_range(row, &left);
            }

            // ----- 鎼滅储鍙宠竟鐣� -----
            uint16 right_start;
            if(last_right + offset < image_w) right_start = last_right + offset;
            else right_start = border_max;
            if(right_start > border_max) right_start = border_max;
            if(right_start < border_min) right_start = border_min;

            right_found = search_right_boundary_limited(row, right_start, &right);

            // 鎵╁ぇ鎼滅储鑼冨洿
            if(!right_found && last_right + SEARCH_OFFSET_EXPAND < image_w)
            {
                uint16 right_start_expand = (last_right + SEARCH_OFFSET_EXPAND < border_max) ?
                                           (last_right + SEARCH_OFFSET_EXPAND) : border_max;
                if(right_start_expand < border_min) right_start_expand = border_min;
                right_found = search_right_boundary_limited(row, right_start_expand, &right);
            }

            // 浠嶆湭鎵惧埌锛屽叏鑼冨洿鎼滅储
            if(!right_found)
            {
                right_found = search_right_boundary_full_range(row, &right);
            }

            // 鏇存柊杈圭晫鍊煎拰杩借釜浣嶇疆
            if(left_found)
            {
                l_border[y] = left;
                last_left = left;
            }
            else
            {
                l_border[y] = border_min;
            }

            if(right_found)
            {
                r_border[y] = right;
                last_right = right;
            }
            else
            {
                r_border[y] = border_max;
            }
        }

        // ==================== 杈圭晫鏈夋晥鎬ф鏌� ====================
        // 宸﹀彸杈圭晫浜ゅ弶鏃堕噸缃�
        if(l_border[y] >= r_border[y])
        {
            l_border[y] = border_min;
            r_border[y] = border_max;
        }

        // 杈圭晫鍊艰寖鍥撮檺鍒�
        if(l_border[y] < border_min) l_border[y] = border_min;
        if(l_border[y] > border_max) l_border[y] = border_max;
        if(r_border[y] < border_min) r_border[y] = border_min;
        if(r_border[y] > border_max) r_border[y] = border_max;
    }
    smooth_boundary_median();
}

void calculate_center_line(void)
{
    uint16 y;
    for(y = 0; y < image_h; y++)
    {
        center_line[y] = (l_border[y] + r_border[y]) / 2;
    }
}

// ==================== 鐧界偣鏁伴噺缁熻 ====================
static uint16 count_white_pixels_in_row(uint8 row)
{
    uint16 count = 0;
    uint16 x;

    if(row >= image_h) return 0;

    for(x = 0; x < image_w; x++)
    {
        if(bin_image[row][x] == white_pixel) count++;
    }
    return count;
}

// ==================== 鎷愮偣妫�娴嬶紙鍩轰簬涓績绾跨獊鍙橈級====================
#define TRIPOD_NODE_INDEX   6
#define NORMAL_START_ROW    8
#define TRIPOD_START_ROW    60

/*
void detect_corner(void)
{
    int16 y;
    int16 center_diff;                     // 涓績绾垮彉鍖栭噺
    int16 start_row = NORMAL_START_ROW;
    int16 end_row = (int16)image_h - 1;
    uint16 loop_count = 0;
    uint16 max_loop = (end_row - start_row) * 2;
    uint8 corner_found = 0;
    uint8 corner_row = 0;
    uint8 corner_col = 0;

    if(start_row < 0) start_row = 0;
    if(end_row >= (int16)image_h) end_row = (int16)image_h - 1;

    for(y = end_row; y >= start_row; y--)
    {
        loop_count++;
        if(loop_count > max_loop) break;
        if(y <= 0 || y >= (int16)image_h) continue;
        if(y == 0) continue;

        // 璁＄畻涓績绾跨殑鍙樺寲閲忥紙鍚戜笂鐩搁偦琛岀殑宸�硷級
        center_diff = (int16)center_line[y] - (int16)center_line[y-1];

        // 涓婁竴琛岀殑鐧界偣鏁伴噺锛堣緟鍔╁垽鏂級
        uint16 white_count_prev = count_white_pixels_in_row((uint8)(y-1));
        uint16 white_count_curr = count_white_pixels_in_row((uint8)y);
        // 鍒ゆ柇鏉′欢锛氫腑蹇冪嚎绐佸彉 + 鐧界偣鏁伴噺鏆村锛堝弻閲嶆娴嬶級
        if((my_abs(center_diff) > CORNER_ANGLE_THRESH))
        {
            corner_found = 1;
            corner_row = (uint8)y;
            // 涓績绾跨獊鍙樻椂锛屾嫄鐐瑰垪鍙栦腑蹇冪嚎鐨勪綅缃�
            corner_col = (uint8)center_line[y];
            break;
        }
    }

    // 甯ч棿纭涓庨槻閲嶅瑙﹀彂閫昏緫
    if(corner_found)
    {
        int16 row_diff = (int16)corner_row - (int16)last_corner_row;
        if(row_diff >= -3 && row_diff <= 3)
            corner_hold_counter++;
        else
        {
            corner_hold_counter = 1;
            last_corner_row = corner_row;
        }

        if(corner_hold_counter >= CORNER_HOLD_FRAMES && !last_corner_detected)
        {
            detected_corner.detected = 1;
            detected_corner.row = corner_row;
            detected_corner.col = corner_col;
            if(corner_index < 25)
                detected_corner.turn_dir = (uint8)active_turn_plan[corner_index];
            last_corner_detected = 1;
        }
    }
    else
    {
        corner_hold_counter = 0;
        last_corner_detected = 0;
        last_corner_row = 0;

        static uint8 no_corner_counter = 0;
        if(detected_corner.detected)
        {
            no_corner_counter++;
            if(no_corner_counter > 10)
            {
                detected_corner.detected = 0;
                no_corner_counter = 0;
            }
        }
        else
        {
            no_corner_counter = 0;
        }
    }
}
*/
void detect_corner(void)
{

    int16 y;
    int16 left_diff, right_diff;
    // 妫ｅ啯鏆� 濞ｅ浂鍠楅弫濂告晬濮橆収姊炬繛鏉戭儏缁ㄦ娊鏌堥敓锟�60閻炴稑鐭夌槐娆撴閻樿櫣绠悘蹇撶箺濠у懘鎯冮崟顐㈤殬闁糕晝鍣︾槐锟�
    int16 start_row;
    // ========== 鍔ㄦ�佽皟鏁存娴嬭寖鍥� ==========
    // 鍒ゆ柇褰撳墠鑺傜偣鏄惁鏄渶瑕佸欢杩熺殑涓夊矓璺�
    if(current_track==TRACK_3&&corner_index == TRIPOD_NODE_INDEX) {
        start_row = NORMAL_START_ROW;   // 涓夊矓璺細鍙娴嬮潬杩戝皬杞︾殑鍖哄煙
    } else {
        start_row = NORMAL_START_ROW;    // 姝ｅ父妯″紡
    }
   /* if(current_track==TRACK_3&&corner_index == 11) {
           start_row = TRIPOD_START_ROW;   // 涓夊矓璺細鍙娴嬮潬杩戝皬杞︾殑鍖哄煙
       } else {
           start_row = NORMAL_START_ROW;    // 姝ｅ父妯″紡
       }*/
    int16 end_row = (int16)image_h - 1;       // 闁告帗婢樺ù姗�宕撹箛鎾朵亢闂侇喓鍔庣划銊╁级閿燂拷
    uint16 loop_count = 0;
    uint16 max_loop = (end_row - start_row) * 2;
    uint8 corner_found = 0;
    uint8 corner_row = 0;
    uint8 corner_col = 0;

    if(start_row < 0) start_row = 0;
    if(end_row >= (int16)image_h) end_row = (int16)image_h - 1;

    // 濞寸姴楠哥花鎶芥焾閵娿儲鍊诲☉鎾筹攻婢瑰倿骞撹箛銉х闂傚牏濮剧换搴ｄ焊韫囨碍绨犻柣銊ュ缂嶅懐绱旈鑽ゅ枠闁稿繐鐗婇ˉ鍛圭�ｅ墎绀�
    for(y = end_row; y >= start_row; y--)
    {
        loop_count++;
        if(loop_count > max_loop) break;

        if(y <= 0 || y >= (int16)image_h) continue;

        // 缁绢収鍠曠换姘跺嫉婢跺鐟愬☉鎿勬嫹閻炴稑鏈弳鐔煎箲閿燂拷
        if(y == 0) continue;

        left_diff = (int16)l_border[y] - (int16)l_border[y-1];
        right_diff = (int16)r_border[y] - (int16)r_border[y-1];

        if(my_abs(left_diff) > CORNER_ANGLE_THRESH || my_abs(right_diff) > CORNER_ANGLE_THRESH)
        {
            corner_found = 1;
            corner_row = (uint8)y;

            if(my_abs(left_diff) > my_abs(right_diff))
            {
                corner_col = (uint8)l_border[y];
            }
            else
            {
                corner_col = (uint8)r_border[y];
            }
            break;
        }
    }

    if(corner_found)
    {
        int16 row_diff = (int16)corner_row - (int16)last_corner_row;
        if(row_diff >= -3 && row_diff <= 3)
        {
            corner_hold_counter++;
        }
        else
        {
            corner_hold_counter = 1;
            last_corner_row = corner_row;
        }

        if(corner_hold_counter >= CORNER_HOLD_FRAMES && !last_corner_detected)
        {
            // 电阻识别：从拐点行向上扫描到第0行，检查第10列和第178列是否有白点
            // 电阻位于赛道中央，边缘为蓝色背景（无白点），而真正拐点处赛道会延伸到图像边缘
            uint8_t edge_white = 0;
            for(uint8_t r = corner_row; r > 0; r--)
            {
                if(bin_image[r][10] == white_pixel || bin_image[r][178] == white_pixel)
                {
                    edge_white = 1;
                    break;
                }
            }
            if(!edge_white && (bin_image[0][10] == white_pixel || bin_image[0][178] == white_pixel))
                edge_white = 1;

            if(edge_white)
            {
                detected_corner.detected = 1;
                detected_corner.row = corner_row;
                detected_corner.col = corner_col;

                if(corner_index < 25)
                {
                    detected_corner.turn_dir = (uint8)active_turn_plan[corner_index];
                }

                last_corner_detected = 1;
            }
            // 无边缘白点 → 电阻 → 不置位 detected，拐点计数不自增
        }
    }
    else
    {
        corner_hold_counter = 0;
        last_corner_detected = 0;
        last_corner_row = 0;

        static uint8 no_corner_counter = 0;
        if(detected_corner.detected)
        {
            no_corner_counter++;
            if(no_corner_counter > 10)
            {
                detected_corner.detected = 0;
                no_corner_counter = 0;
            }
        }
        else
        {
            no_corner_counter = 0;
        }
    }
}

/*
void handle_corner_turn(void)
{
    if (corner_gyro_active)
    {
        if (corner_gyro_timer > 0) corner_gyro_timer--;
        if (corner_gyro_timer == 0)
        {
            corner_gyro_active = 0;
            corner_target_gyro = 0.0f;
            corner_active_flag = 0;
        }
    }

    // ========== 妫�娴嬪埌鏂版嫄鐐癸紝鎵ц杞悜 ==========
    if (detected_corner.detected && !corner_active_flag)
    {
        if (corner_gyro_active)
        {
            corner_gyro_active = 0;
            corner_target_gyro = 0.0f;
            corner_gyro_timer = 0;
        }
        uint8 target_dir = detected_corner.turn_dir;

        int16 row = detected_corner.row;
        int16 left_diff  = (int16)l_border[row] - (int16)l_border[row-1];
        int16 right_diff = (int16)r_border[row] - (int16)r_border[row-1];
        int16 max_diff = (my_abs(left_diff) > my_abs(right_diff)) ? my_abs(left_diff) : my_abs(right_diff);
        if (max_diff > 100) max_diff = 100;

        // 澶勭悊宸﹁浆鎴栧彸杞�
        if (target_dir == TURN_LEFT || target_dir == TURN_RIGHT)
        {
            #define GYRO_BASE       250.0f   // 鍩虹瑙掗�熷害锛堝害/绉掞級
            #define GYRO_GAIN       2.0f    // 澧炵泭绯绘暟
            float speed = GYRO_BASE + GYRO_GAIN * max_diff;
            if (speed < 30.0f) speed = 30.0f;
            if (speed > 300.0f) speed = 300.0f;

            // 鏍规嵁杞悜鏂瑰悜璁剧疆鐩爣瑙掗�熷害
            if (target_dir == TURN_LEFT)
                corner_target_gyro = speed;
            else
                corner_target_gyro = -speed;

            #define TIME_BASE       0       // 鍩虹鏃堕棿
            #define TIME_GAIN       0.01f   // 鏃堕棿澧炵泭
            int16 duration = TIME_BASE + (int16)(TIME_GAIN * speed);
            if (duration < 4) duration = 4;
            if (duration > 10) duration = 10;

            corner_gyro_timer = duration;
            corner_gyro_active = 1;
            corner_active_flag = 1;
        }
        else
        {
            // 鐩磋鎯呭喌
            corner_gyro_active = 0;
            corner_active_flag = 0;
        }

        if (corner_index < 25) corner_index++;

        // 娓呴櫎褰撳墠鎷愮偣锛岀瓑寰呬笅涓�涓�
        detected_corner.detected = 0;
    }

    // ========== 鐩爣瑙掗�熷害閫愭笎褰掗浂 ==========
    if (!corner_gyro_active && corner_target_gyro != 0.0f)
    {
        #define EXIT_STEP       60.0f        // 閫�鍑烘闀匡紙搴�/绉掞級
        if (corner_target_gyro > 0)
        {
            corner_target_gyro -= EXIT_STEP;
            if (corner_target_gyro < 0) corner_target_gyro = 0;
        }
        else
        {
            corner_target_gyro += EXIT_STEP;
            if (corner_target_gyro > 0) corner_target_gyro = 0;
        }
    }

    // ========== 棰濆娓呴櫎閫昏緫 ==========
    if (!corner_gyro_active && detected_corner.detected)
    {
        detected_corner.detected = 0;
    }
}
*/
void handle_corner_turn(void)
{
    // Detect new corner -> start IMU 90-degree angle turn
    if (detected_corner.detected && !corner_turning_active)
    {
        float current_yaw = get_imu_yaw();

        if (detected_corner.turn_dir == TURN_LEFT)
        {
            corner_target_angle = current_yaw + 90.0f;
            if (corner_target_angle > 180.0f) corner_target_angle -= 360.0f;
            if (corner_target_angle < -180.0f) corner_target_angle += 360.0f;
            corner_turning_active = 1;
            corner_start_angle = current_yaw;
        }
        else if (detected_corner.turn_dir == TURN_RIGHT)
        {
            corner_target_angle = current_yaw - 90.0f;
            if (corner_target_angle > 180.0f) corner_target_angle -= 360.0f;
            if (corner_target_angle < -180.0f) corner_target_angle += 360.0f;
            corner_turning_active = 1;
            corner_start_angle = current_yaw;
        }

        corner_index++;
        detected_corner.detected = 0;
    }
}

// ==================== 重置拐点转向状态 ====================
void reset_corner_turn_state(void)
{
    corner_active = 0;
    corner_turn_timer = 0;
    corner_active_flag = 0;
    corner_turn_dir = TURN_STRAIGHT;
    corner_target_row = 0;
    corner_error_bias = 0;
    corner_turning_active = 0;
    corner_target_angle = 0.0f;
    corner_start_angle = 0.0f;
}


// ==================== 鏄剧ず缁樺埗鍑芥暟 ====================
void draw_boundary_and_center(void)
{
    uint16 y;
    for(y = 0; y < image_h; y++)
    {
        if(l_border[y] > 0 && l_border[y] < image_w)
        {
            ips114_draw_point(l_border[y], y, COLOR_BLUE);
            if(l_border[y]+1 < image_w) ips114_draw_point(l_border[y]+1, y, COLOR_BLUE);
        }
        if(r_border[y] > 0 && r_border[y] < image_w)
        {
            ips114_draw_point(r_border[y], y, COLOR_GREEN);
            if(r_border[y]+1 < image_w) ips114_draw_point(r_border[y]+1, y, COLOR_GREEN);
        }
        if(center_line[y] > 0 && center_line[y] < image_w)
        {
            ips114_draw_point(center_line[y], y, COLOR_RED);
            if(center_line[y]+1 < image_w) ips114_draw_point(center_line[y]+1, y, COLOR_RED);
        }
    }
}

void draw_corner_marker(void)
{
    if(detected_corner.detected)
    {
        int16 dy, dx;
        for(dy = -5; dy <= 5; dy++)
        {
            for(dx = -5; dx <= 5; dx++)
            {
                int16 row = (int16)detected_corner.row + dy;
                int16 col = (int16)detected_corner.col + dx;
                if(row >= 0 && row < (int16)image_h && col >= 0 && col < (int16)image_w)
                {
                    ips114_draw_point((uint16)col, (uint16)row, COLOR_YELLOW);
                }
            }
        }
    }
}

void display_white_pixel_count(void)
{
    char str[32];
    uint16 white_bottom, white_top;
    uint8 bottom_row, top_row;

    bottom_row = image_h - 5;
    if(bottom_row >= image_h) bottom_row = image_h - 1;

    top_row = 3;
    if(top_row >= image_h) top_row = image_h - 1;

    white_bottom = count_white_pixels_in_row(bottom_row);
    white_top = count_white_pixels_in_row(top_row);

    ips114_set_font(IPS114_8X16_FONT);

    sprintf(str, "W-B:%03d", white_bottom);
    ips114_set_color(COLOR_CYAN, COLOR_BLACK);
    ips114_show_string(180, 48, str);

    sprintf(str, "W-T:%03d", white_top);
    ips114_set_color(COLOR_CYAN, COLOR_BLACK);
    ips114_show_string(180, 60, str);

    sprintf(str, "D:%+04d", (int)white_top - (int)white_bottom);
    ips114_set_color(COLOR_YELLOW, COLOR_BLACK);
    ips114_show_string(180, 72, str);
}

/*void display_track_info(void)
{
    char str[32];
    uint16 color;

    switch(current_track)
    {
        case TRACK_1: color = COLOR_RED; break;
        case TRACK_2: color = COLOR_GREEN; break;
        case TRACK_3: color = COLOR_BLUE; break;
        default: color = COLOR_WHITE; break;
    }

    ips114_set_font(IPS114_8X16_FONT);

    sprintf(str, "T%d", current_track + 1);
    ips114_set_color(color, COLOR_BLACK);
    ips114_show_string(200, 0, str);

    sprintf(str, "C:%d", corner_index);
    ips114_set_color(COLOR_YELLOW, COLOR_BLACK);
    ips114_show_string(200, 16, str);

    if(corner_active)
    {
        if(corner_turn_dir == TURN_LEFT)
        {
            sprintf(str, "L:%d", corner_turn_timer);
            ips114_set_color(COLOR_RED, COLOR_BLACK);
        }
        else if(corner_turn_dir == TURN_RIGHT)
        {
            sprintf(str, "R:%d", corner_turn_timer);
            ips114_set_color(COLOR_RED, COLOR_BLACK);
        }
        else
        {
            sprintf(str, "S");
            ips114_set_color(COLOR_WHITE, COLOR_BLACK);
        }
        ips114_show_string(200, 32, str);
    }

    display_white_pixel_count();
}*/
void display_track_info(void)
{
    char str[32];
    uint16 color;

    // 璧涢亾缂栧彿棰滆壊
    switch(current_track)
    {
        case TRACK_1: color = COLOR_RED; break;
        case TRACK_2: color = COLOR_GREEN; break;
        case TRACK_3: color = COLOR_BLUE; break;
        default: color = COLOR_WHITE; break;
    }

    ips114_set_font(IPS114_8X16_FONT);

    // ===== 鍙充笂鍖哄煙锛坸=190, 鍥惧儚鍙充晶 52px 瀹斤級=====
    // 鏄剧ず璧涢亾缂栧彿
    sprintf(str, "T%d", current_track + 1);
    ips114_set_color(color, COLOR_BLACK);
    ips114_show_string(190, 0, str);

    // 鏄剧ず宸查�氳繃鐨勬嫄鐐规暟閲�
    sprintf(str, "C:%d", corner_index);
    ips114_set_color(COLOR_YELLOW, COLOR_BLACK);
    ips114_show_string(190, 16, str);

    // 鏄剧ず杞集鐘舵��
    if (corner_turning_active)
    {
        ips114_set_color(COLOR_RED, COLOR_BLACK);
        ips114_show_string(190, 32, "TURN");
    }
    else
    {
        ips114_set_color(COLOR_GREEN, COLOR_BLACK);
        ips114_show_string(190, 32, "STR ");
    }

    // ===== 闄�铻轰华璋冭瘯淇℃伅锛堝彔灞傛樉绀哄湪鍥惧儚宸︿晶锛寈=2锛�=====
    // 鑾峰彇褰撳墠鍋忚埅瑙掋�佺洰鏍囪搴﹀拰璇樊
    float current_yaw = get_imu_yaw();
    float target_angle = corner_target_angle;
    float error = target_angle - current_yaw;
    // 瑙掑害褰掍竴鍖栧埌 [-180,180]
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    sprintf(str, "S:%d", (int)corner_start_angle);
    ips114_set_color(COLOR_WHITE, COLOR_BLACK);
    ips114_show_string(2, 56, str);

    sprintf(str, "Y:%d", (int)current_yaw);
    ips114_set_color(COLOR_CYAN, COLOR_BLACK);
    ips114_show_string(2, 72, str);

    sprintf(str, "T:%d", (int)target_angle);
    ips114_set_color(COLOR_CYAN, COLOR_BLACK);
    ips114_show_string(2, 88, str);

    sprintf(str, "E:%d", (int)error);
    ips114_set_color(COLOR_YELLOW, COLOR_BLACK);
    ips114_show_string(2, 104, str);
}

// ==================== 璧涢亾鍒囨崲 ====================
void switch_to_track(uint8 track_idx)
{
    if(track_idx >= TRACK_COUNT) return;

    current_track = track_idx;

    switch(track_idx)
    {
        case TRACK_1: active_turn_plan = track1_turn_plan; break;
        case TRACK_2: active_turn_plan = track2_turn_plan; break;
        case TRACK_3: active_turn_plan = track3_turn_plan; break;
    }

    corner_index = 0;
    reset_corner_state();
    reset_corner_turn_state();

    printf("Switch to Track %d\r\n", track_idx + 1);
}

// ==================== 鎸夐敭澶勭悊 ====================
static uint8_t motor_enabled = 0;
static uint8_t last_key = 0;

static uint8_t read_key(void)
{
    if(gpio_get_level(KEY_UP_PIN) == 0) return 1;
    if(gpio_get_level(KEY_DOWN_PIN) == 0) return 2;
    if(gpio_get_level(KEY_LEFT_PIN) == 0) return 3;
    if(gpio_get_level(KEY_RIGHT_PIN) == 0) return 4;
    if(gpio_get_level(KEY_ENTER_PIN) == 0) return 5;
    return 0;
}

void check_key(void)
{
    uint8_t key = read_key();

    if(key != 0)
    {
        switch(key)
        {
            case 1:  // 涓婇敭 -> 璧涢亾1
                switch_to_track(TRACK_1);
                break;
            case 2:  // 涓嬮敭 -> 璧涢亾2
                switch_to_track(TRACK_2);
                break;
            case 3:  // 宸﹂敭 -> 璧涢亾3
                switch_to_track(TRACK_3);
                break;
            case 5:  // 涓敭 -> 鍚姩/鍋滄鐢垫満
                if(motor_enabled)
                {
                    motor_enabled = 0;
                    Motor_Control(0, 0);
                    fan_off();
                    printf("Motor STOP!\r\n");
                }
                else
                {
                    fan_on();                          // 鍏堝惎鍔ㄨ礋鍘嬮鎵�
                    fan_prerun_active = 1;              // 杩涘叆棰勮繍琛岄樁孌�
                    fan_prerun_counter = 80;            // 80 * 10ms = 800ms 绛夊緟璐熷帇寤虹珛
                    printf("Fan pre-running 800ms...\r\n");
                }
                break;
        }
    }

    last_key = key;
}

uint8_t is_motor_running(void)
{
    return motor_enabled;
}

void motor_start(void)
{
    motor_enabled = 1;
}

void init_track_selector(void)
{
    gpio_init(KEY_UP_PIN,    GPI, 0, GPI_PULL_UP);
    gpio_init(KEY_DOWN_PIN,  GPI, 0, GPI_PULL_UP);
    gpio_init(KEY_LEFT_PIN,  GPI, 0, GPI_PULL_UP);
    gpio_init(KEY_RIGHT_PIN, GPI, 0, GPI_PULL_UP);
    gpio_init(KEY_ENTER_PIN, GPI, 0, GPI_PULL_UP);

    switch_to_track(TRACK_1);
}

// ==================== 涓诲浘鍍忓鐞嗗嚱鏁� ====================
int image_process(void)
{
    static uint16 frame_count = 0;
    static uint8 consecutive_failures = 0;
    static uint8 first_run = 1;

    // 棣栨杩愯鍒濆鍖�
    if(first_run)
    {
        corner_index = 0;
        reset_corner_state();
        reset_corner_turn_state();
        first_run = 0;
    }

    frame_count++;

    // 鑾峰彇鍥惧儚
    Get_image(mt9v03x_image);

    // 浜屽�煎寲澶勭悊
    turn_to_bin_track();

    // 瀵绘壘宸﹀彸杈圭晫
    find_left_right_boundary();

    // 杈圭晫鏈夋晥鎬ф鏌�
    uint8 boundary_valid = 0;
    for(int i = image_h - 5; i < image_h; i++)
    {
        if(i >= 0 && i < image_h)
        {
            if(l_border[i] < r_border[i] &&
               l_border[i] > border_min &&
               r_border[i] < border_max)
            {
                boundary_valid = 1;
                break;
            }
        }
    }

    // 杈圭晫鏃犳晥鏃剁殑澶勭悊
    if(!boundary_valid)
    {
        consecutive_failures++;
        if(consecutive_failures > 20)
        {
            // 杈圭晫涓㈠け鏃朵娇鐢ㄩ粯璁ゅ��
            for(int i = 0; i < image_h; i++)
            {
                l_border[i] = image_w / 2 - 20;
                r_border[i] = image_w / 2 + 20;
            }
        }
    }
    else
    {
        consecutive_failures = 0;
    }

    // 璁＄畻涓績绾�
    calculate_center_line();

   /* // 妫�娴嬫嫄鐐�
    detect_corner();

    // 澶勭悊鎷愮偣杞悜
    handle_corner_turn();
*/
    // 检测拐点前，先判断是否正在转弯（转弯时强制清标志，防止误触发）
    // 注：如果有连续转弯需求，此处的屏蔽逻辑可能导致漏检，
    //     需要配合 handle_corner_turn 中的缓存机制解除屏蔽
    if (!corner_turning_active) {
        detect_corner();
    } else {
        // 转弯中强行清除任何残留的拐点标志
        detected_corner.detected = 0;
    }
    handle_corner_turn();
    // 鏄剧ず鍥惧儚鍜岃皟璇曚俊鎭�
    #if 1
    ips114_show_gray_image(0, 0, bin_image[0], image_w, image_h, image_w, image_h, 0);
    draw_boundary_and_center();
    draw_corner_marker();
    display_track_info();
    #endif

    return 0;
}
