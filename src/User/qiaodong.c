/*
 * qiaodong.c
 *
 *  Created on: 2025年4月14日
 *      Author: 29500
 */

//摄像头循迹前要进行摄像头曝光度设置，在室外一般选择最低，在mt9v034的头文件设置
#include "include.h"
char txt[30];
int ca_azimuth;
// 高斯滤波去噪
void GaussianFilter(unsigned char img[LCDH][LCDW]) {
    static const int kernel[3][3] = {{1,2,1}, {2,4,2}, {1,2,1}};
    for(int y=1; y<LCDH-1; y++){
        for(int x=1; x<LCDW-1; x++){
            int sum = 0;
            for(int i=-1; i<=1; i++){
                for(int j=-1; j<=1; j++){
                    sum += img[y+i][x+j] * kernel[i+1][j+1];
                }
            }
            img[y][x] = sum / 16;
        }
    }
}
int Calculate_Centerline(void)
{
    int center_sum = 0;
    int valid_rows = 0;
    int last_center = IMAGE_CENTER; // 默认中心（无检测时使用历史值）
    static int lost_counter = 0;
    static int hist_centers[5] = {IMAGE_CENTER};
    static int prev_width = (MIN_LINE_WIDTH + MAX_LINE_WIDTH)/2;

    // 从底部向上扫描（覆盖近处70%区域）
    for (int y = 0; y < CROP_END_ROW; y++) {
        int left_edge = -1, right_edge = -1;

        /*---------- 左边缘检测 ----------*/
        // 快速扫描：隔3像素检测提高搜索速度
        for (int x = 0; x < LCDW; x += EDGE_SEARCH_STEP) {
            if (Pixle[y][x] == 255) {
                left_edge = (x > EDGE_SEARCH_STEP) ? x - EDGE_SEARCH_STEP : 0;
                break;
            }
        }
        // 微调：精确查找边缘过渡点
        if (left_edge != -1) {
            for (int x = left_edge; x >= 0; x--) { // 向左微调
                if (Pixle[y][x] != 255) {
                    left_edge = x + 1; // 找到第一个白点右侧为真实边缘
                    break;
                }
            }
        }

        /*---------- 右边缘检测 ----------*/
        // 快速扫描：从右向左搜索
        for (int x = LCDW-1; x >= 0; x -= EDGE_SEARCH_STEP) {
            if (Pixle[y][x] == 255) {
                right_edge = (x < LCDW-EDGE_SEARCH_STEP) ? x + EDGE_SEARCH_STEP : LCDW-1;
                break;
            }
        }
        // 微调：精确查找右边缘
        if (right_edge != -1) {
            for (int x = right_edge; x < LCDW; x++) { // 向右微调
                if (Pixle[y][x] != 255) {
                    right_edge = x - 1; // 找到最后一个白点左侧为真实边缘
                    break;
                }
            }
        }

        /*---------- 有效性验证 ----------*/
        // 边缘存在性检查
        if (left_edge == -1 || right_edge == -1) continue;
        // 线宽合理性检查（动态调整示例）
        int line_width = right_edge - left_edge;
        int dynamic_min = MIN_LINE_WIDTH + y/10; // 底部允许更宽
        if (line_width < dynamic_min || line_width > MAX_LINE_WIDTH) continue;

        /*---------- 加权中心计算 ----------*/
        // 行中心计算
        int row_center = (left_edge + right_edge) / 2;
        // 动态权重：顶部行权重更高（90->1线性递减）
        int weight = CROP_END_ROW - y;
        center_sum += row_center * weight;
        valid_rows += weight;
    }

    /*---------- 丢失处理逻辑 ----------*/
    // 丢失处理：惯性预测 + 平滑过渡
    if (valid_rows == 0) {
        lost_counter = (lost_counter < 20) ? lost_counter + 1 : 20;
        int predicted = 2*hist_centers[4] - hist_centers[3];
        return (predicted * (20 - lost_counter) + IMAGE_CENTER * lost_counter) / 20 - IMAGE_CENTER;
    } else {
        // 更新历史数据
        lost_counter = 0;
        last_center = center_sum / valid_rows;
        return last_center - IMAGE_CENTER;
    }
}

void Track_White_Line(void)
{
char txt[30];
    if (Camera_Flag == 2)
    {
        /* 1. 图像预处理 */
        Get_Use_Image();        // 获取缩放后图像
        Get_Bin_Image(0);       // 使用大津法二值化（白线为高亮）本函数进去调整，清除干扰白线，例如操场上其他的白线
        Display_Show(0, 0, LCDH, LCDW, (unsigned char *)Pixle);
        /* 2. 计算中线偏差 */
        int deviation = Calculate_Centerline();
        ca_azimuth=deviation;
        sprintf(txt, "T:%d", deviation);
        Display_showString(0, 1, txt, U16_WHITE, U16_BLACK, 16); // 时间
        /* 3. PID控制 */
        Servo_Duty=PidIncCtrl(&CA_PID,ca_azimuth);
           Servo_Ctrl(Servo_Duty);
        /* 4. 输出控制信号 */

        Camera_Flag = 0; // 清除标志位
    }
}
int GetOSTU_Stable(unsigned char tmImage[LCDH][LCDW]) {
    static int hist[THRESH_HISTORY] = {0};
    static int index = 0;

    // 计算当前帧阈值
    int current = GetOSTU(tmImage);

    // 更新历史缓冲区
    hist[index] = current;
    index = (index + 1) % THRESH_HISTORY;

    // 加权平均（最近帧权重高）
    int sum = 0, weight_sum = 0;
    for (int i = 0; i < THRESH_HISTORY; i++) {
        int weight = (i == (THRESH_HISTORY-1)) ? 5 : 1; // 最新帧权重5倍
        sum += hist[(index + i) % THRESH_HISTORY] * weight;
        weight_sum += weight;
    }
    return sum / weight_sum;
}
//void AdaptiveThreshold(unsigned char input[LCDH][LCDW],
//                       unsigned char output[LCDH][LCDW])
//{
//    // Step 1: 计算全局阈值（大津法）
//     int global_thresh = GetOSTU(input);
//
//     // Step 2: 局部自适应阈值
//     int half = BLOCK_SIZE / 2;
//     for (int y = half; y < LCDH - half; y++) {
//         for (int x = half; x < LCDW - half; x++) {
//             // 计算局部均值
//             int local_sum = 0;
//             for (int dy = -half; dy <= half; dy++) {
//                 for (int dx = -half; dx <= half; dx++) {
//                     local_sum += input[y+dy][x+dx];
//                 }
//             }
//             int local_mean = local_sum / (BLOCK_SIZE*BLOCK_SIZE);
//
//             // 混合阈值 = 全局阈值 * 权重 + 局部均值 + C
//             int hybrid_thresh = (global_thresh * GLOBAL_WEIGHT)
//                               + (local_mean * (1 - GLOBAL_WEIGHT))
//                               + C;
//
//             // 二值化
//             output[y][x] = (input[y][x] > hybrid_thresh) ? 255 : 0;
//         }
//     }
//
//    // 边界填充（简单复制边缘值）
//    for (int y = 0; y < LCDH; y++) {
//        for (int x = 0; x < LCDW; x++) {
//            if (y < half || y >= LCDH-half || x < half || x >= LCDW-half) {
//                output[y][x] = (input[y][x] > 128) ? 255 : 0; // 简单全局阈值
//            }
//        }
//    }
//
//}
void AntiGlare_Binarization(unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW],int blockSize,int C)
{
    int i, j, m, n;
    int sum = 0, count = 0;
    int threshold = 0;

    // Edge margin handling
    int margin = blockSize/2;

    for(i = margin; i < LCDH - margin; i++) {
        for(j = margin; j < LCDW - margin; j++) {

            // Calculate local mean
            sum = 0;
            count = 0;
            for(m = -margin; m <= margin; m++) {
                for(n = -margin; n <= margin; n++) {
                    sum += imageIn[i+m][j+n];
                    count++;
                }
            }

            // Adaptive threshold calculation
            threshold = (sum / count) - C;

            // Threshold clamping
            threshold = (threshold < 30) ? 30 : (threshold > 220) ? 220 : threshold;

            // Basic binarization
            imageOut[i][j] = (imageIn[i][j] > threshold+30) ? 255 : 0;

            // Over-exposure processing
            if(imageIn[i][j] > 220)
            { // Over-exposure threshold
                int blackCount = 0;
                for(m = -1; m <= 1; m++) {
                    for(n = -1; n <= 1; n++) {
                        if(imageIn[i+m][j+n] < threshold) blackCount++;
                    }
                }
                imageOut[i][j] = (blackCount >= 4) ? 0 : 255;
            }
        }
    }
}
