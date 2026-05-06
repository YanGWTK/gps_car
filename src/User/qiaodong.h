/*
 * qiaodong.h
 *
 *  Created on: 2025年4月14日
 *      Author: 29500
 */

#ifndef SRC_USER_QIAODONG_H_
#define SRC_USER_QIAODONG_H_
#define MIN_LINE_WIDTH   7    // 白线最小宽度（像素）
#define MAX_LINE_WIDTH   60   // 白线最大宽度（像素）
#define EDGE_SEARCH_STEP 3     // 边缘搜索步长（加速搜索）
#define IMAGE_CENTER (LCDW / 2)  // 图像中心X坐标
#define MAX_DEVIATION 80         // 最大允许偏差（根据实际调整）
#define THRESH_HISTORY 10 // 历史帧数（按50FPS约0.2秒窗口）
#define BLOCK_SIZE 21    // 局部区域大小（建议21x21）
#define GLOBAL_WEIGHT 0.3 // 全局阈值权重（0~1）
#define CROP_END_ROW     (LCDH*3/4)
#define WHITE_LINE_THRESHOLD 200    // 基础白线阈值
#define NOISE_AREA_SIZE      50     // 噪声区域阈值
extern int ca_azimuth;
void GaussianFilter(unsigned char img[LCDH][LCDW]);
int Calculate_Centerline(void);
void Track_White_Line(void);
int GetOSTU_Stable(unsigned char tmImage[LCDH][LCDW]);
void AdaptiveThreshold(unsigned char input[LCDH][LCDW],
                       unsigned char output[LCDH][LCDW]) ;
#endif /* SRC_USER_QIAODONG_H_ */
