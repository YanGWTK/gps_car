/*
 * Tracking.h
 *
 *  Created on: 2025年3月5日
 *      Author: 29500
 */

#ifndef SRC_USER_TRACKING_H_
#define SRC_USER_TRACKING_H_
extern int16_t SPEED_high;//根据距离决定的最大速度（计算得出）
extern int16_t SPEED_VALUE;//最终得出来的速度
extern double azimuth;//偏航角
extern double angle_target;//车到目标点的角度
extern double gd_azimuth;
extern float Servo_Duty;
extern int short_diatance_flag;
extern double distance_max;//最大距离,第一次初始化为5，以便纠正自己位置
extern double distance_low;//最小距离（自己设定）
extern double middle_distance;//减速开始距离
extern double use_angle;
extern double gyro_angle_1;
extern uint8_t car_mode;
extern int16_t duty_max;
typedef struct {
    unsigned char inertial_navigation_state;//惯性导航当前状态
    unsigned char inertial_navigation_state_last;//惯性导航上一状态
    float inertial_navigation_start_absyaw;//惯性导航起始偏航(绝对值)
    float inertial_navigation_set_yaw;//惯性导航设定偏航斜坡当前值
    float inertial_navigation_set_distance;//惯性导航设定距离
    float inertial_navigation_cur_distance;//惯性导航当前距离
    float inertial_navigation_set_yaw_update;//惯性导航设定偏航斜坡更新值
    float inertial_navigation_set_yaw_total;//惯性导航设定偏航斜坡更新值
    float inertial_navigation_set_yaw_up_total;//惯性导航设定偏航斜坡更新值
    float inertial_navigation_set_yaw_old;
    float start_yaw;//惯导起始偏航
    float cur_yaw;//惯导当前偏航
    float x_cur;//惯导当前x坐标
    float y_cur;//惯导当前y坐标
    float x_set;//惯导设定x坐标
    float y_set;//惯导设定y坐标
    float x_start;//惯导起始x坐标
    float y_start;//惯导起始y坐标
}Inertial_Navigation;
extern Inertial_Navigation navigation;
void GPS_Track(double distance,uint8 num);
void Start_Track();
void End_Track(double distance);
void Inertial_Navigation_Start(void);
void GPS3_Track(double distance,uint8 num);
#endif /* SRC_USER_TRACKING_H_ */
