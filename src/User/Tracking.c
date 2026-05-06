/*
 * Tracking.c
 *
 *  Created on: 2025年3月5日
 *      Author: 29500
 */
#include "include.h"
double azimuth=0;//偏航角
double angle_target=0;//车到目标点的角度
double gd_azimuth=0;
float Servo_Duty;
double gyro_angle_1;
double distance_max=3;//最大距离,第一次初始化为5，以便纠正自己位置
double distance_low=3;//最小距离（自己设定）
double middle_distance=5.0;//减速开始距离
int INERTIAL_NAVIGATION=0;//惯导
int short_diatance_flag=0;
int16_t duty_max= 20;
int16_t SPEED_high=0;//根据距离决定的最大速度（计算得出）
int16_t SPEED_VALUE=0;//最终得出来的速度
double use_angle;
uint8_t car_mode = 1;
Inertial_Navigation navigation;
void GPS_Track(double distance,uint8 num)//gps循迹
{
angle_target=  get_two_points_azimuth(GPS_data.LAT,GPS_data.LON, point[num][0],point[num][1]);
azimuth = angle_target - use_angle;
angle_plan(&azimuth);
Servo_Duty=PidLocCtrl(&GPS_PID, azimuth);

   if(fp_flag)//飞坡
   {
     SPEED_high=2500;
     fp_flag=0;
   }
   else if(db_flag)//颠簸路段
   {
     SPEED_high=3000;
     db_flag=0;
   }
   else if(qd_flag)//桥洞
   {
     SPEED_high=1000;
     distance_target=get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[next_point][0], point[next_point][1]);
     while(distance_target>1)
     {
         Track_White_Line();
         if(qd_num>60)
             break;
//         SPEED_VALUE=SPEED_high-(int)(SPEED_VALUE/5/(duty_max)*fabs(Servo_Duty));
//         if(SPEED_VALUE<700) SPEED_VALUE=700;
    #if MOTOR_MODE
         motor_loop(1000);
    #else
         BLDC_motor_crtl(SPEED_VALUE);
    #endif
         distance_target=get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[next_point][0], point[next_point][1]);
         Delay_Ms(6);
//          Track_White_Line();
     }
     qd_flag=0;
   }
   else if(cp_flag)//草坪
   {
       SPEED_high=base_speed;
       cp_flag=0;
   }
   else if(task2_flag==1)
   {
       if(next_point>=2&&next_point<3+zhuitong)
           SPEED_high=gd_base_speed;
       else if(next_point>4+zhuitong&&next_point<10+zhuitong)
           SPEED_high=gd_base_speed;
       else
           SPEED_high=base_speed;
   }
   else
   {
     SPEED_high=base_speed;
   }
// }
 SPEED_VALUE=SPEED_high-(int)(SPEED_VALUE/4/(duty_max)*fabs(Servo_Duty));
 if(SPEED_VALUE<800) SPEED_VALUE=800;
#if MOTOR_MODE
     motor_loop(SPEED_VALUE);
#else
     BLDC_motor_crtl(SPEED_VALUE);
#endif
Servo_Ctrl(Servo_Duty);
Delay_Ms(8);
}
void GPS3_Track(double distance,uint8 num)//这里我想的是取消摄像头循迹，直接全速冲过去，不建议用
{
angle_target=  get_two_points_azimuth(GPS_data.LAT,GPS_data.LON, point[num][0],point[num][1]);
azimuth = angle_target - use_angle;
angle_plan(&azimuth);
Servo_Duty=PidLocCtrl(&GPS_PID, azimuth);

   if(fp_flag)//飞坡
   {
     SPEED_high=1350;
     fp_flag=0;
   }
   else if(db_flag)//颠簸路段
   {
     SPEED_high=1800;
     db_flag=0;
   }
   else if(qd_flag)//桥洞
   {
     SPEED_high=1900;

     qd_flag=0;
   }
   else if(cp_flag)//草坪
   {
       SPEED_high=base_speed;
       cp_flag=0;
   }
   else if(task2_flag==1)
   {
       if(next_point>=2&&next_point<3+zhuitong)
           SPEED_high=gd_base_speed;
       else if(next_point>4+zhuitong&&next_point<10+zhuitong)
           SPEED_high=gd_base_speed;
       else
           SPEED_high=base_speed;
   }
   else
   {
     SPEED_high=base_speed;
   }
// }
 SPEED_VALUE=SPEED_high-(int)(SPEED_VALUE/4/(duty_max)*fabs(Servo_Duty));
 if(SPEED_VALUE<800) SPEED_VALUE=800;
#if MOTOR_MODE
     motor_loop(SPEED_VALUE);
#else
     BLDC_motor_crtl(SPEED_VALUE);
#endif
Servo_Ctrl(Servo_Duty);
Delay_Ms(8);
}
void Start_Track()
{
    encoder_totol=0;
SPEED_VALUE=speed_set-1;
SPEED_high=base_speed;
//#if MOTOR_MODE
     motor_loop(SPEED_VALUE);


while(1)
{

    angle_plan(&use_angle);
    Servo_Duty=PidLocCtrl(&GPS_PID, -use_angle);
    Servo_Ctrl(Servo_Duty);
    if(encoder_totol>70000)
{
start_flag1=1;
gyro_angle_2=gyro_angle;
    gyro_angle_1=get_two_points_azimuth(point[0][0],point[0][1], GPS_data.LAT,GPS_data.LON);
//    gyro_angle=get_two_points_azimuth(point[0][0],point[0][1], GPS_data.LAT,GPS_data.LON);

    check_flag=0;
    break;
}

}

next_point=1;


}
void End_Track(double distance)
{
    angle_target= get_two_points_azimuth(GPS_data.LAT,GPS_data.LON, point[last_point][0],point[last_point][1]);
    azimuth = angle_target - use_angle;
    angle_plan(&azimuth);
    Servo_Duty=PidLocCtrl(&GPS_PID, azimuth);
       SPEED_high=base_speed;
     SPEED_VALUE=SPEED_high-(int)(SPEED_VALUE/4/(duty_max)*fabs(Servo_Duty));
     if(SPEED_VALUE<1000) SPEED_VALUE=1000;
#if MOTOR_MODE
     motor_loop(SPEED_VALUE);
#else
     BLDC_motor_crtl(SPEED_VALUE);
#endif
     Servo_Ctrl(Servo_Duty);
     Delay_Ms(7);
}

//惯导2号，没用到
void Inertial_Navigation_Start(void) {
    //设置当前坐标,抓取当前坐标系为基准坐标系
    navigation.start_yaw = gyro_angle;
    navigation.x_cur = 0;
    navigation.y_cur = 0;
    navigation.x_start = 0 * 1; //mm
    navigation.y_start = 0 * 1; //mm
    navigation.x_set = 0 * 1;   //mm
    navigation.y_set = 0 * 1;   //mm
}
void Inertial_Navigation_Update(void)
{
    navigation.cur_yaw = gyro_angle - navigation.start_yaw;
        navigation.inertial_navigation_cur_distance = encoder_totol;
        navigation.x_cur = navigation.x_cur + sin(navigation.cur_yaw / 180 * 3.14159f) * (encoder / 150 * 50 * 0.002);
        navigation.y_cur = navigation.y_cur + cos(navigation.cur_yaw / 180 * 3.14159f) * (encoder / 150 * 50 * 0.002);
        if (car_mode == 1)
        {
            float dx = navigation.x_set - navigation.x_cur;
            float dy = navigation.y_set - navigation.y_cur;
            navigation.inertial_navigation_set_yaw_update = atan2(dx,dy) / 3.14159f * 180;
        }
        float i = navigation.inertial_navigation_set_yaw_update - navigation.inertial_navigation_set_yaw_old;
        angle_handle_360(&i);
        navigation.inertial_navigation_set_yaw_up_total += i;


                    if (navigation.inertial_navigation_set_yaw_up_total - navigation.cur_yaw > 360) {
                        while (1) {
                            navigation.inertial_navigation_set_yaw_up_total -= 360;
                            if (navigation.inertial_navigation_set_yaw_up_total - navigation.cur_yaw < 360) {
                                break;
                            }
                        }
                    }
                    else if (navigation.inertial_navigation_set_yaw_up_total - navigation.cur_yaw < -360) {
                        while (1) {
                            navigation.inertial_navigation_set_yaw_up_total += 360;
                            if (navigation.inertial_navigation_set_yaw_up_total - navigation.cur_yaw > -360) {
                                break;
                            }
                        }
                    }
                    navigation.inertial_navigation_set_yaw_old = navigation.inertial_navigation_set_yaw_up_total;
}

