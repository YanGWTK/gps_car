/*
 * control.c
 *
 *  Created on: 2025年3月3日
 *      Author: 29500
 */
#include "include.h"
uint8 switch_flag=0;
 uint8 check_flag=1;
sint16 speed_set=1200;//起始速度
sint16 speed_loop_set;
sint16 base_speed=1200;//正常循迹速度
sint16 gd_base_speed=1200;//惯导速度或者绕锥桶速度
sint16 base_loop_speed;
uint8_t kemu[4];
double distance_target=0;
int start_flag;
int stop_flag;
int filter_flag=1;//滤波
double filter_point[2];
int16_t next_point=1;
uint16_t last_point;
uint16_t turn_point;
float this_postion = 0, last_postion = 0; // 记忆当前位置值,历史值
float _get_yaw = 0;
float add_pos = 0;
int8_t save_flag=0;
int8_t save_run=0;
int16_t in_index=0;
int16_t out_index=0;
int16_t _in_index;
int8_t eeprom_in=0;
uint16_t long_index=0;
uint8_t start_flag1=0;
uint8_t back_flag=0;
uint8_t kemu_flag=0;
uint8_t task3_flag=0;
uint8_t num=0;
uint8_t fp_flag=0;
uint8_t db_flag=0;
uint8_t qd_flag=0;
uint8_t cp_flag=0;
uint8_t task1_flag=0;
uint8_t task2_flag=0;
uint8_t task3_flag_s=0;
uint8_t zhuitong=4;
uint8_t text_flag=0;
uint16_t qd_num;
/*********用于缓提速***********/
int16_t V;
int16_t BASE_V;
/*********用于缓提速**********/
void GD_star()//惯导
{
    char txt[30];
//    CCU6_InitConfig(CCU61,CCU6_Channel1,1);
    int8_t SET_NOW=0;//现在正在调节的参数
            int8_t SET_LAST=0;
        Display_showString(0, 1,"->", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 1,"save", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 3, "save_ok", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 5,"run", U16_WHITE, U16_BLACK, 12);
        while(1)
             {
                 key_scan();
                 if(KEY0_flag==1)//检测到KEY0按下，退出
                    {
                      key_clean();
                      break;
                    }
                    else if(KEY1_flag==1)
                    {
                        key_clean();
                       if(SET_NOW==2)
                      {
                           SET_NOW=0;
                      }
                      else
                      {
                          SET_NOW++;
                      }
                    }
                    else if(KEY2_flag==1)
                    {
                        key_clean();
                        switch(SET_NOW)
                        {
                            case 0:save_flag=1;memset(IMU_buf,0xff,2000);Display_showString(0, 11, "save", U16_WHITE, U16_BLACK, 12);Display_CLS(U16_BLACK);break;
                            case 1:eeprom_in=1;Display_showString(0, 11, "save_ok", U16_WHITE, U16_BLACK, 12);Display_CLS(U16_BLACK);break;
                            case 2:save_run=1;out_index=0;gyro_angle_totol=0; GD_get();Display_showString(0, 11, "run", U16_WHITE, U16_BLACK, 12);Display_CLS(U16_BLACK);break;

                        }
                    }
                 if(SET_LAST!=SET_NOW)
                         {
                             Display_CLS(U16_BLACK);
                                    Display_showString(0, 1+(2*SET_NOW), "->", U16_WHITE, U16_BLACK, 12);
                                    SET_LAST=SET_NOW;
                         }
                 if(eeprom_in==1)
                 {
                     _in_index=IMU_buf[0];
                     EEPROM_Write(4, 0, IMU_buf, 2000);
                 save_flag=0;
                 Display_showString(0, 11, "save ok", U16_WHITE, U16_BLACK, 12);
                 }
                 if(save_run==1)
                 {
                     SPEED_high=gd_base_speed;
                     SPEED_VALUE=SPEED_high-(int)(SPEED_VALUE/4/(duty_max)*fabs(Servo_Duty));
                     if(SPEED_VALUE<1000) SPEED_VALUE=1000;
#if MOTOR_MODE
                      motor_loop(SPEED_VALUE);
#else
                      BLDC_motor_crtl(SPEED_VALUE);
#endif
                 Servo_Duty=PidIncCtrl(&GPS_PID,gd_azimuth);
                           Servo_Ctrl(Servo_Duty);

                 }

                           if(out_index>=_in_index)
                              {
                               BLDC_motor_crtl(0);
                                  Servo_Ctrl(0);
                                  save_run=0;
                              }
                 Display_showString(2, 1,"save", U16_WHITE, U16_BLACK, 12);
                 Display_showString(2, 3, "save_ok", U16_WHITE, U16_BLACK, 12);
                     Display_showString(2, 5,"run", U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "LAT:%5f", gyro_angle_totol);
                                       Display_showString(0, 9,txt, U16_WHITE, U16_BLACK, 12);
          sprintf(txt, "LAT:%5f", azimuth);
                             Display_showString(0, 11,txt, U16_WHITE, U16_BLACK, 12);
                        sprintf(txt, "LAT:%5f",     IMU_buf[in_index] );
                   Display_showString(0, 13,txt, U16_WHITE, U16_BLACK, 12);

             }
}
void control()//发车函数
{
    int8_t SET_NOW=0;//现在正在调节的参数
            int8_t SET_LAST=0;
        Display_showString(0, 1,"->", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 1,"Task1", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 3,"Task2", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 5,"Task2_GPS", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 7,"Task3", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 9,"GD_star", U16_WHITE, U16_BLACK, 12);
        while(1)
        {
            key_scan();
            if(KEY0_flag==1)//检测到KEY0按下，退出
               {
                 key_clean();
                 break;
               }
               else if(KEY1_flag==1)
               {
                   key_clean();
                  if(SET_NOW==4)
                 {
                      SET_NOW=0;
                 }
                 else
                 {
                     SET_NOW++;
                 }
               }
               else if(KEY2_flag==1)
               {
                   key_clean();
                   switch(SET_NOW)
                   {
                       case 0:task1_control();Display_showString(0, 7, "Task1", U16_WHITE, U16_BLACK, 12);Display_CLS(U16_BLACK);break;//任务1发车
                       case 1:task2_control();Display_showString(0, 7, "Task2", U16_WHITE, U16_BLACK, 12);Display_CLS(U16_BLACK);break;//任务2发车，融合惯导，不建议用
                       case 2:task2_control_GPS();Display_showString(0, 7, "Task2_GPS", U16_WHITE, U16_BLACK, 12);Display_CLS(U16_BLACK);break;//任务2发车，纯GPS
                       case 3:task3_control();Display_showString(0, 7, "Task3", U16_WHITE, U16_BLACK, 12);Display_CLS(U16_BLACK);break;//任务3发车
                       case 4:GD_star();Display_showString(0, 7, "GD_star", U16_WHITE, U16_BLACK, 12);break;//纯惯导
                   }
               }
            if(SET_LAST!=SET_NOW)
                    {
                        Display_CLS(U16_BLACK);
                               Display_showString(0, 1+(2*SET_NOW), "->", U16_WHITE, U16_BLACK, 12);
                               SET_LAST=SET_NOW;
                    }
            Display_showString(2, 1,"Task1", U16_WHITE, U16_BLACK, 12);
                Display_showString(2, 3,"Task2", U16_WHITE, U16_BLACK, 12);
                Display_showString(2, 5,"Task2_GPS", U16_WHITE, U16_BLACK, 12);
                Display_showString(2, 7,"Task3", U16_WHITE, U16_BLACK, 12);
                Display_showString(2, 9, "GD_star", U16_WHITE, U16_BLACK, 12);
        }
}
void start_deal()//偏差纠正
{
    max_num=last_point;
    char txt[30];
    double digit=0.000001;
    start_flag=1;
    SPEED_VALUE=0;
    V=speed_set-1000;
    BASE_V=base_speed-1000;
    check_flag=1;
    stop_flag=0;
    filter_flag=0;
    short_diatance_flag=0;

    int8_t SET_NOW=0;//现在正在调节的参数
               int8_t SET_LAST=0;
               sprintf(txt, "T:%07f", gyro_angle_totol);
 Display_showString(0, 7, txt, U16_WHITE, U16_BLACK, 16); // 时间
 Display_CLS(U16_WHITE);
 draw_gps_path();
           while(1)
           {
               //路线图纠正
               sprintf(txt, "%d", SET_NOW);
  Display_showString(1, 3, txt, U16_WHITE, U16_BLACK, 12); // 时间
               key_scan();
                     if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY0_flag==1)//检测到KEY0按下，退出
                     {
                       key_clean();
                       break;
                     }
                     else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY1_flag==1)
                     {
                         key_clean();
                        if(SET_NOW==max_num+2)
                       {
                           SET_NOW=0;
                           draw_gps_path();
                       }
                       else
                       {
                         SET_NOW++;
                         draw_gps_path();
                       }
                     }
                     else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY0_flag==1)
                     {
                         key_clean();

                     }
                     if(SWITCH0_FLAG==0&SWITCH1_FLAG==1&KEY0_flag==1)
                     {
                         key_clean();
                         if(SET_NOW==max_num+1)
                         {//科目二的路线图纠正
                             point[4+zhuitong][0]+=digit;
                             point[5+zhuitong][0]+=digit;
                             point[6+zhuitong][0]+=digit;
                             point[7+zhuitong][0]+=digit;
                             point[8+zhuitong][0]+=digit;
                             point[9+zhuitong][0]+=digit;
                         }
                         else if(SET_NOW==max_num+2)
                                           {
                                               point[1][0]+=digit;
                                               point[2][0]+=digit;
                                               point[3][0]+=digit;
                                               point[4][0]+=digit;
                                               point[5][0]+=digit;
                                               point[6][0]+=digit;
                                           }
                         else
                         { point[SET_NOW][0]+=digit;}
                         Display_CLS(U16_WHITE);
                         draw_gps_path();
                     }
                     else if(SWITCH0_FLAG==0&SWITCH1_FLAG==1&KEY1_flag==1)//拨码开关1指向ON时SWITCH0_FLAG为0
                     {
                         key_clean();
                         if(SET_NOW==max_num+1)
                                               {
                             point[4+zhuitong][0]-=digit;
                                                   point[5+zhuitong][0]-=digit;
                                                   point[6+zhuitong][0]-=digit;
                                                   point[7+zhuitong][0]-=digit;
                                                   point[8+zhuitong][0]-=digit;
                                                   point[9+zhuitong][0]-=digit;
                                               }
                         else if(SET_NOW==max_num+2)
                                           {
                                               point[1][0]-=digit;
                                               point[2][0]-=digit;
                                               point[3][0]-=digit;
                                               point[4][0]-=digit;
                                               point[5][0]-=digit;
                                               point[6][0]-=digit;
                                           }
                               else
                         {point[SET_NOW][0]-=digit;}
                         Display_CLS(U16_WHITE);
                         draw_gps_path();
                     }
                     if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY0_flag==1)
                     {
                         key_clean();
                         if(SET_NOW==max_num+1)
                                                                   {
                             point[4+zhuitong][1]+=digit;
                                                                       point[5+zhuitong][1]+=digit;
                                                                       point[6+zhuitong][1]+=digit;
                                                                       point[7+zhuitong][1]+=digit;
                                                                       point[8+zhuitong][1]+=digit;
                                                                       point[9+zhuitong][1]+=digit;
                                                                   }
                         else if(SET_NOW==max_num+2)
                                                        {
                                                            point[1][1]+=digit;
                                                            point[2][1]+=digit;
                                                            point[3][1]+=digit;
                                                            point[4][1]+=digit;
                                                            point[5][1]+=digit;
                                                            point[6][1]+=digit;
                                                        }
                         else
                         {
                             point[SET_NOW][1]+=digit;
                         }
                         Display_CLS(U16_WHITE);
                         draw_gps_path();
                     }
                     if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY1_flag==1)
                     {
                         key_clean();
                         if(SET_NOW==max_num+1)
                                                                                   {
                             point[4+zhuitong][1]-=digit;
                                                                                       point[5+zhuitong][1]-=digit;
                                                                                       point[6+zhuitong][1]-=digit;
                                                                                       point[7+zhuitong][1]-=digit;
                                                                                       point[8+zhuitong][1]-=digit;
                                                                                       point[9+zhuitong][1]-=digit;
                                                                                   }
                         else if(SET_NOW==max_num+2)
                                                        {
                                                            point[1][1]-=digit;
                                                            point[2][1]-=digit;
                                                            point[3][1]-=digit;
                                                            point[4][1]-=digit;
                                                            point[5][1]-=digit;
                                                            point[6][1]-=digit;
                                                        }
                         else
                         {
                             point[SET_NOW][1]-=digit;
                         }
                         Display_CLS(U16_WHITE);
                         draw_gps_path();
                     }
                     if(SET_LAST!=SET_NOW)
                     {
                         Display_CLS(U16_WHITE);
                                SET_LAST=SET_NOW;
                                draw_gps_path();
                     }

           }
           if(get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[0][0], point[0][1])<15)//偏差纠正
              {//加入一段保护程序，防止循迹中途突然改值，导致坐标乱套
                GPS_data.LAT_CORRECT=GPS_data.LAT-point[0][0];
                GPS_data.LON_CORRECT=GPS_data.LON-point[0][1];
                for(uint8 i=0;i<last_point+1;i++)
                {
                  point[i][0]+=GPS_data.LAT_CORRECT;
                  point[i][1]+=GPS_data.LON_CORRECT;
                }
                Display_showString(0, 21, "filter ok",U16_WHITE, U16_BLACK, 12);
              }
}
void start3_deal()//起始准备，偏差纠正，科目三纠正
{
    max_num=last_point;
    char txt[30];
    double digit=0.000001;
    start_flag=1;
    SPEED_VALUE=0;
    V=speed_set-1000;
    BASE_V=base_speed-1000;
    check_flag=1;
    stop_flag=0;
    filter_flag=0;
    short_diatance_flag=0;
//    if(get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[0][0], point[0][1])<15)
//    {//加入一段保护程序，防止循迹中途突然改值，导致坐标乱套
//      GPS_data.LAT_CORRECT=GPS_data.LAT-point[0][0];
//      GPS_data.LON_CORRECT=GPS_data.LON-point[0][1];
//      for(uint8 i=0;i<last_point+1;i++)
//      {
//        point[i][0]+=GPS_data.LAT_CORRECT;
//        point[i][1]+=GPS_data.LON_CORRECT;
//      }
//      Display_showString(0, 21, "filter ok",U16_WHITE, U16_BLACK, 12);
//    }
    int8_t SET_NOW=0;//现在正在调节的参数
               int8_t SET_LAST=0;
               sprintf(txt, "T:%07f", gyro_angle_totol);
 Display_showString(0, 7, txt, U16_WHITE, U16_BLACK, 16); // 时间
 Display_CLS(U16_WHITE);
 draw_gps_path();
           while(1)
           {
               sprintf(txt, "%d", SET_NOW);
  Display_showString(1, 3, txt, U16_WHITE, U16_BLACK, 12); // 时间
               key_scan();
                     if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY0_flag==1)//检测到KEY0按下，退出
                     {
                       key_clean();
                       break;
                     }
                     else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY1_flag==1)
                     {
                         key_clean();
                        if(SET_NOW==max_num+2)
                       {
                           SET_NOW=0;
                           draw_gps_path();
                       }
                       else
                       {
                         SET_NOW++;
                         draw_gps_path();
                       }
                     }
                     else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY0_flag==1)
                     {
                         key_clean();

                     }
                     if(SWITCH0_FLAG==0&SWITCH1_FLAG==1&KEY0_flag==1)
                     {//科目三纠正
                         key_clean();
                         if(SET_NOW==max_num+2)
                         {
                             point[6][0]+=digit;
                             point[7][0]+=digit;
                             point[8][0]+=digit;
                             point[9][0]+=digit;
                             point[10][0]+=digit;
                             point[11][0]+=digit;
                         }
                         else if(SET_NOW==max_num+1)
                                         {
                                             point[1][0]+=digit;
                                             point[2][0]+=digit;
                                             point[3][0]+=digit;
                                             point[4][0]+=digit;
                                             point[5][0]+=digit;

                                         }
                         else
                         { point[SET_NOW][0]+=digit;}
                         Display_CLS(U16_WHITE);
                         draw_gps_path();
                     }
                     else if(SWITCH0_FLAG==0&SWITCH1_FLAG==1&KEY1_flag==1)//拨码开关1指向ON时SWITCH0_FLAG为0
                     {
                         key_clean();
                         if(SET_NOW==max_num+2)
                                               {
                             point[6][0]-=digit;
                                                          point[7][0]-=digit;
                                                          point[8][0]-=digit;
                                                          point[9][0]-=digit;
                                                          point[10][0]-=digit;
                                                          point[11][0]-=digit;
                                               }
                         else if(SET_NOW==max_num+1)
                                                              {
                                                                  point[1][0]-=digit;
                                                                  point[2][0]-=digit;
                                                                  point[3][0]-=digit;
                                                                  point[4][0]-=digit;
                                                                  point[5][0]-=digit;

                                                              }
                               else
                         {point[SET_NOW][0]-=digit;}
                         Display_CLS(U16_WHITE);
                         draw_gps_path();
                     }
                     if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY0_flag==1)
                     {
                         key_clean();
                         if(SET_NOW==max_num+2)
                                                                   {
                             point[6][1]+=digit;
                                                          point[7][1]+=digit;
                                                          point[8][1]+=digit;
                                                          point[9][1]+=digit;
                                                          point[10][1]+=digit;
                                                          point[11][1]+=digit;
                                                                   }
                         else if(SET_NOW==max_num+1)
                                                              {
                                                                  point[1][1]+=digit;
                                                                  point[2][1]+=digit;
                                                                  point[3][1]+=digit;
                                                                  point[4][1]+=digit;
                                                                  point[5][1]+=digit;

                                                              }
                         else
                         {
                             point[SET_NOW][1]+=digit;
                         }
                         Display_CLS(U16_WHITE);
                         draw_gps_path();
                     }
                     if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY1_flag==1)
                     {
                         key_clean();
                         if(SET_NOW==max_num+2)
                                                                                   {
                             point[6][1]-=digit;
                                                                      point[7][1]-=digit;
                                                                      point[8][1]-=digit;
                                                                      point[9][1]-=digit;
                                                                      point[10][1]-=digit;
                                                                      point[11][1]-=digit;
                                                                                   }
                         else if(SET_NOW==max_num+1)
                                                                             {
                                                                                 point[1][1]-=digit;
                                                                                 point[2][1]-=digit;
                                                                                 point[3][1]-=digit;
                                                                                 point[4][1]-=digit;
                                                                                 point[5][1]-=digit;

                                                                             }
                         else
                         {
                             point[SET_NOW][1]-=digit;
                         }
                         Display_CLS(U16_WHITE);
                         draw_gps_path();
                     }
                     if(SET_LAST!=SET_NOW)
                     {
                         Display_CLS(U16_WHITE);
                                SET_LAST=SET_NOW;
                                draw_gps_path();
                     }

           }
           if(get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[0][0], point[0][1])<15)
              {//加入一段保护程序，防止循迹中途突然改值，导致坐标乱套
                GPS_data.LAT_CORRECT=GPS_data.LAT-point[0][0];
                GPS_data.LON_CORRECT=GPS_data.LON-point[0][1];
                for(uint8 i=0;i<last_point+1;i++)
                {
                  point[i][0]+=GPS_data.LAT_CORRECT;
                  point[i][1]+=GPS_data.LON_CORRECT;
                }
                Display_showString(0, 21, "filter ok",U16_WHITE, U16_BLACK, 12);
              }
}
void end_deal()
{


    filter_flag=1;
    speed_set=V+1000;
    base_speed=BASE_V+1000;


}
void out_deal()
{
    static uint8 last=0;

       motor_crtl(0);

        filter_flag=1;
        filter_point[0]=point[next_point][0];
        filter_point[1]=point[next_point][1];
        distance_max=get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[next_point][0], point[next_point][1]);
        check_flag=0;//循迹，标志位置0




}
void gps_track()//循迹代码
{
    char txt[30];
  switch(check_flag){
  case 2://比赛结束
    distance_target=get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[last_point][0], point[last_point][1]);
    if(distance_target<1.5){
        Servo_Ctrl(0);
#if MOTOR_MODE
     motor_loop(0);
     motor_crtl(0);
#else
     BLDC_motor_crtl(0);
#endif
      start_flag=0;
    }
    else
      End_Track(distance_target);
    break;
  case 1: //发车段

      Start_Track();//将车头朝向与gps正北坐标系统一

    break;
  case 0: //循迹段
    distance_target=get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[next_point][0], point[next_point][1]);
    switch_flag=1;
//    if(distance_target>distance_low)
//    {
      GPS_Track(distance_target,next_point);//正常循迹

    break;
  }
}
void gps3_track()
{
    char txt[30];
  switch(check_flag){
  case 2://比赛结束
    distance_target=get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[last_point][0], point[last_point][1]);
    if(distance_target<1.2){
        Servo_Ctrl(0);
#if MOTOR_MODE
     motor_loop(0);
     motor_crtl(0);
#else
     BLDC_motor_crtl(0);
#endif
      start_flag=0;
    }
    else
      End_Track(distance_target);
    break;
  case 1: //发车段

      Start_Track();

    break;
  case 0: //循迹段
    distance_target=get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[next_point][0], point[next_point][1]);
    switch_flag=1;
//    if(distance_target>distance_low)
//    {
      GPS3_Track(distance_target,next_point);

    break;
  }
}
void Switching_point_D()//切点(距离)判断
{
                   if(switch_flag==1&&distance_target<1.2)//距离小于1.2切点
                    {
                          next_point++;
                    }
}
void Switching_point_2()//切点(距离)判断
{
//static uint8_t gd_num=0;
                   if(switch_flag==1&&distance_target<0.85)//切点
                    {
                          next_point++;
                    }
//    switch(gd_num)
//    {case 0:if(next_point==3){out_index=0;gyro_angle_totol=0;gd_num++;}break;
//    case 1:if(next_point==8){out_index=0;gyro_angle_totol=0;gd_num++;}break;
//    case 2:break;
//    }

}
void Switching_point_3()//切点(距离)判断
{

    if(task3_flag==1&&(next_point==2||next_point==4||next_point==8||next_point==10))
    {
       if(next_point==2)
           num=1;
       else if(next_point==4)
                  num=2;
       else if(next_point==8)
                         num=3;
       else if(next_point==10)
                         num=4;
        kemu_flag=1;

    }
    else
    {
        kemu_flag=0;
    }
                   if(switch_flag==1&&distance_target<1)//切点
                    {
                          next_point++;
    //                      Buzzer_check(50,50);                                  // 自检完成

                    }

}

void task1_control()//科目1
{
    gyro_angle=0;
    use_angle=0;
    char txt[30];
start_deal();//先进行偏差纠正与路线图纠正
Display_CLS(U16_BLACK);
while(start_flag)
{
    BEEP_OFF;
    Delay_Ms(5);
gps_track();//循迹
sprintf(txt, "T:%05f", use_angle);
      Display_showString(0, 5, txt, U16_WHITE, U16_BLACK, 12); // 时间
      sprintf(txt, "T:%d", next_point);
            Display_showString(0, 7, txt, U16_WHITE, U16_BLACK, 12); // 时间
      sprintf(txt, "T:%05f", azimuth);
            Display_showString(0, 9, txt, U16_WHITE, U16_BLACK, 12);
            sprintf(txt, "T:%05f", GPS_data.angle);
                   Display_showString(0, 11, txt, U16_WHITE, U16_BLACK, 12);

if(next_point>(last_point-1))//最后一个点位时，停车
{
    switch_flag=0;
check_flag=2;
}
Switching_point_D();
}

}
void task2_control()
{
    uint8_t number=0;
       task3_flag=1;
       gyro_angle=0;
           use_angle=0;
           char txt[30];
       start3_deal();
       Display_CLS(U16_BLACK);
       while(start_flag)
       {
       //    printf("channel_data: %d, %d, %f, %f\n ",SPEED_VALUE,speed_set,MOTOR_PID.kp,MOTOR_PID.ki);
           gps3_track();
       sprintf(txt, "T:%05f", use_angle);
             Display_showString(0, 5, txt, U16_WHITE, U16_BLACK, 12); // 时间
             sprintf(txt, "T:%d", next_point);
                   Display_showString(0, 7, txt, U16_WHITE, U16_BLACK, 12); // 时间
             sprintf(txt, "T:%05f", azimuth);
                   Display_showString(0, 9, txt, U16_WHITE, U16_BLACK, 12);
                   sprintf(txt, "T:%05f", GPS_data.angle);
                          Display_showString(0, 11, txt, U16_WHITE, U16_BLACK, 12);
   //                       sprintf(txt, "T:%d",last_point);
   //                                         Display_showString(0, 13, txt, U16_WHITE, U16_BLACK, 12);
   //                                         if(next_point==3)
   //                                         {     encoder_totol=0;
   //                                         if(encoder_totol>3000)
   //                                         {
   //                                             gyro_m=GPS_data.angle-GPS_gyro_angle;
   //                                         }}
                                            if(next_point==5)
                                                                                 {
                                                encoder_totol=0;if(encoder_totol>20000)
                                                                                 {
                                                                                     gyro_m=GPS_data.angle-GPS_gyro_angle;//这里想用gps角度补偿imu角度
                                                                                 }}
   //                                         if(next_point==9)
   //                                                                              {  encoder_totol=0;if(encoder_totol>3000)
   //                                                                              {
   //                                                                                  gyro_m=GPS_data.angle-GPS_gyro_angle;
   //                                                                              }}
   //                                         if(next_point==11)
   //                                                                              {  encoder_totol=0;if(encoder_totol>3000)
   //                                                                              {
   //                                                                                  gyro_m=GPS_data.angle-GPS_gyro_angle;
   //                                                                              }}
   //                                         if(next_point==3)
   //                                         { gyro_angle=GPS_data.angle;}
   //                                         if(next_point==5)
   //                                                                              { gyro_angle=GPS_data.angle;}
   //                                         if(next_point==9)
   //                                                                              { gyro_angle=GPS_data.angle;}
   //                                         if(next_point==11)
   //                                                                              { gyro_angle=GPS_data.angle;}

       if(next_point>(last_point-1))
       {
       switch_flag=0;
       check_flag=2;
       }
       Switching_point_3();

       if(kemu_flag==1)
       {
           number = kemu[num-1];
           switch(number)
   {
       case 0: fp_flag=1;break;
       case 1:db_flag=1;break;
       case 2:qd_flag=1;break;
       case 3:cp_flag=1;break;
   }

       }
       }
//    CCU6_InitConfig(CCU61,CCU6_Channel1,1);
//    gyro_angle=0;
//    use_angle=0;
//    char txt[30];
//start_deal();
//Display_CLS(U16_BLACK);
//while(start_flag)
//{
//    if(next_point==3||next_point==7)
//                    {
//        save_run=1;
//        distance_target=get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[next_point][0], point[next_point][1]);
//                        SPEED_high=gd_base_speed;
//                        SPEED_VALUE=SPEED_high-(int)(SPEED_VALUE/4/(duty_max)*fabs(Servo_Duty));
//                        if(SPEED_VALUE<1000) SPEED_VALUE=1000;
//#if MOTOR_MODE
//     motor_loop(SPEED_VALUE);
//#else
//     BLDC_motor_crtl(SPEED_VALUE);
//#endif
//                    Servo_Duty=PidIncCtrl(&GPS_PID,gd_azimuth);
//                              Servo_Ctrl(Servo_Duty);
//                              distance_target=get_two_points_distance (GPS_data.LAT, GPS_data.LON, point[next_point][0], point[next_point][1]);
//
//                    }
//    else
//    {
//       gps_track();
//    }
//if(next_point==4)
//{save_run=0;
////gyro_m=GPS_data.angle-GPS_gyro_angle;
//encoder_totol=0;
//if(encoder_totol>3000)
//{
//    gyro_m=GPS_data.angle-GPS_gyro_angle;
//}
//}
//if(next_point==5)
//{
////    gyro_m=GPS_data.angle-GPS_gyro_angle;
//    encoder_totol=0;
//    if(encoder_totol>3000)
//    {
//        gyro_m=GPS_data.angle-GPS_gyro_angle;
//    }
//}
//sprintf(txt, "T:%05f", use_angle);
//      Display_showString(0, 5, txt, U16_WHITE, U16_BLACK, 12); // 时间
//      sprintf(txt, "T:%d", next_point);
//            Display_showString(0, 7, txt, U16_WHITE, U16_BLACK, 12); // 时间
//      sprintf(txt, "T:%05f", azimuth);
//            Display_showString(0, 9, txt, U16_WHITE, U16_BLACK, 12);
//            sprintf(txt, "T:%05f", GPS_data.angle);
//                   Display_showString(0, 11, txt, U16_WHITE, U16_BLACK, 12);
////                   sprintf(txt, "T:%d",last_point);
////                                     Display_showString(0, 13, txt, U16_WHITE, U16_BLACK, 12);
//
//
//if(next_point>(last_point-1))
//{
//    switch_flag=0;
//check_flag=2;
//}
//Switching_point_2();
//}
}
void task2_control_GPS()//科目2
{
//    gyro_angle=0;
//    use_angle=0;
    task2_flag=1;
    char txt[30];
start_deal();
Display_CLS(U16_BLACK);
while(start_flag)
{
    key_scan();
        if(KEY0_flag==1)//检测到KEY0按下，退出
           {
             key_clean();
             break;
           }
       gps_track();
//sprintf(txt, "T:%05f", use_angle);
//      Display_showString(0, 5, txt, U16_WHITE, U16_BLACK, 12); // 时间
//      sprintf(txt, "T:%d", next_point);
//            Display_showString(0, 7, txt, U16_WHITE, U16_BLACK, 12); // 时间
//      sprintf(txt, "T:%05f", azimuth);
//            Display_showString(0, 9, txt, U16_WHITE, U16_BLACK, 12);

if(next_point==3+zhuitong)
{ encoder_totol=0;
if(encoder_totol>3000)
{
    gyro_m=GPS_data.angle-GPS_gyro_angle;
}
}

if(next_point==4+zhuitong)
{ encoder_totol=0;
if(encoder_totol>3000)
{
    gyro_m=GPS_data.angle-GPS_gyro_angle;
}
}
//            if(next_point==3+zhuitong)
//            { gyro_angle=GPS_data.angle;}
//            if(next_point==4+zhuitong)
//            { gyro_angle=GPS_data.angle;}
if(next_point>(last_point-1))
{
    switch_flag=0;
check_flag=2;
}
if(next_point==1||next_point==3+zhuitong||next_point==4+zhuitong||next_point==10+zhuitong)
{
    Switching_point_D();
}
else
{
    Switching_point_2();
}

}
}
void task3_control()//0是飞坡，1是颠簸路段，2是桥洞，3是人工草坪
{
    uint8_t number=0;
    task3_flag=1;
    gyro_angle=0;
        use_angle=0;
        char txt[30];
    start3_deal();
    Display_CLS(U16_BLACK);
    while(start_flag)
    {
    //    printf("channel_data: %d, %d, %f, %f\n ",SPEED_VALUE,speed_set,MOTOR_PID.kp,MOTOR_PID.ki);
    gps_track();
    sprintf(txt, "T:%05f", use_angle);
          Display_showString(0, 5, txt, U16_WHITE, U16_BLACK, 12); // 时间
          sprintf(txt, "T:%d", next_point);
                Display_showString(0, 7, txt, U16_WHITE, U16_BLACK, 12); // 时间
          sprintf(txt, "T:%05f", azimuth);
                Display_showString(0, 9, txt, U16_WHITE, U16_BLACK, 12);
                sprintf(txt, "T:%05f", GPS_data.angle);
                       Display_showString(0, 11, txt, U16_WHITE, U16_BLACK, 12);
//                       sprintf(txt, "T:%d",last_point);
//                                         Display_showString(0, 13, txt, U16_WHITE, U16_BLACK, 12);
//                                         if(next_point==3)
//                                         {     encoder_totol=0;
//                                         if(encoder_totol>3000)
//                                         {
//                                             gyro_m=GPS_data.angle-GPS_gyro_angle;
//                                         }}
                                         if(next_point==5)
                                                                              {
                                             encoder_totol=0;if(encoder_totol>3000)
                                                                              {
                                                                                  gyro_m=GPS_data.angle-GPS_gyro_angle;
                                                                              }}
//                                         if(next_point==9)
//                                                                              {  encoder_totol=0;if(encoder_totol>3000)
//                                                                              {
//                                                                                  gyro_m=GPS_data.angle-GPS_gyro_angle;
//                                                                              }}
//                                         if(next_point==11)
//                                                                              {  encoder_totol=0;if(encoder_totol>3000)
//                                                                              {
//                                                                                  gyro_m=GPS_data.angle-GPS_gyro_angle;
//                                                                              }}
//                                         if(next_point==3)
//                                         { gyro_angle=GPS_data.angle;}
//                                         if(next_point==5)
//                                                                              { gyro_angle=GPS_data.angle;}
//                                         if(next_point==9)
//                                                                              { gyro_angle=GPS_data.angle;}
//                                         if(next_point==11)
//                                                                              { gyro_angle=GPS_data.angle;}

    if(next_point>(last_point-1))
    {
    switch_flag=0;
    check_flag=2;
    }
    Switching_point_3();

    if(kemu_flag==1)
    {
        number = kemu[num-1];
        switch(number)
{
    case 0: fp_flag=1;break;
    case 1:db_flag=1;break;
    case 2:qd_flag=1;break;
    case 3:cp_flag=1;break;
}

    }
    }

}
void task4_control()
{
    char text_buffer[5];
    while(text_flag)
    {


    }
}
