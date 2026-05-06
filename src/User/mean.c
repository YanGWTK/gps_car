/*
 * mean.c
 *
 *  Created on: 2025年3月1日
 *      Author: 29500
 */
#include "include.h"
int  GO_FLAG=0;       //发车标志位
int  STOP_MENU_FALG=1;//菜单标志位
int  func_index = 0; //初始显示欢迎界面
int  last_index = 127; //last初始为无效值
void (*current_operation_index)(void);

key_table table_dispaly[30]=                 //结构体数组
{
    //第0层
    {0,0,0,1,(*fun_0)},                     //AIIT_meun

    //第1层
    {1,4,2, 5,(*fun_a1)},                   //GPS
    {2,1,3, 9,(*fun_b1)},                   //PID
    {3,2,4,13,(*fun_c1)},                   //SPEED
    {4,3,1, 0,(*fun_d1)},                   //ESC

    //第2层
    {5,8,6,17,(*fun_a21)},
    {6,5,7,18,(*fun_a22)},
    {7,6,8,19,(*fun_a23)},
    {8,7,5, 1,(*fun_a24)},

    { 9,12,10,20,(*fun_b21)},
    {10, 9,11,21,(*fun_b22)},
    {11,10,12,22,(*fun_b23)},
    {12,11, 9, 2,(*fun_b24)},

    {13,16,14,23,(*fun_c21)},
    {14,13,15,24,(*fun_c22)},
    {15,14,16,25,(*fun_c23)},
    {16,15,13, 3,(*fun_c24)},

    //第3层
    {17,17,17,5,(*fun_a31)},
    {18,18,18,6,(*fun_a32)},
    {19,19,19,7,(*fun_a33)},

    {20,20,20, 9,(*fun_b31)},
    {21,21,21,10,(*fun_b32)},
    {22,22,22,11,(*fun_b33)},

    {23,23,23,13,(*fun_c31)},
    {24,24,24,14,(*fun_c32)},
    {25,25,25,15,(*fun_c33)},
};
void Menu()//菜单函数
{

    key_scan();

                if(KEY0_flag)
                {
                    key_clean();
                    func_index = table_dispaly[func_index].up;    //向上翻
                }
                if(KEY1_flag)
                {
                    key_clean();
                    func_index = table_dispaly[func_index].down;    //向下翻

                }
                if(KEY2_flag)
                {
                    key_clean();
                    func_index = table_dispaly[func_index].enter;    //确认

                }


            if (func_index != last_index)
            {
                current_operation_index = table_dispaly[func_index].current_operation;

                Display_CLS(U16_BLACK);
                (*current_operation_index)();//执行当前操作函数
                last_index = func_index;

            }
  }
void show_num()
{
    char txt[30];
    while(1)
    {
        key_scan();
        if(KEY0_flag)
        {
            key_clean();
            break;
        }
                     sprintf(txt, "LA_NO:%f", (float)GPS_data.LAT);
                     Display_showString(0, 1,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "LO_NO:%f", (float)GPS_data.LON);
                     Display_showString(0, 3,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "gyroz:%05f", (float)gyro_angle);
                     Display_showString(0, 5,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "GPS_ang:%05f", (float)GPS_data.angle);
                     Display_showString(0, 7,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "speed:%04d", speed_set);
                     Display_showString(0, 9,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "GPS_kp:%.3f", GPS_PID.kp);
                     Display_showString(0, 11, txt, U16_WHITE, U16_BLACK, 12);

                     // 显示 GPS_kd
                     sprintf(txt, "GPS_kd:%.3f", GPS_PID.kd);
                     Display_showString(0, 13, txt, U16_WHITE, U16_BLACK, 12);

                     #if MOTOR_MODE
                     // 显示 MOTOR_kp
                     sprintf(txt, "MOR_kp:%.3f", MOTOR_PID.kp);
                     Display_showString(0, 15, txt, U16_WHITE, U16_BLACK, 12);

                     // 显示 MOTOR_ki
                     sprintf(txt, "MOR_ki:%.3f", MOTOR_PID.ki);
                     Display_showString(0, 17, txt, U16_WHITE, U16_BLACK, 12);

                     // 显示 MOTOR_kd
                     sprintf(txt, "MOR_kd:%.3f", MOTOR_PID.kd);
                     Display_showString(0, 19, txt, U16_WHITE, U16_BLACK, 12);
                     #else
                     // 显示 MOTOR_BRUSH_kp
                     sprintf(txt, "MO_BR_kp:%.3f",MOTOR_BRUSH_PID.kp);
                     Display_showString(0, 15, txt, U16_WHITE, U16_BLACK, 12);

                     // 显示 MOTOR_BRUSH_ki
                     sprintf(txt, "MO_BR_ki:%.3f", MOTOR_BRUSH_PID.ki);
                     Display_showString(0, 17, txt, U16_WHITE, U16_BLACK, 12);

                     // 显示 MOTOR_BRUSH_kd
                     sprintf(txt, "MO_BR_kd:%.3f", MOTOR_BRUSH_PID.kd);
                     Display_showString(0, 19, txt, U16_WHITE, U16_BLACK, 12);
                     #endif
    }

}
void motor_test()
{

    char txt[30];
int16_t speed;
    while(1)
    {
        key_scan();
                if(KEY0_flag==1)//检测到KEY0按下，退出
                   {
                     key_clean();
                     break;
                   }
                if(KEY1_flag==1)//检测到KEY0按下，退出
                                                 {
                                                   key_clean();
                                                   speed+=100;
                                                 }
                               if(KEY2_flag==1)//检测到KEY0按下，退出
                                                 {
                                                   key_clean();
                                                   speed-=100;
                                                 }
//                angle_target=40;
//            azimuth = angle_target - gyro_angle;
//            angle_plan(&azimuth);
//            Servo_Duty=PidLocCtrl(&GPS_PID, azimuth);
//            motor_crtl(speed);
sprintf(txt, "T:%05d", speed);
Display_showString(0, 1, txt, U16_WHITE, U16_BLACK, 16);
    }
}
void servo_test()
{

    char txt[30];
    int16_t angle2=0;
    while(1)
    {
//        key_scan();
//        if(KEY0_flag==1)//检测到KEY0按下，退出
//                         {
//                            key_clean();
//                            break;
//                          }
//                if(KEY1_flag==1)//检测到KEY0按下，退出
//                                  {
//                                    key_clean();
//                                    angle2+=1;
//                                  }
//                if(KEY2_flag==1)//检测到KEY0按下，退出
//                                  {
//                                    key_clean();
//                                    angle2-=1;
//                                  }
//                Servo_Ctrl(angle2);
//                sprintf(txt, "T:%05d", angle2);
//                Display_showString(0, 1, txt, U16_WHITE, U16_BLACK, 16);}

        key_scan();
                if(KEY0_flag==1)//检测到KEY0按下，退出
                   {
                     key_clean();
                     break;
                   }
                angle_target=0;
//                angle_target=get_two_points_azimuth(GPS_data.LAT, GPS_data.LON, point[0][0], point[0][1]);
            azimuth = angle_target - gyro_angle;
            angle_plan(&azimuth);
            Servo_Duty=PidLocCtrl(&GPS_PID, azimuth);
            Servo_Ctrl(Servo_Duty);
//            motor_crtl(2000);
sprintf(txt, "T:%05f", azimuth);
Display_showString(0, 1, txt, U16_WHITE, U16_BLACK, 16);}
    }





//void  Menu_flash_clear(int PAGE_INDEX)//清除FLASH
//{
//    EEPROM_EraseSector(PAGE_INDEX);
//    Display_showString(0, 2, "Clear FLASH OK!", U16_WHITE, U16_BLACK, 12);  // 显示清除成功信息
//
//        if (PAGE_INDEX == 1)
//        {
//            Display_showString(0, 4, "SPEED_PAGE_INDEX(10)-->", U16_WHITE, U16_BLACK, 12);  // 显示速度页信息
//        }
//        else if (PAGE_INDEX == 2)
//        {
//            Display_showString(0, 4, "Stop_Point_PAGE_INDEX(8)-->", U16_WHITE, U16_BLACK, 12);  // 显示停止点页信息
//        }
//        else if (PAGE_INDEX == 3)
//        {
//            Display_showString(0, 4, "Point_Dis_PAGE_INDEX(7)-->", U16_WHITE, U16_BLACK, 12);  // 显示点位距离页信息
//        } else {
//            // 其他情况不显示
//        }
//
//        printf("\r\n已擦除指定页FLASH:%d", PAGE_INDEX);  // 打印日志
//}
///*********第0层***********/
void fun_0()
{
    Display_showString(0, 0, "start", U16_WHITE, U16_BLACK, 16);
}
///*********第1层***********/
    void fun_a1() {
        Display_showString(0, 1, "->", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 1, "GPS_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 3, "Parameter_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 5, "Other_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 7, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_b1() {
        Display_showString(0, 3, "->", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 1, "GPS_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 3, "Parameter_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 5, "Other_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 7, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_c1() {
        Display_showString(0, 5, "->", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 1, "GPS_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 3, "Parameter_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 5, "Other_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 7, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_d1() {
        Display_showString(0, 7, "->", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 1, "GPS_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 3, "Parameter_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 5, "Other_SET", U16_WHITE, U16_BLACK, 12);
          Display_showString(3, 7, "ESC", U16_WHITE, U16_BLACK, 12);
    }
    ///*********第1层***********/

    ///*********第2层***********/
    void fun_a21() {  // GPS_SET
        Display_showString(0, 2, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "GPS storage point", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "Com_display", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "servo", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_a22() {
        Display_showString(0, 4, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "GPS storage point", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "Com_display", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "servo", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_a23() {
        Display_showString(0, 6, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "GPS storage point", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "Com_display", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "servo", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_a24() {
        Display_showString(0, 8, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "GPS storage point", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "Com_display", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "servo", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }
    void fun_b21() {  // Parameter_SET
        Display_showString(0, 2, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "PID_SET", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "SPEED_SET", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "CLEAR_PAT_FLASH", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_b22() {
        Display_showString(0, 4, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "PID_SET", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "SPEED_SET", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "CLEAR_PAT_FLASH", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_b23() {
        Display_showString(0, 6, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "PID_SET", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "SPEED_SET", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "CLEAR_PAT_FLASH", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_b24() {
        Display_showString(0, 8, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "PID_SET", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "SPEED_SET", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "CLEAR_PAT_FLASH", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }
    void fun_c21() {  // Other_SET
        Display_showString(0, 2, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "START_CAR", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "Flash_Get", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "kemu", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_c22() {
        Display_showString(0, 4, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "START_CAR", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "Flash_Get", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "kemu", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_c23() {
        Display_showString(0, 6, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "START_CAR", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "Flash_Get", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "kemu", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }

    void fun_c24() {
        Display_showString(0, 8, "->", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 2, "START_CAR", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 4, "Flash_Get", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 6, "kemu", U16_WHITE, U16_BLACK, 12);
        Display_showString(3, 8, "ESC", U16_WHITE, U16_BLACK, 12);
    }
    ///*********第2层***********/

    ///*********第3层***********/
    void fun_a31()  //GPS点位采集并存入对应FLASH
    {
     GPS_point_get();//gps打点
     func_index=5;
    }
    void fun_a32()
    {
     show_num();//显示当前所有信息
     func_index=5;
    }
    void fun_a33()//pid
    {
servo_test();
     func_index=5;
    }
    void fun_b31()
    {
PID_SET();//pid菜单设置
func_index=9;
    }
    void fun_b32()
    {
SPEED_SET();//速度设置
func_index=9;
    }
    void fun_b33()
    {
        mean_flash_clear();//清楚flash内的数据，改变flash数据前一定要清楚
        func_index=9;
    }
    void fun_c31()
    {
        control();//发车控制

        func_index=9;


    }
    void fun_c32()
    {
flash_get();//获得flash内的数据
func_index=9;
    }
    void fun_c33()
    {
kemu3_SET();//设置科目三的元素顺序
        func_index=9;
    }

