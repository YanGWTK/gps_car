/*
 * eeprom.c
 *
 *  Created on: 2024年12月29日
 *      Author: 29500
 */
#include "include.h"
#define MAX_Dot 50
Position_t T_XY;
uint32_t Flash_buf[MAX_Dot*4+1];                          //转换类型的数据，用于存入flash中
float PID_buf[5];
int16_t speed_buf[3];
float IMU_buf[2000];
uint8_t num_dot;
uint8_t kemu_dot=0;
unsigned long long Flash_ValX = 0, Flash_ValY = 0;    //转换类型的时候的中间缓存
float yaw_gd[2000];
uint8_t kemu_buf[4];
uint16_t max_num=0;//定了多少个点
float distance_test;
double angle_test;
void gps_write()
{  memset(Flash_buf,0xff,MAX_Dot*4+1);
    for(num_dot=0;num_dot<=MAX_Dot;num_dot++)
    {
        T_XY.lat = point[num_dot][0];
T_XY.lon = point[num_dot][1];
//放入数组中，拆成32位，方便放入flash中 此时还没写入，等采集完成后集中写入
Flash_ValX = *(unsigned long long*)&T_XY.lat;
Flash_ValY = *(unsigned long long*)&T_XY.lon;

Flash_buf[0] = (unsigned long long)max_num;
Flash_buf[num_dot*4+1] = Flash_ValX & 0xFFFFFFFF;
Flash_buf[num_dot*4+2] = Flash_ValX >> 32;
Flash_buf[num_dot*4+3] = Flash_ValY & 0xFFFFFFFF;
Flash_buf[num_dot*4+4] = Flash_ValY >> 32;
    }

/*采集点加一*/
if(kemu_dot==0)
{
    EEPROM_Write(5, 1, Flash_buf, MAX_Dot*4+1);
}
if(kemu_dot==1)
{
    EEPROM_Write(7, 0, Flash_buf, MAX_Dot*4+1);
}
if(kemu_dot==2)
{
    EEPROM_Write(8, 0, Flash_buf, MAX_Dot*4+1);
}
Display_showString(0, 19, "already save", U16_WHITE, U16_BLACK, 12);
}
void GPS_point_get()
{
    max_num=0;
    char txt[30];
  uint8 num=0;
  double lat=0;
  double lon=0;

  uint8 change_flag=0;//检测此次执行该函数后是否改变点位信息
  sprintf(txt, "NOW POINT:%d", (uint32)num);
                 Display_showString(0, 1,txt, U16_WHITE, U16_BLACK, 12);
                 sprintf(txt, "LAT:%f", (float)point[num][0]);
                 Display_showString(0, 3,txt, U16_WHITE, U16_BLACK, 12);
                 sprintf(txt, "LON:%f", (float)point[num][1]);
                 Display_showString(0, 5,txt, U16_WHITE, U16_BLACK, 12);
                 sprintf(txt, "NOW:%f", (float)GPS_data.LAT);
                 Display_showString(0, 7,txt, U16_WHITE, U16_BLACK, 12);
                 sprintf(txt, "NOW:%f", (float)GPS_data.LON);
                 Display_showString(0, 9,txt, U16_WHITE, U16_BLACK, 12);
                 sprintf(txt, "kemu:%d", kemu_dot);
                                               Display_showString(0, 15,txt, U16_WHITE, U16_BLACK, 12);
//  ips200_show_string(0,  16*0, "NOW POINT: ");
//  ips200_show_uint (8*13, 16*0, (uint32)num, 2);
//  ips200_show_string(0,  16*3, "LAT: ");
//  ips200_show_float (0, 16*4, (float)point[num][0], 2, 6);
//  ips200_show_string(0,  16*5, "LON: ");
//  ips200_show_float (0, 16*6, (float)point[num][1], 3, 6);
//  ips200_show_string(0,  16*11, "LAT_NOW: ");
//  ips200_show_float (0, 16*12, (float)GPS_data.LAT, 2, 6);
//  ips200_show_string(0,  16*13, "LON_NOW: ");
//  ips200_show_float (0, 16*14, (float)GPS_data.LON, 3, 6);
  while(1)
  {

    key_scan();
    if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY0_flag==1)//检测到KEY0按下，退出
    {
      key_clean();
      break;
    }
    else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY1_flag==1)
    {
        key_clean();
       if(num==15)
      {
          num=0;
      }
      else
      {
        num++;
      }
    }
    else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY2_flag==1)
    {
        key_clean();//清除标志
        lat=0;
          lon=0;
          for(uint8 z=0;z<5;z++)
          {
            Delay_Ms(102);
            lat+=GPS_data.LAT;
            lon+=GPS_data.LON;
          }
          lat/=5.00;
          lon/=5.00;
          point[num][0]=lat;//存放纬度
          point[num][1]=lon;//存放经度
          change_flag=1;//检测到点位信息改变
    }
    if(SWITCH0_FLAG==0&SWITCH1_FLAG==1&KEY2_flag==1)
    {
        key_clean();
        kemu_dot++;
        if(kemu_dot>2)
            kemu_dot=0;
    }
    distance_test=get_two_points_distance(point[num][0],point[num][1],point[num+1][0],point[num+1][1]);
    angle_test=get_two_points_azimuth(point[num][0],point[num][1],point[num+1][0],point[num+1][1]);
                     sprintf(txt, "NOW POINT:%02d", (uint32)num);
                     Display_showString(0, 1,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "LAT:%f", (float)point[num][0]);
                     Display_showString(0, 3,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "LON:%f", (float)point[num][1]);
                     Display_showString(0, 5,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "NOW:\n%f", GPS_data.LAT);
                     Display_showString(0, 7,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "NOW:\n%f", GPS_data.LON);
                     Display_showString(0, 9,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "dis:\n%f", distance_test);
                     Display_showString(0, 11,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "angle:\n%f", angle_test);
                     Display_showString(0, 13,txt, U16_WHITE, U16_BLACK, 12);
                     sprintf(txt, "kemu:%d", kemu_dot);
                               Display_showString(0, 15,txt, U16_WHITE, U16_BLACK, 12);

  }
  max_num = num;
  if(change_flag==1)////////跳出循环代表采点的结束，但该函数工作并未完成，退出后存储点位
  {
      gps_write();
  }
}
void point_get()
{
    EEPROM_Read(5, 1, Flash_buf, MAX_Dot*4+1);
    num_dot=Flash_buf[0]+1;
    last_point=Flash_buf[0];
    for(uint8 i =0;i<num_dot;i++)
    {
           Flash_ValX = Flash_buf[i*4+2];
           Flash_ValX = Flash_ValX<<32 | Flash_buf[i*4+1];
           Flash_ValY = Flash_buf[i*4+4];
           Flash_ValY = Flash_ValY<<32 | Flash_buf[i*4+3];
        point[i][0]=*(double*)&Flash_ValX;//x是纬度
        point[i][1]=*(double*)&Flash_ValY;//y是经度
    }
}
void point2_get()
{
    EEPROM_Read(7, 0, Flash_buf, MAX_Dot*4+1);
    num_dot=Flash_buf[0]+1;
    last_point=Flash_buf[0];
    for(uint8 i =0;i<num_dot;i++)
    {
           Flash_ValX = Flash_buf[i*4+2];
           Flash_ValX = Flash_ValX<<32 | Flash_buf[i*4+1];
           Flash_ValY = Flash_buf[i*4+4];
           Flash_ValY = Flash_ValY<<32 | Flash_buf[i*4+3];
        point[i][0]=*(double*)&Flash_ValX;//x是纬度
        point[i][1]=*(double*)&Flash_ValY;//y是经度
    }
}
void point3_get()
{
    EEPROM_Read(8, 0, Flash_buf, MAX_Dot*4+1);
    num_dot=Flash_buf[0]+1;
    last_point=Flash_buf[0];
    for(uint8 i =0;i<num_dot;i++)
    {
           Flash_ValX = Flash_buf[i*4+2];
           Flash_ValX = Flash_ValX<<32 | Flash_buf[i*4+1];
           Flash_ValY = Flash_buf[i*4+4];
           Flash_ValY = Flash_ValY<<32 | Flash_buf[i*4+3];
        point[i][0]=*(double*)&Flash_ValX;//x是纬度
        point[i][1]=*(double*)&Flash_ValY;//y是经度
    }
}
void PID_PARA_Init()
{

  PidInit(&GPS_PID);
  PidInit(&MOTOR_PID);
  PidInit(&MOTOR_BRUSH_PID);
  PidInit(&ICM_PID);
  EEPROM_Read(1, 1, PID_buf, 5);
  GPS_PID.kp=PID_buf[0];
  GPS_PID.kd=PID_buf[1];
#if MOTOR_MODE
  MOTOR_PID.kp=PID_buf[2];
  MOTOR_PID.ki=PID_buf[3];
  MOTOR_PID.kd=PID_buf[4];
#else
  MOTOR_BRUSH_PID.kp=PID_buf[2];
  MOTOR_BRUSH_PID.ki=PID_buf[3];
  MOTOR_BRUSH_PID.kd=PID_buf[4];
#endif
}
void PID_SET()
{
    char txt[30];

#if MOTOR_MODE
  float para_initial[5]={GPS_PID.kp,GPS_PID.kd,MOTOR_PID.kp,MOTOR_PID.ki,MOTOR_PID.kd};
#else
  float para_initial[5]={GPS_PID.kp,GPS_PID.kd,MOTOR_BRUSH_PID.kp,MOTOR_BRUSH_PID.ki,MOTOR_BRUSH_PID.kd};
#endif
  float digit=0.001;//默认按0.001调节
    int8_t SET_NOW=0;//现在正在调节的参数
    int8_t SET_LAST=0;
    sprintf(txt, "digit:%.3f", digit);
        Display_showString(2, 1, txt, U16_WHITE, U16_BLACK, 12);
        Display_showString(0, 3, "->", U16_WHITE, U16_BLACK, 12);
//    ips200_show_string(0*8,0*16,"digit:  ");
//    ips200_show_float(10*8,0*16,digit,3,3);
//    ips200_show_string(27*8,3*16,"<- ");
  // 显示 GPS_kp
    sprintf(txt, "GPS_kp:%.3f", GPS_PID.kp);
    Display_showString(2, 3, txt, U16_WHITE, U16_BLACK, 12);

    // 显示 GPS_kd
    sprintf(txt, "GPS_kd:%.3f", GPS_PID.kd);
    Display_showString(2, 5, txt, U16_WHITE, U16_BLACK, 12);

    #if MOTOR_MODE
    // 显示 MOTOR_kp
    sprintf(txt, "MOR_kp:%.3f", MOTOR_PID.kp);
    Display_showString(2, 7, txt, U16_WHITE, U16_BLACK, 12);

    // 显示 MOTOR_ki
    sprintf(txt, "MOR_ki:%.3f", MOTOR_PID.ki);
    Display_showString(2, 9, txt, U16_WHITE, U16_BLACK, 12);

    // 显示 MOTOR_kd
    sprintf(txt, "MOR_kd:%.3f", MOTOR_PID.kd);
    Display_showString(2, 11, txt, U16_WHITE, U16_BLACK, 12);
    #else
    // 显示 MOTOR_BRUSH_kp
    sprintf(txt, "MO_BR_kp:%.3f", MOTOR_BRUSH_PID.kp);
    Display_showString(2, 7, txt, U16_WHITE, U16_BLACK, 12);

    // 显示 MOTOR_BRUSH_ki
    sprintf(txt, "MO_BR_ki:%.3f", MOTOR_BRUSH_PID.ki);
    Display_showString(2, 9, txt, U16_WHITE, U16_BLACK, 12);

    // 显示 MOTOR_BRUSH_kd
    sprintf(txt, "MO_BR_kd:%.3f", MOTOR_BRUSH_PID.kd);
    Display_showString(2, 11, txt, U16_WHITE, U16_BLACK, 12);
    #endif
    while(1)
      {
        key_scan();
        if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY0_flag==1)//检测到KEY0按下，退出
        {
          key_clean();
          break;
        }
        else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY1_flag==1)
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
        else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY2_flag==1)
        {
            key_clean();

        }
        if(SWITCH0_FLAG==0&SWITCH1_FLAG==1&KEY0_flag==1)
        {
            key_clean();
            digit/=10;
        }
        else if(SWITCH0_FLAG==0&SWITCH1_FLAG==1&KEY1_flag==1)//拨码开关1指向ON时SWITCH0_FLAG为0
        {
            key_clean();
            digit*=10;
        }
        if(digit<0.001)
        {
            digit=0.001;
        }
        if(SET_LAST!=SET_NOW)
        {
            Display_CLS(U16_WHITE);
                   Display_showString(0, 3+(2*SET_NOW), "->", U16_WHITE, U16_BLACK, 12);
                   SET_LAST=SET_NOW;
        }
        if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY0_flag==1)
        {
            key_clean();
            para_initial[SET_NOW]+=digit;
        }
        if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY1_flag==1)
        {
            key_clean();
            para_initial[SET_NOW]-=digit;
        }
//        printf("channel_data: %d, %d, %f, %f\n ",encoder,speed_set,MOTOR_PID.kp,MOTOR_PID.ki);
//                motor_loop(speed_set);
        sprintf(txt, "digit:%.3f", digit);
               Display_showString(2, 1, txt, U16_WHITE, U16_BLACK, 12);
               sprintf(txt, "GPS_kp:%.3f", para_initial[0]);
               Display_showString(2, 3, txt, U16_WHITE, U16_BLACK, 12);

               // 显示 GPS_kd
               sprintf(txt, "GPS_kd:%.3f", para_initial[1]);
               Display_showString(2, 5, txt, U16_WHITE, U16_BLACK, 12);

               #if MOTOR_MODE
               // 显示 MOTOR_kp
               sprintf(txt, "MOR_kp:%.3f", para_initial[2]);
               Display_showString(2, 7, txt, U16_WHITE, U16_BLACK, 12);

               // 显示 MOTOR_ki
               sprintf(txt, "MOR_ki:%.3f", para_initial[3]);
               Display_showString(2, 9, txt, U16_WHITE, U16_BLACK, 12);

               // 显示 MOTOR_kd
               sprintf(txt, "MOR_kd:%.3f", para_initial[4]);
               Display_showString(2, 11, txt, U16_WHITE, U16_BLACK, 12);
               #else
               // 显示 MOTOR_BRUSH_kp
               sprintf(txt, "MO_BR_kp:%.3f", para_initial[2]);
               Display_showString(2, 7, txt, U16_WHITE, U16_BLACK, 12);

               // 显示 MOTOR_BRUSH_ki
               sprintf(txt, "MO_BR_ki:%.3f", para_initial[3]);
               Display_showString(2, 9, txt, U16_WHITE, U16_BLACK, 12);

               // 显示 MOTOR_BRUSH_kd
               sprintf(txt, "MO_BR_kd:%.3f", para_initial[4]);
               Display_showString(2, 11, txt, U16_WHITE, U16_BLACK, 12);
               #endif
               GPS_PID.kp=para_initial[0];
               GPS_PID.kd=para_initial[1];
             #if MOTOR_MODE
               MOTOR_PID.kp=para_initial[2];
               MOTOR_PID.ki=para_initial[3];
               MOTOR_PID.kd=para_initial[4];
             #else
               MOTOR_BRUSH_PID.kp=para_initial[2];
               MOTOR_BRUSH_PID.ki=para_initial[3];
               MOTOR_BRUSH_PID.kd=para_initial[4];
             #endif
        }
//                 GPS_PID.kp=para_initial[0];
//                 GPS_PID.kd=para_initial[1];
//               #if MOTOR_MODE
//                 MOTOR_PID.kp=para_initial[2];
//                 MOTOR_PID.ki=para_initial[3];
//                 MOTOR_PID.kd=para_initial[4];
//               #else
//                 MOTOR_BRUSH_PID.kp=para_initial[2];
//                 MOTOR_BRUSH_PID.ki=para_initial[3];
//                 MOTOR_BRUSH_PID.kd=para_initial[4];
//               #endif
                 memset(PID_buf,0xff,sizeof(PID_buf));
                 PID_buf[0]=para_initial[0];
                 PID_buf[1]=para_initial[1];
#if MOTOR_MODE
                 PID_buf[2]=para_initial[2];
                 PID_buf[3]=para_initial[3];
                 PID_buf[4]=para_initial[4];
#else
                 PID_buf[2]=para_initial[2];
                 PID_buf[3]=para_initial[3];
                 PID_buf[4]=para_initial[4];
#endif
                 EEPROM_EraseSector(1);
                 EEPROM_Write(1, 1, PID_buf,5 );
}
void SPEED_SET()
{
    uint8 SET_NOW = 0;
    uint8 SET_LAST = 0;
char txt[30];
    // 显示固定字符串
    Display_showString(0, 1, "->", U16_WHITE, U16_BLACK, 12);
//    Display_showString(2, 1, "GPS_SPEED", U16_WHITE, U16_BLACK, 12);
//    Display_showString(2, 3, "BASE_SPEED", U16_WHITE, U16_BLACK, 12);

    #if LOOP == 0 // 开环

    int speed = speed_set;
    int BASE_speed = base_speed;
    int GD_BASE_speed= gd_base_speed;
    int para_initial[3]={speed,BASE_speed,GD_BASE_speed};
    int digit = 50;

    // 显示开环设置提示
//    Display_showString(2, 13, "PLEASE SET SPEED", U16_WHITE, U16_BLACK, 12);

    #else // 闭环
    int digit = 10;
    int speed = speed_loop_set;
    int BASE_speed = base_loop_speed;
    int para_initial[2]={speed,BASE_speed};
    // 显示闭环设置提示
//    Display_showString(2, 13, "PLEASE SET LOOP SPEED", U16_WHITE, U16_BLACK, 12);
    #endif
    sprintf(txt,"%04d",speed);
    Display_showString(8, 1, txt, U16_WHITE, U16_BLACK, 12);
    sprintf(txt,"%04d",BASE_speed);
        Display_showString(8, 3, txt, U16_WHITE, U16_BLACK, 12);
        sprintf(txt,"%04d",gd_base_speed);
             Display_showString(8, 5, txt, U16_WHITE, U16_BLACK, 12);
while(1)
{
    key_scan();
    if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY0_flag==1)//检测到KEY0按下，退出
            {
              key_clean();
              break;
            }
            else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY1_flag==1)
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
            else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY2_flag==1)
            {
                key_clean();

            }
    if(SET_LAST!=SET_NOW)
           {
               Display_CLS(U16_BLACK);
                      Display_showString(0, 1+(2*SET_NOW), "->", U16_WHITE, U16_BLACK, 12);
                      SET_LAST=SET_NOW;
           }
    if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY0_flag==1)
    {
        key_clean();
        para_initial[SET_NOW]+=digit;
    }
    if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY1_flag==1)
    {
        key_clean();
        para_initial[SET_NOW]-=digit;
    }
#if MOTOR_MODE
               // 显示 speed
               sprintf(txt, "speed:%04d", para_initial[0]);
               Display_showString(2, 1, txt, U16_WHITE, U16_BLACK, 12);

               // 显示 base_speed
               sprintf(txt, "base_spd:%04d", para_initial[1]);
               Display_showString(2, 3, txt, U16_WHITE, U16_BLACK, 12);
               sprintf(txt,"gd_speed%04d",para_initial[2]);
                        Display_showString(2, 5, txt, U16_WHITE, U16_BLACK, 12);


               #else
               // 显示 speed
                  sprintf(txt, "speed:%04d", para_initial[0]);
                  Display_showString(2, 1, txt, U16_WHITE, U16_BLACK, 12);

                  // 显示 base_speed
                  sprintf(txt, "base_sp:%04d", para_initial[1]);
                  Display_showString(2, 3, txt, U16_WHITE, U16_BLACK, 12);
                  sprintf(txt,"gd_speed:%04d",para_initial[2]);
                Display_showString(2, 5, txt, U16_WHITE, U16_BLACK, 12);

               #endif
}
speed=para_initial[0];
BASE_speed=para_initial[1];
GD_BASE_speed=para_initial[2];
speed_set=para_initial[0];
base_speed=para_initial[1];
gd_base_speed=para_initial[2];
memset(speed_buf,0xff,20);
speed_buf[0]=para_initial[0];
speed_buf[1]=para_initial[1];
speed_buf[2]=para_initial[2];
EEPROM_EraseSector(2);
EEPROM_Write(2, 0, speed_buf, 3);
}
void speed_GET()
{
EEPROM_Read(2, 0, speed_buf, 3);
#if MOTOR_MODE
speed_set=speed_buf[0];
base_speed=speed_buf[1];
gd_base_speed=speed_buf[2];
#else if
speed_set=speed_buf[0];
base_speed=speed_buf[1];
gd_base_speed=speed_buf[2];
#endif
}
void flash_get()
{
    int8_t SET_NOW=0;//现在正在调节的参数
        int8_t SET_LAST=0;
    Display_showString(0, 1,"->", U16_WHITE, U16_BLACK, 12);
    Display_showString(2, 1,"gps_save", U16_WHITE, U16_BLACK, 12);
    Display_showString(2, 3,"pid_save", U16_WHITE, U16_BLACK, 12);
    Display_showString(2, 5,"speed_save", U16_WHITE, U16_BLACK, 12);
    Display_showString(2, 7,"kemu_save", U16_WHITE, U16_BLACK, 12);
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
              if(SET_NOW==6)
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
                   case 0:point_get();Display_showString(0, 15, "gps save", U16_WHITE, U16_BLACK, 12);break;
                   case 1:point2_get();Display_showString(0, 15, "gps2 save", U16_WHITE, U16_BLACK, 12);break;
                   case 2:point3_get();Display_showString(0, 15, "gps3 save", U16_WHITE, U16_BLACK, 12);break;
                   case 3:PID_PARA_Init();Display_showString(0, 15, "PID save", U16_WHITE, U16_BLACK, 12);break;
                   case 4:speed_GET();Display_showString(0, 15, "speed save", U16_WHITE, U16_BLACK, 12);break;
                   case 5:kemu3_GET();Display_showString(0, 15, "kemu save", U16_WHITE, U16_BLACK, 12);break;
                   case 6:out_index=0;gyro_angle_totol=0;GD_get();Display_showString(0, 15, "GD save", U16_WHITE, U16_BLACK, 12);break;
               }
           }
        if(SET_LAST!=SET_NOW)
                {
                    Display_CLS(U16_BLACK);
                           Display_showString(0, 1+(2*SET_NOW), "->", U16_WHITE, U16_BLACK, 12);
                           SET_LAST=SET_NOW;
                }
        Display_showString(2, 1,"gps_save", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 3,"gps2_save", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 5,"gps3_save", U16_WHITE, U16_BLACK, 12);
            Display_showString(2, 7,"pid_save", U16_WHITE, U16_BLACK, 12);
            Display_showString(2, 9,"speed_save", U16_WHITE, U16_BLACK, 12);
            Display_showString(2, 11,"kemu_save", U16_WHITE, U16_BLACK, 12);
            Display_showString(2, 13,"GD_save", U16_WHITE, U16_BLACK, 12);
    }
}
void mean_flash_clear()
{
    int8_t SET_NOW=0;//现在正在调节的参数
            int8_t SET_LAST=0;
        Display_showString(0, 1,"->", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 1,"pid_clear", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 3,"speed_clear", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 5,"gps_clear", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 7,"GD_clear", U16_WHITE, U16_BLACK, 12);
        Display_showString(2, 9,"kemu_clear", U16_WHITE, U16_BLACK, 12);
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
                      if(SET_NOW==6)
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
                           case 0:EEPROM_EraseSector(SET_NOW+1);Display_showString(0, 15, "already clean", U16_WHITE, U16_BLACK, 12);break;
                           case 1:EEPROM_EraseSector(SET_NOW+1);Display_showString(0, 15, "already clean", U16_WHITE, U16_BLACK, 12);break;
                           case 2:memset(Flash_buf,0xff,MAX_Dot*4+1);EEPROM_EraseSector(5);Display_showString(0, 15, "gps_already clean", U16_WHITE, U16_BLACK, 12);break;
                           case 3:memset(Flash_buf,0xff,MAX_Dot*4+1);EEPROM_EraseSector(7);Display_showString(0, 15, "gps2_already clean", U16_WHITE, U16_BLACK, 12);break;
                           case 4:memset(Flash_buf,0xff,MAX_Dot*4+1);EEPROM_EraseSector(8);Display_showString(0, 15, "gps3_already clean", U16_WHITE, U16_BLACK, 12);break;
                           case 5:EEPROM_EraseSector(4);Display_showString(0, 15,"GD_clear", U16_WHITE, U16_BLACK, 12);break;
                           case 6:EEPROM_EraseSector(6);Display_showString(0, 15,"kemu_clear", U16_WHITE, U16_BLACK, 12);break;
                       }
                   }
                if(SET_LAST!=SET_NOW)
                        {
                            Display_CLS(U16_BLACK);
                                   Display_showString(0, 1+(2*SET_NOW), "->", U16_WHITE, U16_BLACK, 12);
                                   SET_LAST=SET_NOW;
                        }
                Display_showString(2, 1,"pid_clear", U16_WHITE, U16_BLACK, 12);
                    Display_showString(2, 3,"speed_clear", U16_WHITE, U16_BLACK, 12);
                    Display_showString(2, 5,"gps_clear", U16_WHITE, U16_BLACK, 12);
                    Display_showString(2, 7,"gps2_clear", U16_WHITE, U16_BLACK, 12);
                    Display_showString(2, 9,"gps3_clear", U16_WHITE, U16_BLACK, 12);
                    Display_showString(2, 11,"GD_clear", U16_WHITE, U16_BLACK, 12);
                    Display_showString(2, 13,"kemu_clear", U16_WHITE, U16_BLACK, 12);

            }
}
void GD_get()
{
    EEPROM_Read(4, 0, IMU_buf, 2000);
    _in_index=IMU_buf[0];
    for(uint16_t i =1;i<=_in_index;i++)
    {
        yaw_gd[i]=IMU_buf[i];

    }
}
void kemu3_SET()
{
    uint8 SET_NOW = 0;
    uint8 SET_LAST = 0;
char txt[30];
    // 显示固定字符串
    Display_showString(0, 1, "->", U16_WHITE, U16_BLACK, 12);
    kemu_buf[4]=zhuitong;
while(1)
{
    key_scan();
    if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY0_flag==1)//检测到KEY0按下，退出
            {
              key_clean();
              break;
            }
            else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY1_flag==1)
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
            else if(SWITCH0_FLAG==1&SWITCH1_FLAG==1&KEY2_flag==1)
            {
                key_clean();

            }
    if(SET_LAST!=SET_NOW)
           {
               Display_CLS(U16_BLACK);
                      Display_showString(0, 1+(2*SET_NOW), "->", U16_WHITE, U16_BLACK, 12);
                      SET_LAST=SET_NOW;
           }
    if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY0_flag==1)
    {
        key_clean();
        kemu_buf[SET_NOW]+=1;
    }
    if(SWITCH0_FLAG==1&SWITCH1_FLAG==0&KEY1_flag==1)
    {
        key_clean();
        kemu_buf[SET_NOW]-=1;
    }
               // 显示 speed
               sprintf(txt, "feipo:%04d", kemu_buf[0]);
               Display_showString(2, 1, txt, U16_WHITE, U16_BLACK, 12);

               // 显示 base_speed
               sprintf(txt, "dbld:%04d", kemu_buf[1]);
               Display_showString(2, 3, txt, U16_WHITE, U16_BLACK, 12);
               // 显示 speed
               sprintf(txt, "qiaodong:%04d", kemu_buf[2]);
               Display_showString(2, 5, txt, U16_WHITE, U16_BLACK, 12);

               // 显示 base_speed
               sprintf(txt, "cp:%04d", kemu_buf[3]);
               Display_showString(2, 7, txt, U16_WHITE, U16_BLACK, 12);
               sprintf(txt, "zhuitong:%04d", kemu_buf[4]);
                        Display_showString(2, 9, txt, U16_WHITE, U16_BLACK, 12);



}
kemu[0]=kemu_buf[0];
kemu[1]=kemu_buf[1];
kemu[2]=kemu_buf[2];
kemu[3]=kemu_buf[3];
zhuitong=kemu_buf[4];


EEPROM_EraseSector(6);
EEPROM_Write(6, 1, kemu_buf, 4);
}
void kemu3_GET()
{
    EEPROM_Read(6, 1, kemu_buf, 4);
    kemu[0]=kemu_buf[0];
    kemu[1]=kemu_buf[1];
    kemu[2]=kemu_buf[2];
    kemu[3]=kemu_buf[3];
    zhuitong=kemu_buf[4];
}
//void GPS_point_get()
//{
//    char txt[30];
//  uint8 num=0;
//  double lat=0;
//  double lon=0;
//  uint8 change_flag=0;//检测此次执行该函数后是否改变点位信息
//  sprintf(txt, "NOW POINT:%d", (uint32)num);
//                 Display_showString(0, 1,txt, U16_WHITE, U16_BLACK, 12);
//                 sprintf(txt, "X:%f", (float)point[num][0]);
//                 Display_showString(0, 3,txt, U16_WHITE, U16_BLACK, 12);
//                 sprintf(txt, "Y:%f", (float)point[num][1]);
//                 Display_showString(0, 5,txt, U16_WHITE, U16_BLACK, 12);
//                 sprintf(txt, "LAT_NOW:%f", (float)GPS_data.LAT);
//                 Display_showString(0, 7,txt, U16_WHITE, U16_BLACK, 12);
//                 sprintf(txt, "LON_NOW:%f", (float)GPS_data.LON);
//                 Display_showString(0, 9,txt, U16_WHITE, U16_BLACK, 12);
////  ips200_show_string(0,  16*0, "NOW POINT: ");
////  ips200_show_uint (8*13, 16*0, (uint32)num, 2);
////  ips200_show_string(0,  16*3, "LAT: ");
////  ips200_show_float (0, 16*4, (float)point[num][0], 2, 6);
////  ips200_show_string(0,  16*5, "LON: ");
////  ips200_show_float (0, 16*6, (float)point[num][1], 3, 6);
////  ips200_show_string(0,  16*11, "LAT_NOW: ");
////  ips200_show_float (0, 16*12, (float)GPS_data.LAT, 2, 6);
////  ips200_show_string(0,  16*13, "LON_NOW: ");
////  ips200_show_float (0, 16*14, (float)GPS_data.LON, 3, 6);
//  while(1)
//  {
//    key_scan();
//    if(KEY0_flag==1)//检测到KEY0按下，退出
//    {
//      key_clean();
//      break;
//    }
//    else if(KEY1_flag==1)
//    {
//        key_clean();
//       if(num==10)
//      {
//          num=0;
//      }
//      else
//      {
//        num++;
//      }
//    }
//    else if(KEY2_flag==1)
//    {
//        key_clean();//清除标志
//        lat=0;
//          lon=0;
//          for(uint8 z=0;z<10;z++)
//          {
//            Delay_Ms(102);
//            lat+=GPS_data.LAT;
//            lon+=GPS_data.LON;
//          }
//          lat/=10.00;
//          lon/=10.00;
//          point[num][0]=lat;//存放纬度
//          point[num][1]=lon;//存放经度
//          change_flag=1;//检测到点位信息改变
//    }
//                     sprintf(txt, "NOW POINT:%02d", (uint32)num);
//                     Display_showString(0, 1,txt, U16_WHITE, U16_BLACK, 12);
//                     sprintf(txt, "LAT:%f", (float)point[num][0]);
//                     Display_showString(0, 3,txt, U16_WHITE, U16_BLACK, 12);
//                     sprintf(txt, "LON:%f", (float)point[num][1]);
//                     Display_showString(0, 5,txt, U16_WHITE, U16_BLACK, 12);
//                     sprintf(txt, "LAT_NOW:%f", (float)GPS_data.LAT);
//                     Display_showString(0, 7,txt, U16_WHITE, U16_BLACK, 12);
//                     sprintf(txt, "LON_NOW:%f", (float)GPS_data.LON);
//                     Display_showString(0, 9,txt, U16_WHITE, U16_BLACK, 12);
//  }
//  if(change_flag==1)////////跳出循环代表采点的结束，但该函数工作并未完成，退出后存储点位
//  {
//      gps_write();
//  }
//}
