/*
 * ICM.c
 *
 *  Created on: 2024  12  8
 *      Author: 29500
 */
//#include <IfxCpu.h>
//#include <IfxScuCcu.h>
//#include <IfxScuWdt.h>
//#include <IfxStm.h>
//#include <IfxStm_reg.h>
//#include <stdio.h>
#include "include.h"
#define delta_T 0.0025f // 2ms计算一次
#define alpha 0.3f
double GPS_gyro_angle;
float GyroOffset_Xdata = 0, icm_data_acc_x = 0, icm_data_gyro_x = 0;
float GyroOffset_Ydata = 0, icm_data_acc_y = 0, icm_data_gyro_y = 0;
float GyroOffset_Zdata = 0, icm_data_acc_z = 0, icm_data_gyro_z = 0;
float Q_info_q0 = 1, Q_info_q1 = 0, Q_info_q2 = 0, Q_info_q3 = 0;
float I_ex=0, I_ey=0, I_ez=0; // 误差积分
float param_Kp = 0.170;  // 加速度计的收敛速率比例增益
float param_Ki = 0.003; // 陀螺仪收敛速率的积分增益 0.004
float eulerAngle_yaw = 0, eulerAngle_pitch = 0, eulerAngle_roll = 0, eulerAngle_yaw_total = 0, eulerAngle_yaw_old = 0;
float q0, q1, q2, q3;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数初始化
float beta = BETA_DEF; // 滤波器增益
gyro_param gyro_offset;//存放陀螺仪标定数据
uint8 gyro_offset_flag=0;
IMU_param IMU_data;//存放IMU各项数据
double gyro_angle=0;//陀螺仪得到的角度
sint8 MODE=1;
double gyro_angle_old=0;
double gyro_angle_totol=0;
signed short  gyrox,gyroy,gyroz,aacx,aacy,aacz;
double gyro_m=0;
double gyro_angle_2=0;
void IMU_gyro_offset()//陀螺仪数据纠正
{
  gyro_offset_flag=0;
  gyro_offset.Zdata=0;
  sint16 i;
  for(i=0;i<400;i++)
  {
      ICM_Get_Raw_data(&aacx, &aacy, &aacz, &gyrox, &gyroy, &gyroz);
    gyro_offset.Zdata+=gyroz;
    gyro_offset.Xdata+=gyrox;
    gyro_offset.Ydata+=gyroy;
    Delay_Ms(5);
  }
  gyro_offset.Zdata=gyro_offset.Zdata/400;
  gyro_offset.Xdata=gyro_offset.Xdata/400;
  gyro_offset.Ydata=gyro_offset.Ydata/400;
  gyro_offset_flag=1;
}

void IMU_Init()
{
    SPI_Gryo_Init();
    CCU6_InitConfig_u(CCU60,CCU6_Channel0,2500);
    IMU_gyro_offset();
}

void angle_handle_360(double *angle)//规划坐标系角度
{
  if(*angle>360){*angle-=360;}
  else if(*angle<0){*angle+=360;}
}

void angle_plan(double *angle)//规划偏航角
{
  if(*angle>180){*angle=*angle-360;}
  else if(*angle<-180){*angle=*angle+360;}
  else if(*angle==-180){*angle=180;}
}


//void IMU_getvalue()
//{
//  IMU_data.gyro_z=((float)gyroz-gyro_offset.Zdata)*PI/180/14.3f;
//  if(IMU_data.gyro_z<0.020&&IMU_data.gyro_z>-0.020)
//  {
//      IMU_data.gyro_z=0;
//  }
//}
//
//void IMU_gyro_integral()//积分
//{
//  static float gyro_lastlastlast=0;
//  static float gyro_lastlast=0;
//  static float gyro_last=0;
//  static uint8 time=0;//此时获取数据的次数
//  IMU_getvalue();
//  switch (time)
//  {
//  case 0:
//    gyro_lastlastlast=IMU_data.gyro_z;
//    time=1;
//    break;
//  case 1:
//    gyro_lastlast=IMU_data.gyro_z;
//    time=2;
//    break;
//  case 2:
//    gyro_last=IMU_data.gyro_z;
//    time=3;
//    break;
//  case 3:
//
//    gyro_angle+=-RAD_TO_ANGLE(0.9*(gyro_lastlast+4*gyro_last+IMU_data.gyro_z)*0.005/6.000+0.1*(gyro_lastlastlast+4*gyro_lastlast+gyro_last)*0.005/6.00);//    ɭ   ֹ ʽ
//    gyro_lastlast=IMU_data.gyro_z;
//    gyro_lastlastlast=gyro_last;
//    angle_handle_360(&gyro_angle);
//    time=2;
//    break;
//  }
//}


//
//
//
float fast_sqrt(float num) {
    float halfx = 0.5f * num;
    float y = num;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1);

    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
    // float y = sqrtf(num);
    // return y;
}
float test = 1;
float test1 = 1;
float test2 = 1;
float test3 = 1;




// 转化为实际物理值
void ICM_getValues() {
    // 一阶低通滤波，单位g/s
    IMU_data.acc_x = (((float)aacx) * alpha) + IMU_data.acc_x * (1 - alpha);
    IMU_data.acc_y = (((float)aacy) * alpha) + IMU_data.acc_y * (1 - alpha);
    IMU_data.acc_z = (((float)aacz) * alpha) + IMU_data.acc_z * (1 - alpha);
    // 陀螺仪角速度转弧度
    IMU_data.gyro_z=((float)gyroz - gyro_offset.Zdata) * PI / 180 / 14.3f;
    IMU_data.gyro_x=((float)gyrox - gyro_offset.Xdata) * PI / 180 / 14.3f;
    IMU_data.gyro_y=((float)gyroy - gyro_offset.Ydata) * PI / 180 / 14.3f;
      if(IMU_data.gyro_z<0.040&&IMU_data.gyro_z>-0.040)
      {
          IMU_data.gyro_z=0;
  }
}

// 互补滤波
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float halfT = 0.5 * delta_T;
    float vx, vy, vz; // 当前的机体坐标系上的重力单位向量
    float ex, ey, ez; // 四元数计算值与加速度计测量值的误差
    float q0 = Q_info_q0;
    float q1 = Q_info_q1;
    float q2 = Q_info_q2;
    float q3 = Q_info_q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    //float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    //float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    // float delta_2 = 0;

    // 对加速度数据进行归一化 得到单位加速度
    float norm = fast_sqrt(ax * ax + ay * ay + az * az);

    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // 根据当前四元数的姿态值来估算出各重力分量。用于和加速计实际测量出来的各重力分量进行对比，从而实现对四轴姿态的修正
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    // vz = (q0*q0-0.5f+q3 * q3) * 2;

    // 叉积来计算估算的重力和实际测量的重力这两个重力向量之间的误差。
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 用叉乘误差来做PI修正陀螺零偏，
    // 通过调节 param_Kp，param_Ki 两个参数，
    // 可以控制加速度计修正陀螺仪积分姿态的速度。
    I_ex += halfT * ex; // integral error scaled by Ki
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;

    /*数据修正完成，下面是四元数微分方程*/

    // 四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    //    整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT

    // normalise quaternion
    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info_q0 = q0 * norm;
    Q_info_q1 = q1 * norm;
    Q_info_q2 = q2 * norm;
    Q_info_q3 = q3 * norm;

}

void ICM_getEulerianAngles(void)
{
    // 采集陀螺仪数据
    ICM_Get_Raw_data(&aacx, &aacy, &aacz, &gyrox, &gyroy, &gyroz);
    aac_x=(float)aac_x/ 16384.0;
    aac_y=(float)aac_y/ 16384.0;
    aac_z=(float)aac_z/ 16384.0;

    ICM_getValues();
    ICM_AHRSupdate(IMU_data.gyro_x, IMU_data.gyro_y, IMU_data.gyro_z, IMU_data.acc_x, IMU_data.acc_y, IMU_data.acc_z);
    float q0 = Q_info_q0;
    float q1 = Q_info_q1;
    float q2 = Q_info_q2;
    float q3 = Q_info_q3;
    // 四元数计算欧拉角---原始
    gyro_angle = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI;
    GPS_gyro_angle = gyro_angle+gyro_angle_1-gyro_angle_2;  // yaw
    angle_handle_360(&gyro_angle);
    angle_handle_360(&GPS_gyro_angle);
}



