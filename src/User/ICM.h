/*
 * ICM.h
 *
 *  Created on: 2024年12月8日
 *      Author: 29500
 */

#ifndef SRC_USER_ICM_H_
#define SRC_USER_ICM_H_
#define M_PI 3.14159265358979323846

#include "include.h"
typedef struct
{
    short accX;
    short accY;
    short accZ;
    short gyroX;
    short gyroY;
    short gyroZ;
} ICM_DATA;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} ICM_Angle;


#define ANGLE_TO_RAD(x)    ((x) * PI / 180.0)                                   // 角度转换为弧度
#define RAD_TO_ANGLE(x)    ((x) * 180.0 / PI)                                   // 弧度转换为角度
#define PI                 (3.1415926535898)

extern float GyroOffset_Xdata;
extern float GyroOffset_Ydata;
extern float GyroOffset_Zdata;
extern float GyroOffset_Xdata, icm_data_acc_x, icm_data_gyro_x;
extern float GyroOffset_Ydata, icm_data_acc_y, icm_data_gyro_y;
extern float GyroOffset_Zdata, icm_data_acc_z, icm_data_gyro_z;
extern double GPS_gyro_angle;
extern float Q_info_q0 , Q_info_q1 , Q_info_q2 , Q_info_q3 ;
typedef struct{
    float Xdata;   //零飘参数X
    float Ydata;   //零飘参数Y
    float Zdata;   //零飘参数Z

}gyro_param;
typedef struct {
    float q0;  // w分量
    float q1;  // x分量
    float q2;  // y分量
    float q3;  // z分量
} Quaternion;

extern Quaternion imu_quat; // 在icm.h中声明
extern float sample_freq;   // 采样频率（根据实际设置）
extern float twoKp;         // 比例增益
extern float twoKi;         // 积分增益

typedef struct{
    float acc_x;   //x轴加速度
    float acc_y;   //y轴加速度
    float acc_z;   //z轴加速度

    float gyro_x;  //x轴角速度
    float gyro_y;  //y轴角速度
    float gyro_z;  //z轴角速度

    float mag_x;   //x轴磁力计
    float mag_y;   //y轴磁力计
    float mag_z;   //z轴磁力计
}IMU_param;
extern signed short  gyrox,gyroy,gyroz,aacx,aacy,aacz;
extern double gyro_angle;
extern gyro_param gyro_offset;
extern uint8 gyro_offset_flag;
extern IMU_param IMU_data;
extern double gyro_angle_old;//惯导
extern double gyro_angle_totol;
#define BETA_DEF     0.1f    // 滤波器增益
extern float q0, q1, q2, q3; // 四元数
extern float eulerAngle_yaw;
extern double gyro_m;
extern double gyro_angle_2;
extern ICM_DATA icm_data;

extern ICM_DATA icm_offset;

extern ICM_Angle icm_angle;

void IMU_GetOffset(ICM_DATA *offset);

// 姿态更新函数
void IMUupdate(ICM_DATA *pMpu, ICM_Angle *pAngle);

// 获取姿态数据
void Attitude_get(void);
void IMU_gyro_offset();

void IMU_Read(ICM_DATA *pMpu);
void IMU_Init();

void angle_handle_360(double *angle);

void angle_plan(double *angle);

void IMU_getvalue();


void IMU_gyro_integral();
void ICM_getValues();
void ICM_getEulerianAngles(void);
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
void ICM_getEulerianAngles(void);
void EulerToQuaternion(float yaw_deg, float pitch_deg, float roll_deg,
                      float *q0, float *q1, float *q2, float *q3) ;

#endif /* SRC_USER_ICM_H_ */
