/*
 * kalman.h
 *
 *  Created on: 2025年4月27日
 *      Author: 29500
 */

#ifndef SRC_USER_KALMAN_H_
#define SRC_USER_KALMAN_H_
#define Pi 3.1415926f
typedef struct IMU_KALMAN
{
    float angle;
    float gyro;
    float delta_t;
    float Q_angle; //平滑 越小越平滑
    float Q_gyro;   //没啥用
    float R_angle;  //相位 越大，越超前
    float Q_bias;
    float p[2][2];
    float k[2];

}IMU_KALMAN;


#endif /* SRC_USER_KALMAN_H_ */
