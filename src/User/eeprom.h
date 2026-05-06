/*
 * eeprom.h
 *
 *  Created on: 2024年12月30日
 *      Author: 29500
 */

#ifndef SRC_USER_EEPROM_H_
#define SRC_USER_EEPROM_H_
#define MAX_Dot 50
#define MOTOR_MODE 1
#define LOOP 0
//typedef struct {
//        double PX;                 //坐标值
//        double PY;
//        double Direction;         //车头朝向（方向）
//}Position_t;
extern float IMU_buf[2000];
extern uint8_t num_dot;
extern uint32_t Flash_buf[MAX_Dot*4+1];
extern float PID_buf[5];
extern int16_t speed_buf[3];
extern float yaw_gd[2000];
extern uint8_t kemu_buf[4];
extern uint16_t max_num;
extern float distance_test;
extern double angle_test;
void gps_write();

void GPS_point_get();

void point_get();
void point2_get();
void point3_get();
void PID_PARA_Init();
void PID_SET();
void SPEED_SET();
void speed_GET();
void flash_get();
void GD_get();
void mean_flash_clear();
void kemu3_SET();
void kemu3_GET();
#endif /* SRC_USER_EEPROM_H_ */
