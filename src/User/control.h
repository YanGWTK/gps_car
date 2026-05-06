/*
 * control.h
 *
 *  Created on: 2025年3月3日
 *      Author: 29500
 */

#ifndef SRC_USER_CONTROL_H_
#define SRC_USER_CONTROL_H_
extern uint8 check_flag;//自检标志位
extern int start_flag;
extern uint8 switch_flag;
extern sint16 speed_set;
extern sint16 speed_loop_set;
extern sint16 base_speed;
extern sint16 gd_base_speed;
extern sint16 base_loop_speed;
extern double filter_point[2];
extern int16_t next_point;
extern int filter_flag;
extern uint16_t last_point;
extern uint16_t turn_point;
extern float this_postion, last_postion; // 记忆当前位置值,历史值
extern float _get_yaw;
extern float add_pos;
extern int8_t save_flag;
extern int8_t save_run;
extern int16_t in_index;
extern int16_t _in_index;
extern int16_t out_index;
extern int8_t eeprom_in;
extern uint16_t long_index;
extern uint8_t start_flag1;
extern uint8_t kemu_flag;
extern uint8_t task3_flag;
extern uint8_t fp_flag;
extern uint8_t db_flag;
extern uint8_t qd_flag;
extern uint8_t cp_flag;
extern uint8_t kemu[4];
extern uint8_t task1_flag;
extern uint8_t task2_flag;
extern uint8_t task3_flag_s;
extern uint8_t zhuitong;
extern uint16_t qd_num;
extern double distance_target;
void Switching_point_D();
void Switching_point_2();
void Switching_point_3();
void control();
void start_deal();
void start3_deal();
void task1_control();

void task2_control();

void task2_control_GPS();

void task3_control();
void gps3_track();
#endif /* SRC_USER_CONTROL_H_ */
