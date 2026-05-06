/*
 * peripherals.h
 *
 *  Created on: 2024��12��13��
 *      Author: 29500
 */

#ifndef SRC_USER_PERIPHERALS_H_
#define SRC_USER_PERIPHERALS_H_
#define MOTOR          IfxGtm_ATOM0_6_TOUT42_P23_1_OUT
#define DIR            P32_4
#define ATOMSERVO       IfxGtm_ATOM2_0_TOUT32_P33_10_OUT
#define SERVO_MOTOR_DUTY(x)         ((float)ATOM_PWM_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))// ------------------ 舵机占空比计算方式 ------------------
#define MOTOR_FREQUENCY    10000
#define bldc_MOTOR_FREQUENCY    1000
#define servo_middle       67
#define servo_right        40//右打满
#define servo_left         94//左打满
#define  SERVO_MOTOR_FREQ  50
//按键
extern uint8 KEY0_flag;
extern uint8 KEY1_flag;
extern uint8 KEY2_flag;
extern uint8 SWITCH0_FLAG;
extern uint8 SWITCH1_FLAG;
void KEY_Init();

void key_scan();

void key_clean();
//电机与编码器
extern volatile sint32 encoder;
extern volatile sint32 encoder_totol;
extern sint16 current_speed;
extern sint16 error;
extern sint16 out;
extern sint16 value_speed;
void BLDC_motor_Init();

void BLDC_motor_crtl(sint16 duty);
void motor_Init();

void motor_crtl(sint16 duty);

void encoder_get();

void motor_loop(sint16 speed);

//舵机
void servo_Init();

void Servo_Ctrl (float angle);



#endif /* SRC_USER_PERIPHERALS_H_ */
