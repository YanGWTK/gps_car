/*
 * peripherals.c
 *
 *  Created on: 2024��12��13��
 *      Author: 29500
 */

#include "include.h"

//电机与编码器
volatile sint32 encoder=0;
sint16 current_speed;
sint16 error;
sint16 out;
sint16 value_speed;
volatile sint32 encoder_totol=0;
void BLDC_motor_Init()
{

    ATOM_PWM_InitConfig(MOTOR, 0, bldc_MOTOR_FREQUENCY);
    PIN_InitConfig(P32_4, PIN_MODE_OUTPUT, 1);
    ENC_InitConfig(ENC1_InPut_P33_7, ENC1_Dir_P33_6);
    CCU6_InitConfig(CCU61,CCU6_Channel0,20);
}
void BLDC_motor_crtl(sint16 duty)
{
    if(duty>=0)
    {
        ATOM_PWM_SetDuty (MOTOR,duty,bldc_MOTOR_FREQUENCY);
        PIN_Write(P32_4,0);
    }
    else
    {
        ATOM_PWM_SetDuty (MOTOR,-(duty),bldc_MOTOR_FREQUENCY);
        PIN_Write(P32_4,1);
    }
}
void motor_Init()
{
    ATOM_PWM_InitConfig(MOTOR, 0, MOTOR_FREQUENCY);
    PIN_InitConfig(P32_4, PIN_MODE_OUTPUT, 1);
    ENC_InitConfig(ENC1_InPut_P33_7, ENC1_Dir_P33_6);
    CCU6_InitConfig(CCU61,CCU6_Channel0,20);
}
void motor_crtl(sint16 duty)
{
    if(duty>=0)
    {
        ATOM_PWM_SetDuty (MOTOR,duty,MOTOR_FREQUENCY);
        PIN_Write(P32_4,1);
    }
    else
    {
        ATOM_PWM_SetDuty (MOTOR,-(duty),MOTOR_FREQUENCY);
        PIN_Write(P32_4,0);
    }
}
void encoder_get()
{
    encoder=-ENC_GetCounter(ENC1_InPut_P33_7);
}
void motor_loop(sint16 speed)//有刷闭环
{
   current_speed=encoder;
   error=speed-current_speed;
   out=PidIncCtrl(&MOTOR_PID, (float)error);
   motor_crtl(out);
}


//舵机
void servo_Init()
{
    ATOM_PWM_InitConfig(ATOMSERVO1, (uint32)SERVO_MOTOR_DUTY(servo_middle), SERVO_MOTOR_FREQ);
}
void Servo_Ctrl (float angle)
{
    angle = servo_middle-angle;
    if (angle >= servo_left)                  //限制幅值
        angle = servo_left;
    else if (angle <= servo_right )            //限制幅值
        angle = servo_right ;

    ATOM_PWM_SetDuty(ATOMSERVO1, (uint32)SERVO_MOTOR_DUTY(angle),50);
}
//按键
uint8 KEY0_flag=0;
uint8 KEY1_flag=0;
uint8 KEY2_flag=0;
uint8 SWITCH0_FLAG=1;
uint8 SWITCH1_FLAG=1;
void KEY_Init(void)
{
      // 初始化,输入口，高电平
      PIN_InitConfig(KEY0p, PIN_MODE_INPUT, 1);
      PIN_InitConfig(KEY1p, PIN_MODE_INPUT, 1);
      PIN_InitConfig(KEY2p, PIN_MODE_INPUT, 1);
      PIN_InitConfig(DSW0p, PIN_MODE_INPUT, 1);
      PIN_InitConfig(DSW1p, PIN_MODE_INPUT, 1);
}
void key_scan()
{
if(PIN_Read(KEY0p)==0)
{
    Delay_Ms(5);
while(!PIN_Read(KEY0p));
Delay_Ms(5);
KEY0_flag=1;
}
if(PIN_Read(KEY1p)==0)
{
    Delay_Ms(5);
while(!PIN_Read(KEY1p));
Delay_Ms(5);
KEY1_flag=1;
}
if(PIN_Read(KEY2p)==0)
{
    Delay_Ms(5);
while(!PIN_Read(KEY2p));
Delay_Ms(5);
KEY2_flag=1;
}
if(PIN_Read(DSW0p)==0)
{
SWITCH0_FLAG=0;
}
if(PIN_Read(DSW0p)==1)
{
SWITCH0_FLAG=1;
}
if(PIN_Read(DSW1p)==0)
{
SWITCH1_FLAG=0;
}
if(PIN_Read(DSW1p)==1)
{
SWITCH1_FLAG=1;
}
}
void key_clean()
{
    KEY0_flag=0;
    KEY1_flag=0;
    KEY2_flag=0;
}

