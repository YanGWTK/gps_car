#ifndef _LQ_PWM_BLDC_H_
#define _LQ_PWM_BLDC_H_

// 电机频率
#define MOTOR_FREQUENCY 10000
// 电机PWM 宏定义
#define BLDC_AP IfxGtm_ATOM0_6_TOUT42_P23_1_OUT
#define BLDC_BP IfxGtm_ATOM0_5_TOUT40_P32_4_OUT
#define BLDC_CP IfxGtm_ATOM0_0_TOUT53_P21_2_OUT

#define BLDC_AI P21_4
#define BLDC_BI P21_3
#define BLDC_CI P20_8
extern int ENC,S_ENC;
extern float MPS;
void BLDC_HallInit(void);
uint8_t Get_Hall_State(void);
void TestMotorBLDC (void);
void LQ_BLDCStop(void);
void LQ_BLDCCtrl(void);

extern int BLDC_duty;
#endif
