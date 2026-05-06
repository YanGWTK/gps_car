#include "include.h"
#include "LQ_PWM_BLDC.h"


int BLDC_duty=0;
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void MotorInit(void)
@功能说明：电机PWM初始化
@参数说明：无
@函数返回：无
@调用方法：MotorInit(100);
@备    注：驱动4个电机
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void BLDCInit(void)
{
    ATOM_PWM_InitConfig(BLDC_AP, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(BLDC_BP, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(BLDC_CP, 0, MOTOR_FREQUENCY);


    PIN_InitConfig(BLDC_AI, PIN_MODE_OUTPUT, 0);
    PIN_InitConfig(BLDC_BI, PIN_MODE_OUTPUT, 0);
    PIN_InitConfig(BLDC_CI, PIN_MODE_OUTPUT, 0);
    //PIN_InitConfig(MOTOR4_I, PIN_MODE_OUTPUT, 0);
}


/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void BLDC_HallInit(void);
@功能说明：初始化霍尔引脚及编码器接口
@参数说明：无
@函数返回：无
@备    注：编码器接口同时为霍尔引脚状态读取接口
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void BLDC_HallInit(void)
{
   PIN_InitConfig(P33_7, IfxPort_Mode_inputPullUp, 1);
//   PIN_InitConfig(P20_3, IfxPort_Mode_inputPullUp, 1);
//   PIN_InitConfig(P20_0, IfxPort_Mode_inputPullUp, 1);
   ENC_InitConfig(ENC4_InPut_P20_3, ENC4_Dir_P20_0);//BC相使用 编码器4接口
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：uint8_t Get_Hall_State(void);
@功能说明：读取霍尔状态
@参数说明：无
@函数返回：hall_state（霍尔状态）
@备    注：203-a 200-b 337-c
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
uint8_t Get_Hall_State(void)
{
  uint8_t hall_state = 0;

  if(PIN_Read(P20_3)) //F11/* 读取霍尔传感器 U 的状态 */
  {
    hall_state |= 0x01U << 0;
  }
  if(PIN_Read(P20_0)) //F12/* 读取霍尔传感器 V 的状态 */
  {
    hall_state |= 0x01U << 1;
  }
  if(PIN_Read(P33_7)) //F13 /* 读取霍尔传感器 W 的状态 */
  {
    hall_state |= 0x01U << 2;
  }
  return hall_state;
}


/**LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void LQ_BLDCCorotation(void);
@功能说明：控制无刷电机正转
@参数说明：无
@函数返回：无
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ**/
void LQ_BLDCCorotation(void)
{
    switch(Get_Hall_State())
    {
      case 1:
        //B->C
        ATOM_PWM_SetDuty(BLDC_AP, 0,MOTOR_FREQUENCY);            // 0
        ATOM_PWM_SetDuty(BLDC_BP, BLDC_duty,MOTOR_FREQUENCY);    // BLDC_duty
        ATOM_PWM_SetDuty(BLDC_CP, 0,MOTOR_FREQUENCY);            // 0
        PIN_Write(BLDC_AI, 0);              //0
        PIN_Write(BLDC_BI, 0);              //0
        PIN_Write(BLDC_CI, 1);              //1
        break;

      case 2:
        //C->A
        ATOM_PWM_SetDuty(BLDC_AP, 0,MOTOR_FREQUENCY);                  // P1_PWM_P输出0
        ATOM_PWM_SetDuty(BLDC_BP, 0,MOTOR_FREQUENCY);                  // P3_PWM_P输出0
        ATOM_PWM_SetDuty(BLDC_CP, BLDC_duty,MOTOR_FREQUENCY);  // P5_PWM_P输出PWM
        PIN_Write(BLDC_AI,1);              //关闭
        PIN_Write(BLDC_BI,0);              //关闭
        PIN_Write(BLDC_CI,0);              //打开
        break;
//
      case 3:
        //B->A
        ATOM_PWM_SetDuty(BLDC_AP, 0,MOTOR_FREQUENCY); // P1_PWM_P输出PWM
        ATOM_PWM_SetDuty(BLDC_BP, BLDC_duty,MOTOR_FREQUENCY);                // P3_PWM_P输出0
        ATOM_PWM_SetDuty(BLDC_CP, 0,MOTOR_FREQUENCY);                // P5_PWM_P输出0
        PIN_Write(BLDC_AI,1);              //关闭
        PIN_Write(BLDC_BI,0);              //打开
        PIN_Write(BLDC_CI,0);              //关闭
        break;

      case 4:
        //A->B
        ATOM_PWM_SetDuty(BLDC_AP, BLDC_duty,MOTOR_FREQUENCY);                // P1_PWM_P输出0
        ATOM_PWM_SetDuty(BLDC_BP, 0,MOTOR_FREQUENCY); // P3_PWM_P输出PWM
        ATOM_PWM_SetDuty(BLDC_CP, 0,MOTOR_FREQUENCY);                // P5_PWM_P输出0
        PIN_Write(BLDC_AI,0);              //打开
        PIN_Write(BLDC_BI,1);              //关闭
        PIN_Write(BLDC_CI,0);              //关闭
        break;

      case 5:
        //A->C
        ATOM_PWM_SetDuty(BLDC_AP, BLDC_duty,MOTOR_FREQUENCY);                   // P1_PWM_P输出0
        ATOM_PWM_SetDuty(BLDC_BP, 0,MOTOR_FREQUENCY);   // P3_PWM_P输出PWM
        ATOM_PWM_SetDuty(BLDC_CP, 0,MOTOR_FREQUENCY);                    // P5_PWM_P输出0
        PIN_Write(BLDC_AI,0);              //打开
        PIN_Write(BLDC_BI,0);              //关闭
        PIN_Write(BLDC_CI,1);              //关闭
        break;

      case 6:
        //C->B
        ATOM_PWM_SetDuty(BLDC_AP, 0,MOTOR_FREQUENCY);                  // P1_PWM_P输出0
        ATOM_PWM_SetDuty(BLDC_BP, 0,MOTOR_FREQUENCY);                  // P3_PWM_P输出0
        ATOM_PWM_SetDuty(BLDC_CP, BLDC_duty,MOTOR_FREQUENCY);  // P5_PWM_P输出PWM
        PIN_Write(BLDC_AI,0);              //关闭
        PIN_Write(BLDC_BI,1);              //关闭
        PIN_Write(BLDC_CI,0);              //打开
        break;

      default:
        LQ_BLDCStop();
        break;
    }
}

/**LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void LQ_BLDCReversal(void);
@功能说明：控制无刷电机反转
@参数说明：无
@函数返回：无
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ**/
void LQ_BLDCReversal(void)
{
    switch(Get_Hall_State())
    {
      case 1:
        //C->B
        ATOM_PWM_SetDuty( BLDC_AP, 0,MOTOR_FREQUENCY);                    // P1_PWM_P关闭
        ATOM_PWM_SetDuty( BLDC_BP, 0,MOTOR_FREQUENCY);                  // P3_PWM_P输出0
        ATOM_PWM_SetDuty( BLDC_CP, -BLDC_duty,MOTOR_FREQUENCY);  // P5_PWM_P输出PWM
        PIN_Write( BLDC_AI, 0);                  //关闭
        PIN_Write( BLDC_BI, 1);                  //关闭
        PIN_Write( BLDC_CI, 0);                  //打开
        break;

      case 2:
        //A->C
        ATOM_PWM_SetDuty( BLDC_AP, -BLDC_duty,MOTOR_FREQUENCY);   // P231输出PWM
        ATOM_PWM_SetDuty( BLDC_BP, 0,MOTOR_FREQUENCY);                     // P3_PWM_P关闭
        ATOM_PWM_SetDuty( BLDC_CP, 0,MOTOR_FREQUENCY);                   // P5_PWM_P输出0
        PIN_Write( BLDC_AI, 0);                   //打开
        PIN_Write( BLDC_BI, 0);                   //关闭
        PIN_Write( BLDC_CI, 1);                   //关闭
        break;

      case 3:
        //A->B
        ATOM_PWM_SetDuty( BLDC_AP, -BLDC_duty,MOTOR_FREQUENCY);  // P1_PWM_P输出0
        ATOM_PWM_SetDuty( BLDC_BP, 0,MOTOR_FREQUENCY);                  // P3_PWM_P输出PWM
        ATOM_PWM_SetDuty( BLDC_CP, 0,MOTOR_FREQUENCY);                    // P5_PWM_P输出0
        PIN_Write( BLDC_AI, 0);                  //打开
        PIN_Write( BLDC_BI, 1);                  //关闭
        PIN_Write( BLDC_CI, 0);                  //关闭
        break;

      case 4:
        //B->A
        ATOM_PWM_SetDuty( BLDC_AP, 0,MOTOR_FREQUENCY);                   // P1_PWM_P输出0
        ATOM_PWM_SetDuty( BLDC_BP, -BLDC_duty,MOTOR_FREQUENCY);   // P3_PWM_P输出PWM
        ATOM_PWM_SetDuty( BLDC_CP, 0,MOTOR_FREQUENCY);                     // P5_PWM_P关闭
        PIN_Write( BLDC_AI, 1);                   //关闭
        PIN_Write( BLDC_BI, 0);                   //打开
        PIN_Write( BLDC_CI, 0);                   //关闭
        break;

      case 5:
        //C->A
        ATOM_PWM_SetDuty( BLDC_AP, 0,MOTOR_FREQUENCY);  // P1_PWM_P输出PWM
        ATOM_PWM_SetDuty( BLDC_BP, 0,MOTOR_FREQUENCY);                // P3_PWM_P输出0
        ATOM_PWM_SetDuty( BLDC_CP, -BLDC_duty,MOTOR_FREQUENCY);                // P5_PWM_P输出0
        PIN_Write( BLDC_AI, 1);              //关闭
        PIN_Write( BLDC_BI, 0);              //关闭
        PIN_Write( BLDC_CI, 0);              //打开
        break;

      case 6:
        //B->C
        ATOM_PWM_SetDuty( BLDC_AP, 0,MOTOR_FREQUENCY);      // P1_PWM_P输出PWM
        ATOM_PWM_SetDuty( BLDC_BP, -BLDC_duty,MOTOR_FREQUENCY);                      // P3_PWM_P输出0
        ATOM_PWM_SetDuty( BLDC_CP, 0,MOTOR_FREQUENCY);                      // P5_PWM_P输出0
        PIN_Write( BLDC_AI, 0);              //关闭
        PIN_Write( BLDC_BI, 0);              //打开
        PIN_Write( BLDC_CI, 1);              //关闭
        break;
      default:
        LQ_BLDCStop();
        break;
    }
}


/**LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void LQ_BLDCStop(void);
@功能说明：关闭无刷电机，停止运行
@参数说明：无
@函数返回：无
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ**/
void LQ_BLDCStop(void)
{
    PIN_Write(BLDC_AI,1);   //停止时，默认下桥打开，有刹车功能，推动时有阻力
    PIN_Write(BLDC_BI,1);   //输出高电平，下桥打开，启用刹车，注意该刹车为停车后的刹车
    PIN_Write(BLDC_CI,1);   //不启用刹车时，将电平改为0即可，即P2P4P6输出低电平时，刹车失效
    ATOM_PWM_SetDuty(BLDC_AP, 0,MOTOR_FREQUENCY);
    ATOM_PWM_SetDuty(BLDC_BP, 0,MOTOR_FREQUENCY);
    ATOM_PWM_SetDuty(BLDC_CP, 0,MOTOR_FREQUENCY);
    BLDC_duty = 0;
//    g_sBLDCMag.duty = 0;
//    g_sBLDCMag.bldc_state = state_idle;
}

/**LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void LQ_BLDCCtrl(void);
@功能说明：控制无刷电机
@参数说明：无
@函数返回：无
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ**/
void LQ_BLDCCtrl(void)
{
   if(BLDC_duty>0)
   {
       LQ_BLDCCorotation();
   }
   else if(BLDC_duty<0)
   {
       LQ_BLDCReversal();
   }
   else
   {
       LQ_BLDCStop();
   }
}

int S_PWM=0;
unsigned int RFx=0;
int ENC = 0,S_ENC=0;
float MPS =0,Vel=0;
int32_t NowTime =0;
float EMC_MAX=0;
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void TestMotorBLDC (void);
@功能说明：无刷电机测试函数
@参数说明：无
@函数返回：无
@备    注：
接线说明：
HAL接线：HA接P20_3   HB接P20_0   HC接P33_7
PWM口接线：P1接23_1  P2接21_4 P3接32_4 P4接21_3 P5接21_2 P6接20_8
电机接线：A接电机的橙色线  B接电机的黄色线 C接电机的蓝色线
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TestMotorBLDC (void)
{
    char txt[32];
    short duty = 0;
    uint8_t HALL;

    BLDC_HallInit();
    Display_Init();
    Display_CLS(U16_WHITE);
    BLDCInit();
    ENC_InitConfig(ENC2_InPut_P02_8, ENC2_Dir_P33_5);
    CCU6_InitConfig_u(CCU60, CCU6_Channel1, 20);  // 20us进入一次中断

    while(1)
    {
        if (KEY_Read(KEY0) == 0) // 按下KEY0键，占空比减小
        {
            if (duty > -ATOM_PWM_MAX)
                duty -= 100;
            while(KEY_Read(KEY0) == 0);
        }
        if (KEY_Read(KEY2) == 0) // 按下KEY2键，占空比加大
        {
            if (duty < ATOM_PWM_MAX) // 满占空比为12500
                duty += 100;
            while(KEY_Read(KEY2) == 0);
        }
        if (KEY_Read(KEY1) == 0) // 按下KEY1键，占空比中值
        {
            duty = 0;

        }
        HALL = Get_Hall_State();
        BLDC_duty = duty;
        sprintf(txt, "HALL:%05d",HALL);
        Display_showString(0, 3, txt, U16_WHITE, U16_BLACK, 16);; // 将txt中 内容显示出来

        sprintf(txt, "PWM: %05d;", duty);
        Display_showString(0, 5, txt, U16_WHITE, U16_BLACK, 16); // 字符串显示


    }
}
