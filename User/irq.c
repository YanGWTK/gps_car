 #include "include.h"
#include "IfxCcu6_reg.h"


/***************************定时器中断回调函数**********************************/
IFX_INTERRUPT(CCU60_CH0_IRQHandler, CCU60_VECTABNUM, CCU60_CH0_PRIORITY);
IFX_INTERRUPT(CCU60_CH1_IRQHandler, CCU60_VECTABNUM, CCU60_CH1_PRIORITY);
IFX_INTERRUPT(CCU61_CH0_IRQHandler, CCU61_VECTABNUM, CCU61_CH0_PRIORITY);
IFX_INTERRUPT(CCU61_CH1_IRQHandler, CCU61_VECTABNUM, CCU61_CH1_PRIORITY);
// CCU60_CH0中断服务函数
void CCU60_CH0_IRQHandler(void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* 用户代码 */
    if(gyro_offset_flag==1)
       {

//           ICM_Get_Raw_data(&aacx, &aacy, &aacz, &gyrox, &gyroy, &gyroz);
//           IMU_gyro_integral();
        ICM_getEulerianAngles();
//        Attitude_get();
           double i = gyro_angle - gyro_angle_old;
          angle_plan(&i);
             gyro_angle_totol += i;
             gyro_angle_old = gyro_angle;
//           if(gyro_angle>180){gyro_angle=gyro_angle-360;}
//             else if(gyro_angle<-180){gyro_angle=gyro_angle+360;}
//             else if(gyro_angle==-180){gyro_angle=180;}

       }

}

// CCU60_CH1中断服务函数
void CCU60_CH1_IRQHandler(void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t13PeriodMatch);
    if(qd_flag==1)
qd_num++;
    /* 用户代码 */
    gps_get();

//    if(start_flag==1&&use_angle<GPS_data.angle)
//    {
//        use_angle+=temp;
//    }
//    else if(start_flag==1&&use_angle>GPS_data.angle)
//    {
//        use_angle-=temp;
//    }
    if(start_flag==1&&start_flag1==1&&SPEED_VALUE>5000)
         {
//        Q_info_q0 = 1;Q_info_q1 = 0; Q_info_q2 = 0; Q_info_q3 = 0;
//         q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
           use_angle=GPS_data.angle;
//           gyro_angle=GPS_data.angle;
           gyro_m=GPS_data.angle-GPS_gyro_angle;
//           gyro_angle_1=GPS_data.angle;//想靠这个用来补偿imu
         }
//         else if(start_flag==1&&start_flag1==1&&SPEED_VALUE<=5000&&SPEED_VALUE>3000)
//         {
//           float k=(float)(((float)SPEED_VALUE-3000)/2000+0.3);
//           if(k>1) k=1;
//           use_angle=angle_weighting(GPS_gyro_angle,GPS_data.angle,1.0-k,k);
//           gyro_m=use_angle-GPS_gyro_angle;
//         }
//         else if(start_flag==1&&start_flag1==1&&SPEED_VALUE>2000&&SPEED_VALUE<=3000)
//         {
//             use_angle=angle_weighting(GPS_gyro_angle,GPS_data.angle,0.8,0.2);
//             gyro_m=use_angle-GPS_gyro_angle;
//         }
         else
         {
//             use_angle=GPS_gyro_angle;
             use_angle=GPS_gyro_angle+gyro_m;
//             use_angle=GPS_data.angle;
//             use_angle=gyro_angle;
         }
       }
//    point_p.Direction=
//    get_two_points_azimuth(last_data.LAT, last_data.LON, GPS_data.LAT, GPS_data.LON);
//    GPS_data.angle=point_p.Direction;
//    last_data.LAT=GPS_data.LAT;
//    last_data.LON=GPS_data.LON;

// CCU61_CH0中断服务函数
void CCU61_CH0_IRQHandler(void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* 用户代码 */
    encoder_get();
    encoder_totol+=encoder;

}
// CCU61_CH1中断服务函数
void CCU61_CH1_IRQHandler(void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t13PeriodMatch);

    /* 用户代码 */



}

/***************************外部中断中断回调函数**********************************/
/* GPIO外部中断 */
IFX_INTERRUPT(PIN_INT0_IRQHandler, PIN_INT0_VECTABNUM, PIN_INT0_PRIORITY);
IFX_INTERRUPT(PIN_INT1_IRQHandler, PIN_INT1_VECTABNUM, PIN_INT1_PRIORITY);
IFX_INTERRUPT(PIN_INT2_IRQHandler, PIN_INT2_VECTABNUM, PIN_INT2_PRIORITY);
IFX_INTERRUPT(PIN_INT3_IRQHandler, PIN_INT3_VECTABNUM, PIN_INT3_PRIORITY);
IFX_INTERRUPT(PIN_INT4_IRQHandler, PIN_INT4_VECTABNUM, PIN_INT4_PRIORITY);
IFX_INTERRUPT(PIN_INT5_IRQHandler, PIN_INT5_VECTABNUM, PIN_INT5_PRIORITY);
IFX_INTERRUPT(PIN_INT6_IRQHandler, PIN_INT6_VECTABNUM, PIN_INT6_PRIORITY);
IFX_INTERRUPT(PIN_INT7_IRQHandler, PIN_INT7_VECTABNUM, PIN_INT7_PRIORITY);
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：PIN_INT0_IRQHandler中断服务函数
@功能说明：
@参数说明：无
@函数返回：无
@备    注：外部中断0组管脚 使用的中断服务函数
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void PIN_INT0_IRQHandler(void)
{
    /* 用户代码 */

}
//PIN_INT1_IRQHandler中断服务函数
void PIN_INT1_IRQHandler(void)
{
    /* 用户代码 */
}

//PIN_INT2_IRQHandler中断服务函数
void PIN_INT2_IRQHandler(void)
{
    // 用户代码
}

//PIN_INT3_IRQHandler中断服务函数
void PIN_INT3_IRQHandler(void)
{
    // 用户代码
}
//PIN_INT4_IRQHandler中断服务函数
void PIN_INT4_IRQHandler(void)
{
    // 用户代码
}
//PIN_INT5_IRQHandler中断服务函数
void PIN_INT5_IRQHandler(void)
{
    // 用户代码
}
//PIN_INT6_IRQHandler中断服务函数
void PIN_INT6_IRQHandler(void)
{
    // 用户代码
}
//PIN_INT7_IRQHandler中断服务函数
void PIN_INT7_IRQHandler(void)
{
    //已被摄像头占用，不可使用
    DMA_CameraStart(PIN_INT2_PRIORITY);
}
/***************************串口中断回调函数**********************************/
/* UART中断 */
IFX_INTERRUPT(UART0_RX_IRQHandler, UART0_VECTABNUM, UART0_RX_PRIORITY);
IFX_INTERRUPT(UART1_RX_IRQHandler, UART1_VECTABNUM, UART1_RX_PRIORITY);
IFX_INTERRUPT(UART2_RX_IRQHandler, UART2_VECTABNUM, UART2_RX_PRIORITY);
IFX_INTERRUPT(UART3_RX_IRQHandler, UART3_VECTABNUM, UART3_RX_PRIORITY);
IFX_INTERRUPT(UART0_TX_IRQHandler, UART0_VECTABNUM, UART0_TX_PRIORITY);
IFX_INTERRUPT(UART1_TX_IRQHandler, UART1_VECTABNUM, UART1_TX_PRIORITY);
IFX_INTERRUPT(UART2_TX_IRQHandler, UART2_VECTABNUM, UART2_TX_PRIORITY);
IFX_INTERRUPT(UART3_TX_IRQHandler, UART3_VECTABNUM, UART3_TX_PRIORITY);
IFX_INTERRUPT(UART0_ER_IRQHandler, UART0_VECTABNUM, UART0_ER_PRIORITY);
IFX_INTERRUPT(UART1_ER_IRQHandler, UART1_VECTABNUM, UART1_ER_PRIORITY);
IFX_INTERRUPT(UART2_ER_IRQHandler, UART2_VECTABNUM, UART2_ER_PRIORITY);
IFX_INTERRUPT(UART3_ER_IRQHandler, UART3_VECTABNUM, UART3_ER_PRIORITY);

// 串口0 RX中断函数
void UART0_RX_IRQHandler(void)
{
    IfxAsclin_Asc_isrReceive(&g_UartConfig[0]);

    /* 用户代码 */

}
// 串口0 TX中断函数
void UART0_TX_IRQHandler(void)
{
    IfxAsclin_Asc_isrTransmit(&g_UartConfig[0]);

    /* 用户代码 */
}
// 串口0 ER中断函数
void UART0_ER_IRQHandler(void)
{
    IfxAsclin_Asc_isrError(&g_UartConfig[0]);

    /* 用户代码 */
}

// 串口1 RX中断函数
void UART1_RX_IRQHandler(void)
{
    IfxAsclin_Asc_isrReceive(&g_UartConfig[1]);

    /* 用户代码 */
    BD_Read();
//    GPS_data.LAT=point_p.lat;
//    GPS_data.LON=point_p.lon;
//    point_p.Direction= get_two_points_azimuth(last_data.LAT, last_data.LON, GPS_data.LAT, GPS_data.LON);
//    GPS_data.angle=point_p.Direction;
//    last_data.LAT=GPS_data.LAT;
//    last_data.LON=GPS_data.LON;
}
// 串口1 TX中断函数
void UART1_TX_IRQHandler(void)
{
    IfxAsclin_Asc_isrTransmit(&g_UartConfig[1]);

    /* 用户代码 */
}
// 串口1 ER中断函数
void UART1_ER_IRQHandler(void)
{
    IfxAsclin_Asc_isrError(&g_UartConfig[1]);

    /* 用户代码 */
}
// 串口2 RX中断函数
void UART2_RX_IRQHandler(void)
{
    IfxAsclin_Asc_isrReceive(&g_UartConfig[2]);

    /* 用户代码 */
}
// 串口2 TX中断函数
void UART2_TX_IRQHandler(void)
{
    IfxAsclin_Asc_isrTransmit(&g_UartConfig[2]);

    /* 用户代码 */
}
// 串口2 ER中断函数
void UART2_ER_IRQHandler(void)
{
    IfxAsclin_Asc_isrError(&g_UartConfig[2]);

    /* 用户代码 */
}
// 串口3 RX中断函数
void UART3_RX_IRQHandler(void)
{
    IfxAsclin_Asc_isrReceive(&g_UartConfig[3]);

    /* 用户代码 */
}
// 串口3 TX中断函数
void UART3_TX_IRQHandler(void)
{
    IfxAsclin_Asc_isrTransmit(&g_UartConfig[3]);

    /* 用户代码 */
}
// 串口3 ER中断函数
void UART3_ER_IRQHandler(void)
{
    IfxAsclin_Asc_isrError(&g_UartConfig[3]);
    /* 用户代码 */
}


// STM0_CH0中断服务函数
void STM0_CH0_IRQHandler(void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();
    // 清除中断标志
    IfxStm_clearCompareFlag(&MODULE_STM0, g_StmCompareConfig[0].comparator);
    // 开启新的中断配置，开始下次中断
    IfxStm_increaseCompare(&MODULE_STM0, g_StmCompareConfig[0].comparator, g_StmCompareConfig[0].ticks);
    /* 用户代码 */
}
// STM0_CH1中断服务函数
IFX_INTERRUPT(STM0_CH0_IRQHandler, STM0_VECTABNUM, STM0_CH0_PRIORITY);
IFX_INTERRUPT(STM0_CH1_IRQHandler, STM0_VECTABNUM, STM0_CH1_PRIORITY);
IFX_INTERRUPT(STM1_CH0_IRQHandler, STM1_VECTABNUM, STM1_CH0_PRIORITY);
IFX_INTERRUPT(STM1_CH1_IRQHandler, STM1_VECTABNUM, STM1_CH1_PRIORITY);
void STM0_CH1_IRQHandler(void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();
    // 清除中断标志
    IfxStm_clearCompareFlag(&MODULE_STM0, g_StmCompareConfig[1].comparator);
    // 开启新的中断配置，开始下次中断
    IfxStm_increaseCompare(&MODULE_STM0, g_StmCompareConfig[1].comparator, g_StmCompareConfig[1].ticks);
    /* 用户代码 */
}

// STM1_CH0中断服务函数
void STM1_CH0_IRQHandler(void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();
    // 清除中断标志
    IfxStm_clearCompareFlag(&MODULE_STM1, g_StmCompareConfig[2].comparator);
    // 开启新的中断配置，开始下次中断
    IfxStm_increaseCompare(&MODULE_STM1, g_StmCompareConfig[2].comparator, g_StmCompareConfig[2].ticks);
    /* 用户代码 */
}

// STM1_CH1中断服务函数
void STM1_CH1_IRQHandler(void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();
    // 清除中断标志
    IfxStm_clearCompareFlag(&MODULE_STM1, g_StmCompareConfig[3].comparator);
    // 开启新的中断配置，开始下次中断
    IfxStm_increaseCompare(&MODULE_STM1, g_StmCompareConfig[3].comparator, g_StmCompareConfig[3].ticks);
    /* 用户代码 */
}
