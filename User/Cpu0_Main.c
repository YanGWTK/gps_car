/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：AURIX Development Studio 1.10.2及以上版本
@使用平台：北京龙邱智能科技TC297TX核心板和母板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V5.1 版权所有，单位使用请先联系授权

@修改日期：2024-12-17
@修改内容：
@注意事项：
    导入工程的时候路径要为纯英文,不要有空格、特殊符合和中文
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

/*
 *
 * 需要在config.h里面打开对应自己的屏幕
 *
 *
 *
 * */

//================================ 系统代码 ================================//
#include "include.h"                 //头文件
IfxCpu_syncEvent g_cpuSyncEvent = 0; // 核心同步所需要的变量

//================================ 用户代码 ================================//

int core0_main(void)
{
    //================================ 系统代码 ================================//
    cpu_init();                        // 等待cpu初始化
    IfxCpu_emitEvent(&g_cpuSyncEvent); // 等待cpu同步
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);
    UART_InitConfig(UART0_RX_P14_1, UART0_TX_P14_0, 115200); // 初始化上位机串口
 UART_InitConfig(UART1_RX_P20_9, UART1_TX_P15_0, 115200); // 接收北斗模块坐标
    //================================ 用户代码 ================================//
    /* 初始化 */

    // 在config.h里面配置自己的屏幕参数，然后需要吧BD的读取函数，放在UART1读取中断里面,串口中断在irq.c里面
    // 北斗读取函数BD_Read();
     char txt[30];
     gps_Init();
     IMU_Init();
//     BLDC_motor_Init();
     motor_Init();
     Display_Init();
     servo_Init();
     CAMERA_Init(50);
     Display_CLS(U16_BLACK);
     Delay_Ms(100);
//     PID_PARA_Init();
     GPIO_BEEP_Init();
//     point_get();
//     Delay_Ms(100);
//     speed_GET();
//     Delay_Ms(100);
//     GPS_PID.kp=0.36;
//     GPS_PID.kd=0.14;
     GPS_PID.kp=0.54;//舵机pid
     GPS_PID.kd=0.52;
     MOTOR_PID.kp=0.6;//电机PID
     MOTOR_PID.ki=0.3;
//     MOTOR_PID.kp=1.0;
//          MOTOR_PID.ki=1.2;
     CA_PID.kp=0.23;//摄像头舵机PID
     CA_PID.kd=0.3;

    /* 主循环 */
    while (1)
    {
//cpu0是主程序 cpu1为惯导，需要自行开启
        Menu();//菜单

    }
}

