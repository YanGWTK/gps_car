#include "LQ_CCU6.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@void LQ_CCU6_Timer (void)
@功能说明：测试程序
@参数说明：无
@函数返回：无
@备   注：核心板上的LED灯闪烁，中断时P10.5/P10.6闪灯
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_CCU6_Timer(void)
{
    CCU6_InitConfig(CCU60, CCU6_Channel0, 100); // CCU6初始化
    CCU6_InitConfig(CCU60, CCU6_Channel1, 200); // CCU6初始化
    CCU6_InitConfig(CCU61, CCU6_Channel0, 400); // CCU6初始化
    CCU6_InitConfig(CCU61, CCU6_Channel1, 800); // CCU6初始化

    // 中断服务函数中翻转LED
    while (1)
    {
        ;
    }
}
