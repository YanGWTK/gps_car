/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编   写：龙邱科技
@邮   箱：chiusir@163.com
@编译IDE：AURIX Development Studio 1.10.2及以上版本
@使用平台：北京龙邱智能科技TC297TX核心板和母板
@相关信息参考下列地址
    网      站：http://www.lqist.cn
    淘 宝 店 铺：http://longqiu.taobao.com
    程序配套视频：https://space.bilibili.com/95313236
@软件版本：V2.2 版权所有，单位使用请先联系授权

@修改日期：2024-11-23
@修改内容：
@注意事项：
    导入工程的时候路径要为纯英文,不要有空格、特殊符合和中文
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

//================================ 系统代码 ================================//
#include "include.h"                //头文件
extern IfxCpu_syncEvent g_cpuSyncEvent; //核心同步所需要的变量

//================================ 用户代码 ================================//

int core1_main(void)
{
    //================================ 系统代码 ================================//
    IfxCpu_emitEvent(&g_cpuSyncEvent);  // 等待cpu同步
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    //================================ 用户代码 ================================//
    /* 初始化 */


    /* 主循环 */
    while (1)
        //1190为1m
    {//2350为1m
        //20000为1m
//        if(distance_target>70)
//            motor_crtl(0);
if(encoder>5000)
    motor_crtl(0);
//        char txt[30];
//      this_postion=(float)encoder_totol;
//    add_pos+=(this_postion-last_postion);
//    last_postion=this_postion;
//    if (fabs(add_pos) >= 36384.0)
//            add_pos = 0;
//    if (fabs(add_pos) >= (18.0)) // 大约1cm判断一次
//        {
//            // 开始记忆路径
//            if (save_flag == 1)
//            {
//
//                IMU_buf[0]=(double)in_index;
//                IMU_buf[in_index+1] = gyro_angle_totol;
//                if (in_index < 2000)
//                    in_index++;
//
//            }
//            // 开始复现路径
//            if (save_run == 1)
//            {
//    gd_azimuth=IMU_buf[out_index+1+long_index]-gyro_angle_totol;
//               out_index++;
//    long_index=fabs(SPEED_VALUE/2000);
//
//
//            }
//            add_pos -= 18.0; // 自身清零，这个也是误差积累中重要的一环，不可忽视
//        }

    }
}
