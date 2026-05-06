/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编    写：龙邱科技
@E-mail  ：chiusir@163.com
@编译IDE：AURIX Development Studio1.6及以上版本
@使用平台：北京龙邱智能科技TC297TX核心板和母板
@SYS  PLL：300MHz*3 【Crystal】 20.000Mhz
@最后更新：2023年6月6日，持续更新，请关注最新版！
@功能介绍：
@相关信息参考下列地址
@网    站：http://www.lqist.cn
@淘宝店铺：http://longqiu.taobao.com
@软件版本：V1.1 版权所有，单位使用请先联系授权
@程序配套视频地址：https://space.bilibili.com/95313236
****************************************************************
基于iLLD_1_0_1_11_0底层程序,
使用例程的时候，建议采用没有空格的英文路径，切换workspace到当前工程所放文件夹！
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "LQ_PID.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：float constrain_float(float amt, float low, float high)
@功能说明：限幅函数
@参数说明：amt：参数 low：最低值 high：最高值
@函数返回：无
@修改时间：2022/03/15
@调用方法：constrain_float(pid->integrator, -pid->imax, pid->imax);
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
pid_param_t MOTOR_BRUSH_PID;
pid_param_t ICM_PID;
pid_param_t GPS_PID;
pid_param_t MOTOR_PID;
pid_param_t CA_PID;
float constrain_float(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// pid参数初始化函数
void PidInit(pid_param_t * pid)
{
  pid->kp        = 0;
  pid->ki        = 0;
  pid->kd        = 0;
  pid->imax      = 0;
  pid->out_p     = 0;
  pid->out_i     = 0;
  pid->out_d     = 0;
  pid->out       = 0;
  pid->integrator= 0;
  pid->last_error= 0;
  pid->last_derivative   = 0;
  pid->last_t    = 0;
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：float constrain_float(float amt, float low, float high)
@功能说明：pid位置式控制器输出
@参数说明：pid     pid参数  error   pid输入误差
@函数返回：PID输出结果
@修改时间：2022/03/15
@调用方法：
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
float PidLocCtrl(pid_param_t * pid, float error)
{
  /* 累积误差 */
  pid->integrator += error;

  /* 误差限幅 */
  constrain_float(pid->integrator, -pid->imax, pid->imax);


  pid->out_p = pid->kp * error;
  pid->out_i = pid->ki * pid->integrator;
  pid->out_d = pid->kd * (error - pid->last_error);

  pid->last_error = error;

  pid->out = pid->out_p + pid->out_i + pid->out_d;

  return pid->out;
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：float constrain_float(float amt, float low, float high)
@功能说明：pid增量式控制器输出
@参数说明：
@param    pid     pid参数
@param    error   pid输入误差
@函数返回：PID输出结果   注意输出结果已经包涵了上次结果
@修改时间：2020年4月1日
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
float PidIncCtrl(pid_param_t * pid, float error)
{

    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;
    if(pid->out>8500)
        {
          pid->out=8500;
        }
        else if(pid->out<-8500)
        {
          pid->out=-8500;
        }
    return pid->out;
}

