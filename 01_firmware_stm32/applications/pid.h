#ifndef __PID_H
#define __PID_H

#include "main.h"

/* ================= 位置式 PID =================
   out = Kp*e + Ki*∫e dt - Kd*d(measurement)/dt

   微分作用在测量值而不是误差上，避免目标值阶跃时的 derivative kick。
   积分带独立限幅，抗积分饱和 */

/* ================= 类型 ================= */

typedef struct
{
  float   kp;
  float   ki;
  float   kd;
  float   dt;                 /* 控制周期(秒) */

  float   integral;           /* 误差积分，单位：误差 x 秒 */
  float   prev_measurement;   /* 上一次的测量值，用于求微分 */

  float   out_min;
  float   out_max;
  float   integral_limit;     /* 积分项(ki*integral)的幅值上限 */

  uint8_t first_run;          /* 首次运行跳过微分，避免开机尖峰 */
} PID_Controller;

/* ================= 接口 ================= */

/* 初始化并清零内部状态，dt 为固定的控制周期(秒) */
void  PID_Init(PID_Controller *pid, float kp, float ki, float kd, float dt);

/* 输出限幅，应与执行器量程一致 */
void  PID_SetOutputLimits(PID_Controller *pid, float min, float max);

/* 积分项限幅，建议取输出量程的 50%~70% */
void  PID_SetIntegralLimit(PID_Controller *pid, float limit);

/* 在线改参数，不影响已累积的积分 */
void  PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd);

/* 跑一拍闭环，必须以 dt 为周期稳定调用 */
float PID_Update(PID_Controller *pid, float setpoint, float measurement);

/* 清空积分与微分历史，目标大幅跳变或重新使能执行器时调用 */
void  PID_Reset(PID_Controller *pid);

#endif /* __PID_H */
