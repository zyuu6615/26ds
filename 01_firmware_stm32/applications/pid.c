/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : 位置式 PID 控制器（微分先行 + 抗积分饱和）
  ******************************************************************************
  */

#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float dt)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;

  pid->out_min        = -1000.0f;
  pid->out_max        =  1000.0f;
  pid->integral_limit =  1000.0f;

  PID_Reset(pid);
}

void PID_SetOutputLimits(PID_Controller *pid, float min, float max)
{
  pid->out_min = min;
  pid->out_max = max;
}

void PID_SetIntegralLimit(PID_Controller *pid, float limit)
{
  pid->integral_limit = limit;
}

void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void PID_Reset(PID_Controller *pid)
{
  pid->integral         = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->first_run        = 1;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement)
{
  float error = setpoint - measurement;
  float p_term;
  float i_term;
  float d_term = 0.0f;
  float out;

  /* ---------- 比例 ---------- */
  p_term = pid->kp * error;

  /* ---------- 积分（带抗饱和限幅） ---------- */
  if (pid->ki > 1e-6f)
  {
    float integral_max;

    pid->integral += error * pid->dt;

    /* 限幅是对积分【项】(ki*integral)做的，这样改 Ki 时限幅含义不变 */
    integral_max = pid->integral_limit / pid->ki;
    if (pid->integral > integral_max)
    {
      pid->integral = integral_max;
    }
    else if (pid->integral < -integral_max)
    {
      pid->integral = -integral_max;
    }
  }
  else
  {
    pid->integral = 0.0f;      /* Ki 关掉时不要留着旧的积分量 */
  }
  i_term = pid->ki * pid->integral;

  /* ---------- 微分（作用于测量值，故取负号） ---------- */
  if (pid->first_run)
  {
    pid->first_run = 0;        /* 第一拍没有历史值，微分记 0 */
  }
  else
  {
    d_term = -pid->kd * (measurement - pid->prev_measurement) / pid->dt;
  }
  pid->prev_measurement = measurement;

  /* ---------- 求和与输出限幅 ---------- */
  out = p_term + i_term + d_term;

  if (out > pid->out_max)
  {
    out = pid->out_max;
  }
  else if (out < pid->out_min)
  {
    out = pid->out_min;
  }

  return out;
}
