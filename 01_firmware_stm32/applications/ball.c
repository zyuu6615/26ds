#include "ball.h"
#include "vision.h"
#include "servo.h"
#include "pid.h"

#include <math.h>

static PID_Controller s_pid;

static float    s_target_cm   = BALL_TARGET_CM;
static float    s_pos_cm      = BALL_TARGET_CM;
static float    s_vel_cm_s    = 0.0f;
static float    s_output_us   = 0.0f;
static float    s_accel_ff    = 0.0f;
static float    s_curve_ff    = 0.0f;

static uint32_t s_last_frames  = 0;
static uint32_t s_last_good_ms = 0;
static uint8_t  s_enabled      = 0;
static uint8_t  s_tracking     = 0;
static uint8_t  s_have_prev    = 0;
static float    s_prev_pos_cm  = 0.0f;

static void Ball_GoLevel(void)
{
  Servo_SetPulseUs(SERVO_LEVEL_US);
  PID_Reset(&s_pid);

  s_output_us = 0.0f;
  s_vel_cm_s  = 0.0f;
  s_have_prev = 0;
  s_tracking  = 0;
}

void Ball_Init(void)
{
  PID_Init(&s_pid, BALL_KP, BALL_KI, BALL_KD, BALL_DT_MAX_S);
  PID_SetOutputLimits(&s_pid, -BALL_OUTPUT_LIMIT_US, BALL_OUTPUT_LIMIT_US);
  PID_SetIntegralLimit(&s_pid, BALL_INTEGRAL_LIMIT_US);

  s_target_cm    = BALL_TARGET_CM;
  s_pos_cm       = BALL_TARGET_CM;
  s_last_frames  = Vision_GetFrameCount();
  s_last_good_ms = HAL_GetTick();
  s_enabled      = 0;

  Ball_GoLevel();
}

void Ball_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t frames;

  if (!s_enabled)
  {
    return;
  }

  frames = Vision_GetFrameCount();
  if (frames != s_last_frames)
  {
    const Vision_Ball *b = Vision_GetBall();

    s_last_frames = frames;

    if ((b->valid != 0U) && (b->confidence >= BALL_MIN_CONFIDENCE))
    {
      float dt = (float)(now - s_last_good_ms) / 1000.0f;

      if (dt < BALL_DT_MIN_S)
      {
        dt = BALL_DT_MIN_S;
      }
      else if (dt > BALL_DT_MAX_S)
      {
        dt = BALL_DT_MAX_S;
      }

      s_pos_cm       = b->x_cm;
      s_last_good_ms = now;

      if (s_have_prev)
      {
        float raw_vel = (s_pos_cm - s_prev_pos_cm) / dt;

        s_vel_cm_s += BALL_VEL_LPF * (raw_vel - s_vel_cm_s);
      }
      else
      {
        s_vel_cm_s  = 0.0f;
        s_have_prev = 1;
      }
      s_prev_pos_cm = s_pos_cm;

      s_pid.dt    = dt;
      s_output_us = PID_Update(&s_pid, s_target_cm, s_pos_cm);

      {
        float out = s_output_us;

#if BALL_FF_ENABLE
        out -= BALL_FF_US_PER_RPMS * s_accel_ff;
#endif
#if BALL_FF_CURVE_ENABLE
        out -= BALL_FF_CURVE_GAIN * s_curve_ff;
#endif

        Servo_SetPulseUs((uint16_t)(SERVO_LEVEL_US +
                                    (int16_t)(BALL_OUTPUT_SIGN * out)));
      }
      s_tracking = 1;
    }
  }

  if ((now - s_last_good_ms) > BALL_TIMEOUT_MS)
  {
    Ball_GoLevel();
  }
}

void Ball_Enable(uint8_t on)
{
  if (on && !s_enabled)
  {
    s_last_frames  = Vision_GetFrameCount();
    s_last_good_ms = HAL_GetTick();
    Ball_GoLevel();
  }

  s_enabled = (on != 0U);

  if (!s_enabled)
  {
    Ball_GoLevel();
  }
}

uint8_t Ball_IsEnabled(void)
{
  return s_enabled;
}

void Ball_SetTarget(float cm)
{
  s_target_cm = cm;
}

void Ball_SetAccelFF(float rpm_per_s)
{
  s_accel_ff = rpm_per_s;
}

void Ball_SetCurveFF(float v_times_diff)
{
  s_curve_ff = v_times_diff;
}

float Ball_GetTarget(void)
{
  return s_target_cm;
}

float Ball_GetPosCm(void)
{
  return s_pos_cm;
}

float Ball_GetVelCmS(void)
{
  return s_vel_cm_s;
}

float Ball_GetOutputUs(void)
{
  return s_output_us;
}

uint8_t Ball_IsTracking(void)
{
  return s_tracking;
}
