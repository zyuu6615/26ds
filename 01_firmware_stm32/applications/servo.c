/* ================= 舵机驱动 =================
   180 度舵机  TIM4_CH4 / PD15
   复用由 MX_TIM4_Init() 完成，须在其之后调用                    */

#include "servo.h"
#include "tim.h"

#define SERVO_TIM_HANDLE    (&htim4)
#define SERVO_TIM_CHANNEL   TIM_CHANNEL_4

/* 每度对应多少微秒 */
#define SERVO_US_PER_DEG    ((float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) \
                             / SERVO_MAX_ANGLE_DEG)

/* ---------------- 运行时状态 ---------------- */
static float    s_angle_deg = 0.0f;
static uint16_t s_pulse_us  = SERVO_SAFE_MIN_US;
static uint8_t  s_at_limit  = 0;    /* 上次设定是否被安全限位截断 */

/* 演示用 */
typedef enum
{
  SERVO_DEMO_HOLD_MIN = 0,
  SERVO_DEMO_SWEEP_UP,
  SERVO_DEMO_HOLD_MAX,
  SERVO_DEMO_SWEEP_DOWN
} Servo_DemoState;

static Servo_DemoState s_demo_state = SERVO_DEMO_HOLD_MIN;
static uint32_t        s_demo_tick  = 0;    /* 当前阶段的起始时刻 */

/* ================= 基本接口 ================= */

void Servo_Init(void)
{
  if (HAL_TIM_PWM_Start(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  /* 上电回到实测的水平点，不是行程中点 */
  Servo_SetPulseUs(SERVO_LEVEL_US);
}

/* 所有设定的唯一出口，安全限幅在此处 */
void Servo_SetPulseUs(uint16_t us)
{
  s_at_limit = 0;

  if (us < SERVO_SAFE_MIN_US)
  {
    us = SERVO_SAFE_MIN_US;
    s_at_limit = 1;
  }
  else if (us > SERVO_SAFE_MAX_US)
  {
    us = SERVO_SAFE_MAX_US;
    s_at_limit = 1;
  }

  s_pulse_us = us;

  s_angle_deg = (float)(us - SERVO_MIN_PULSE_US) / SERVO_US_PER_DEG;

  /* 比较值单位即微秒 */
  __HAL_TIM_SET_COMPARE(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL, us);
}

void Servo_StepUs(int16_t delta_us)
{
  int32_t us = (int32_t)s_pulse_us + delta_us;

  if (us < 0)
  {
    us = 0;
  }
  else if (us > 0xFFFF)
  {
    us = 0xFFFF;
  }

  Servo_SetPulseUs((uint16_t)us);
}

uint16_t Servo_GetPulseUs(void)
{
  return s_pulse_us;
}

void Servo_SetAngle(float deg)
{
  float us;

  if (deg < 0.0f)
  {
    deg = 0.0f;
  }
  else if (deg > SERVO_MAX_ANGLE_DEG)
  {
    deg = SERVO_MAX_ANGLE_DEG;
  }

  us = (float)SERVO_MIN_PULSE_US + deg * SERVO_US_PER_DEG;

  Servo_SetPulseUs((uint16_t)us);
}

uint8_t Servo_IsAtLimit(void)
{
  return s_at_limit;
}

float Servo_GetAngle(void)
{
  return s_angle_deg;
}

/* ================= 自检演示 ================= */

void Servo_DemoInit(void)
{
  s_demo_state = SERVO_DEMO_HOLD_MIN;
  s_demo_tick  = HAL_GetTick();

  Servo_SetPulseUs(SERVO_SAFE_MIN_US);
}

void Servo_DemoUpdate(void)
{
  uint32_t now     = HAL_GetTick();
  uint32_t elapsed = now - s_demo_tick;

  float swept = SERVO_DEMO_SPEED_UPS * (float)elapsed / 1000.0f;

  switch (s_demo_state)
  {
    case SERVO_DEMO_HOLD_MIN:
      if (elapsed >= SERVO_DEMO_HOLD_MS)
      {
        s_demo_state = SERVO_DEMO_SWEEP_UP;
        s_demo_tick  = now;
      }
      break;

    case SERVO_DEMO_SWEEP_UP:
      if (((float)SERVO_SAFE_MIN_US + swept) >= (float)SERVO_SAFE_MAX_US)
      {
        Servo_SetPulseUs(SERVO_SAFE_MAX_US);
        s_demo_state = SERVO_DEMO_HOLD_MAX;
        s_demo_tick  = now;
      }
      else
      {
        Servo_SetPulseUs((uint16_t)((float)SERVO_SAFE_MIN_US + swept));
      }
      break;

    case SERVO_DEMO_HOLD_MAX:
      if (elapsed >= SERVO_DEMO_HOLD_MS)
      {
        s_demo_state = SERVO_DEMO_SWEEP_DOWN;
        s_demo_tick  = now;
      }
      break;

    default:    /* SERVO_DEMO_SWEEP_DOWN */
      if (((float)SERVO_SAFE_MAX_US - swept) <= (float)SERVO_SAFE_MIN_US)
      {
        Servo_SetPulseUs(SERVO_SAFE_MIN_US);
        s_demo_state = SERVO_DEMO_HOLD_MIN;
        s_demo_tick  = now;
      }
      else
      {
        Servo_SetPulseUs((uint16_t)((float)SERVO_SAFE_MAX_US - swept));
      }
      break;
  }
}
