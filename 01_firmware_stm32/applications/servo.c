/**
  ******************************************************************************
  * @file           : servo.c
  * @brief          : 180 度舵机驱动 (TIM4_CH4 / PD15) + 往复自检演示
  ******************************************************************************
  * @note  TIM4 与 PD15 的复用由 CubeMX 生成的 MX_TIM4_Init() 完成，
  *        因此 Servo_Init() 必须在它之后调用。
  ******************************************************************************
  */

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
static uint8_t  s_at_limit  = 0;    /* 上一次设定是否被安全限位截断 */

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

  /* 上电回到实测的水平点。注意不能用 (MIN+MAX)/2 —— 连杆几何决定了水平点
     并不在行程中点上(1050/2400 的中点是 1725，而水平是 1640) */
  Servo_SetPulseUs(SERVO_LEVEL_US);
}

/**
  * @brief  所有设定的唯一出口 —— 保护就在这里
  * @note   夹的是【机构安全行程】而不是舵机的电气量程。舵机能转到 500us
  *         不代表机构受得了，越界的后果是持续堵转烧机或掰断连杆。
  *         SetAngle / StepUs 都经由本函数，没有绕过去的路径。
  */
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

  /* 角度跟着脉宽一起更新，免得用 SetPulseUs 微调之后 GetAngle 读到旧值 */
  s_angle_deg = (float)(us - SERVO_MIN_PULSE_US) / SERVO_US_PER_DEG;

  /* TIM4 是 1MHz 计数，比较值的单位就是微秒 */
  __HAL_TIM_SET_COMPARE(SERVO_TIM_HANDLE, SERVO_TIM_CHANNEL, us);
}

void Servo_StepUs(int16_t delta_us)
{
  /* 先在 32 位里算再夹紧，避免加减越过 0 或 65535 时绕回去；
     真正的安全限幅在 Servo_SetPulseUs() 里做 */
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

  /* 只做换算，越界交给 Servo_SetPulseUs() 统一处理，
     这样 s_at_limit 才能如实反映"指令被截断了" */
  if (deg < 0.0f)
  {
    deg = 0.0f;
  }
  else if (deg > SERVO_MAX_ANGLE_DEG)
  {
    deg = SERVO_MAX_ANGLE_DEG;
  }

  us = (float)SERVO_MIN_PULSE_US + deg * SERVO_US_PER_DEG;

  /* s_angle_deg 由 Servo_SetPulseUs() 按实际脉宽反算，读回来的是真正做到的角度 */
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

  /* 扫过的量由"本阶段已经过去多久"算出来，与本函数被调用的频率无关 */
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

/**
  ******************************************************************************
  * 机构安全行程的标定步骤
  *
  * 目标：找出 SERVO_SAFE_MIN_US / SERVO_SAFE_MAX_US 这两个数。它们是整个
  *       保护机制的全部 —— 舵机没有反馈，程序无法"发现顶死了再退回来"。
  *
  * 1. 【断开连杆】先把舵机摇臂和管子脱开，单独通电。
  *    这一步的意义：万一 500~2500us 的默认范围本身就超出机构，
  *    连着连杆试就是直接往死点上顶。
  *
  * 2. 把 app.c 的 APP_SERVO_ENABLE 改成 1，用 KEY3/KEY4 微调，
  *    屏幕上的 US 就是当前脉宽。确认舵机确实在动、方向符合预期。
  *
  * 3. 【装回连杆】，把舵机停在管子大致水平的位置再拧紧摇臂。
  *    记下这个脉宽，它就是后面小球闭环的零点。
  *
  * 4. 用 KEY3 一点点往上加，直到管子碰到机构的物理死点【立刻停手】。
  *    记下这个脉宽，减去 5~10 度对应的量(约 55~110us)作为 SERVO_SAFE_MAX_US。
  *    KEY4 方向同理得到 SERVO_SAFE_MIN_US。
  *
  *    余量不能省：机构装配误差、齿轮间隙、舵机自身定位误差加起来能有好几度，
  *    贴着死点填的话，某次上电就会顶住。判断"到死点了"的信号是舵机开始
  *    发出持续的嗡嗡声或明显发烫 —— 听到就说明已经在堵转了，赶紧退回来。
  *
  * 5. 填好后重新编译，再跑一次 KEY3/KEY4 到两端，确认舵机停在限位处
  *    【安静无声】。有嗡嗡声就说明余量还不够。
  *
  * 另外两条与代码无关但同样重要：
  *   - 舵机单独供电，不要从 3.3V 逻辑电源取。堵转瞬间电流 1A 以上，
  *     会把主控拉复位。
  *   - 电源地与主控地务必共地，否则 PWM 信号没有参考电平。
  ******************************************************************************
  */
