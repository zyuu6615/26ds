/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : TB6612FNG 双路电机驱动 —— TIM8_CH1/CH2 调速 + GPIO 方向
  ******************************************************************************
  * @note  TIM8 与 PC6/PC7 的复用由 CubeMX 生成的 MX_TIM8_Init() 完成，
  *        PF0~PF4 由 MX_GPIO_Init() 配置，本文件只负责启动 PWM 与业务逻辑。
  *        因此 Motor_Init() 必须在这两个 MX 初始化之后调用。
  ******************************************************************************
  */

#include "motor.h"
#include "tim.h"

/* ---------------- 引脚定义 ---------------- */
#define MOTOR_PWM_HANDLE        (&htim8)
#define MOTOR_PWMA_CHANNEL      TIM_CHANNEL_1   /* PC6 */
#define MOTOR_PWMB_CHANNEL      TIM_CHANNEL_2   /* PC7 */

#define MOTOR_DIR_GPIO_PORT     GPIOF
#define MOTOR_AIN1_PIN          GPIO_PIN_0
#define MOTOR_AIN2_PIN          GPIO_PIN_1
#define MOTOR_BIN1_PIN          GPIO_PIN_2
#define MOTOR_BIN2_PIN          GPIO_PIN_3

#define MOTOR_STBY_GPIO_PORT    GPIOF
#define MOTOR_STBY_PIN          GPIO_PIN_4

/* ---------------- 每路电机的静态配置 ---------------- */
typedef struct
{
  GPIO_TypeDef *dir_port;
  uint16_t      in1_pin;
  uint16_t      in2_pin;
  uint32_t      channel;       /* TIM8 的 PWM 通道 */
  uint8_t       reversed;      /* 安装方向修正 */
} Motor_Config;

static const Motor_Config s_motor_cfg[MOTOR_NUM] =
{
  [MOTOR_LEFT] = {
    .dir_port = MOTOR_DIR_GPIO_PORT,
    .in1_pin  = MOTOR_AIN1_PIN,
    .in2_pin  = MOTOR_AIN2_PIN,
    .channel  = MOTOR_PWMA_CHANNEL,
    .reversed = MOTOR_LEFT_REVERSED,
  },
  [MOTOR_RIGHT] = {
    .dir_port = MOTOR_DIR_GPIO_PORT,
    .in1_pin  = MOTOR_BIN1_PIN,
    .in2_pin  = MOTOR_BIN2_PIN,
    .channel  = MOTOR_PWMB_CHANNEL,
    .reversed = MOTOR_RIGHT_REVERSED,
  },
};

/* ---------------- 运行时状态 ---------------- */
static uint32_t       s_period      = 0;                  /* ARR + 1，即满占空比计数 */
static int16_t        s_speed[MOTOR_NUM] = {0, 0};        /* 当前设定速度 */
static Motor_StopMode s_stop_mode   = MOTOR_STOP_COAST;
static uint8_t        s_initialized = 0;

/* ---------------- 内部函数 ---------------- */

/**
  * @brief  按方向标志驱动 IN1 / IN2
  */
static void Motor_ApplyDirection(const Motor_Config *cfg, uint8_t in1, uint8_t in2)
{
  HAL_GPIO_WritePin(cfg->dir_port, cfg->in1_pin, in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(cfg->dir_port, cfg->in2_pin, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  把限幅后的速度落到硬件上
  */
static void Motor_Apply(Motor_ID id, int16_t speed)
{
  const Motor_Config *cfg = &s_motor_cfg[id];
  int32_t             mag = (speed >= 0) ? speed : -speed;
  uint8_t             forward;

  if (speed == 0)
  {
    __HAL_TIM_SET_COMPARE(MOTOR_PWM_HANDLE, cfg->channel, 0);
    Motor_ApplyDirection(cfg, (s_stop_mode == MOTOR_STOP_BRAKE),
                              (s_stop_mode == MOTOR_STOP_BRAKE));
    return;
  }

  forward = (speed > 0);
  if (cfg->reversed)
  {
    forward = !forward;
  }
  Motor_ApplyDirection(cfg, forward, !forward);

  /* 速度量程 -> 占空比，用 32 位中间量避免溢出 */
  __HAL_TIM_SET_COMPARE(MOTOR_PWM_HANDLE, cfg->channel,
                        (uint32_t)((mag * (int32_t)s_period) / MOTOR_SPEED_MAX));
}

/* ---------------- 对外接口 ---------------- */

void Motor_Init(void)
{
  /* 方向脚与 STBY 先全部拉低，保证启动 PWM 前电机不会误动作 */
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_PORT,
                    MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | MOTOR_BIN1_PIN | MOTOR_BIN2_PIN,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);

  /* 满占空比计数取自 CubeMX 里设定的 ARR，改动 TIM8 Period 不用同步改这里 */
  s_period = __HAL_TIM_GET_AUTORELOAD(MOTOR_PWM_HANDLE) + 1U;

  __HAL_TIM_SET_COMPARE(MOTOR_PWM_HANDLE, MOTOR_PWMA_CHANNEL, 0);
  __HAL_TIM_SET_COMPARE(MOTOR_PWM_HANDLE, MOTOR_PWMB_CHANNEL, 0);

  if (HAL_TIM_PWM_Start(MOTOR_PWM_HANDLE, MOTOR_PWMA_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(MOTOR_PWM_HANDLE, MOTOR_PWMB_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  s_speed[MOTOR_LEFT]  = 0;
  s_speed[MOTOR_RIGHT] = 0;
  s_stop_mode          = MOTOR_STOP_COAST;
  s_initialized        = 1;

  Motor_Apply(MOTOR_LEFT,  0);
  Motor_Apply(MOTOR_RIGHT, 0);
}

void Motor_Enable(void)
{
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}

void Motor_Disable(void)
{
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
}

void Motor_SetSpeed(Motor_ID id, int16_t speed)
{
  if ((id >= MOTOR_NUM) || (s_initialized == 0))
  {
    return;
  }

  if (speed > MOTOR_SPEED_MAX)
  {
    speed = MOTOR_SPEED_MAX;
  }
  else if (speed < -MOTOR_SPEED_MAX)
  {
    speed = -MOTOR_SPEED_MAX;
  }

  s_speed[id] = speed;
  Motor_Apply(id, speed);
}

void Motor_SetSpeeds(int16_t left, int16_t right)
{
  Motor_SetSpeed(MOTOR_LEFT,  left);
  Motor_SetSpeed(MOTOR_RIGHT, right);
}

int16_t Motor_GetSpeed(Motor_ID id)
{
  return (id < MOTOR_NUM) ? s_speed[id] : 0;
}

void Motor_SetStopMode(Motor_StopMode mode)
{
  s_stop_mode = mode;

  /* 已经停着的电机立刻切到新的停车方式 */
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    if (s_speed[i] == 0)
    {
      Motor_Apply((Motor_ID)i, 0);
    }
  }
}

void Motor_Brake(Motor_ID id)
{
  if ((id >= MOTOR_NUM) || (s_initialized == 0))
  {
    return;
  }

  s_speed[id] = 0;
  __HAL_TIM_SET_COMPARE(MOTOR_PWM_HANDLE, s_motor_cfg[id].channel, 0);
  Motor_ApplyDirection(&s_motor_cfg[id], 1, 1);
}

void Motor_Coast(Motor_ID id)
{
  if ((id >= MOTOR_NUM) || (s_initialized == 0))
  {
    return;
  }

  s_speed[id] = 0;
  __HAL_TIM_SET_COMPARE(MOTOR_PWM_HANDLE, s_motor_cfg[id].channel, 0);
  Motor_ApplyDirection(&s_motor_cfg[id], 0, 0);
}

void Motor_BrakeAll(void)
{
  Motor_Brake(MOTOR_LEFT);
  Motor_Brake(MOTOR_RIGHT);
}

void Motor_CoastAll(void)
{
  Motor_Coast(MOTOR_LEFT);
  Motor_Coast(MOTOR_RIGHT);
}
