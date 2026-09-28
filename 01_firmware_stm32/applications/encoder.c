/**
  ******************************************************************************
  * @file           : encoder.c
  * @brief          : 正交编码器读取 —— TIM3(左轮) / TIM1(右轮) 编码器接口
  ******************************************************************************
  * @note  定时器与引脚复用由 CubeMX 生成的 MX_TIM1_Init() / MX_TIM3_Init()
  *        完成，本文件只负责启动计数、把 16 位 CNT 展开成 32 位累计值。
  ******************************************************************************
  */

#include "encoder.h"
#include "tim.h"

/* ---------------- 每路编码器的静态配置 ---------------- */
typedef struct
{
  TIM_HandleTypeDef *htim;
  int8_t             sign;     /* 计数方向修正：+1 或 -1 */
} Encoder_Config;

static const Encoder_Config s_encoder_cfg[ENCODER_NUM] =
{
  /* 左轮 PB4 / PA7 */
  [ENCODER_LEFT]  = { .htim = &htim3, .sign = ENCODER_LEFT_REVERSED  ? -1 : 1 },
  /* 右轮 PE9 / PE11 */
  [ENCODER_RIGHT] = { .htim = &htim1, .sign = ENCODER_RIGHT_REVERSED ? -1 : 1 },
};

/* ---------------- 运行时状态 ---------------- */
static uint16_t s_last_cnt[ENCODER_NUM]   = {0, 0};   /* 上次读到的硬件 CNT */
static int32_t  s_total[ENCODER_NUM]      = {0, 0};   /* 补齐回绕后的累计计数 */
static int32_t  s_delta_mark[ENCODER_NUM] = {0, 0};   /* 上次取增量时的累计值 */
static uint8_t  s_initialized             = 0;

/* ---------------- 内部函数 ---------------- */

/**
  * @brief  采样一次硬件计数器，把增量累加到 32 位总计数
  * @note   CNT 与上次值都按 16 位无符号相减、再当作有符号 16 位解释，
  *         回绕(65535 -> 0 或 0 -> 65535)会自然得到 ±1，无需特判方向位。
  */
static void Encoder_Sample(Encoder_ID id)
{
  const Encoder_Config *cfg = &s_encoder_cfg[id];
  uint16_t              cnt = (uint16_t)__HAL_TIM_GET_COUNTER(cfg->htim);
  int16_t               diff;

  diff = (int16_t)(cnt - s_last_cnt[id]);
  s_last_cnt[id] = cnt;
  s_total[id]   += (int32_t)diff * cfg->sign;
}

/* ---------------- 对外接口 ---------------- */

void Encoder_Init(void)
{
  for (uint8_t i = 0; i < ENCODER_NUM; i++)
  {
    if (HAL_TIM_Encoder_Start(s_encoder_cfg[i].htim, TIM_CHANNEL_ALL) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(s_encoder_cfg[i].htim, 0);

    s_last_cnt[i]   = 0;
    s_total[i]      = 0;
    s_delta_mark[i] = 0;
  }

  s_initialized = 1;
}

int32_t Encoder_GetCount(Encoder_ID id)
{
  if ((id >= ENCODER_NUM) || (s_initialized == 0))
  {
    return 0;
  }

  Encoder_Sample(id);
  return s_total[id];
}

int32_t Encoder_GetDelta(Encoder_ID id)
{
  int32_t delta;

  if ((id >= ENCODER_NUM) || (s_initialized == 0))
  {
    return 0;
  }

  Encoder_Sample(id);
  delta             = s_total[id] - s_delta_mark[id];
  s_delta_mark[id]  = s_total[id];
  return delta;
}

float Encoder_GetRPM(Encoder_ID id, uint32_t sample_ms)
{
  int32_t delta;

  if (sample_ms == 0U)
  {
    return 0.0f;
  }

  delta = Encoder_GetDelta(id);

  /* 每分钟转数 = 增量 / 每圈计数 / 采样秒数 * 60 */
  return ((float)delta * 60000.0f) / ((float)ENCODER_COUNTS_PER_REV * (float)sample_ms);
}

void Encoder_Reset(Encoder_ID id)
{
  if ((id >= ENCODER_NUM) || (s_initialized == 0))
  {
    return;
  }

  __HAL_TIM_SET_COUNTER(s_encoder_cfg[id].htim, 0);
  s_last_cnt[id]   = 0;
  s_total[id]      = 0;
  s_delta_mark[id] = 0;
}

void Encoder_ResetAll(void)
{
  Encoder_Reset(ENCODER_LEFT);
  Encoder_Reset(ENCODER_RIGHT);
}
