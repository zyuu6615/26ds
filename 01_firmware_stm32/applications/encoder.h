#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* ================= 正交编码器接口 =================
   左轮 E1A/E1B -> PB4/PA7  TIM3_CH1/CH2  AF2
   右轮 E2A/E2B -> PE9/PE11 TIM1_CH1/CH2  AF1

   CubeMX 配为 Encoder Interface / TIM_ENCODERMODE_TI12，
   即 AB 相双边沿四倍频：输出轴一圈计数 = 线数 x 4 x 减速比。

   TIM1/TIM3 为 16 位计数器，硬件 CNT 在 0~65535 间回绕，
   本模块在软件里补成 32 位累计值，所以至少要每 32767 个计数调用一次 */

/* 输出轴(轮子)转一圈的计数值 = 500ppr x 4(四倍频) x 28(减速比) */
#define ENCODER_COUNTS_PER_REV      56000

/* 计数方向修正：轮子正转时读到负数，就把对应项改成 1 */
#define ENCODER_LEFT_REVERSED       0
#define ENCODER_RIGHT_REVERSED      1

/* ================= 类型 ================= */

typedef enum
{
  ENCODER_LEFT  = 0,
  ENCODER_RIGHT = 1,
  ENCODER_NUM
} Encoder_ID;

/* ================= 接口 ================= */

/* 须在 MX_TIM1_Init() / MX_TIM3_Init() 之后调用 */
void Encoder_Init(void);

/* 上电(或上次 Reset)以来的累计计数，正负代表转向 */
int32_t Encoder_GetCount(Encoder_ID id);

/* 距上次调用本函数的增量计数，用于定周期测速 */
int32_t Encoder_GetDelta(Encoder_ID id);

/* 由增量计数换算转速，sample_ms 为两次采样的间隔(毫秒) */
float Encoder_GetRPM(Encoder_ID id, uint32_t sample_ms);

/* 累计值清零，硬件 CNT 一并清零 */
void Encoder_Reset(Encoder_ID id);
void Encoder_ResetAll(void);

#endif /* __ENCODER_H */
