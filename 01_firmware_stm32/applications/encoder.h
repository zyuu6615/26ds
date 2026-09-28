#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/**
  ******************************************************************************
  * 正交编码器接口 (STM32F407ZGT6)
  *
  *   编码器      信号   STM32   定时器通道    复用
  *   左轮        E1A    PB4     TIM3_CH1     AF2
  *   左轮        E1B    PA7     TIM3_CH2     AF2
  *   右轮        E2A    PE9     TIM1_CH1     AF1
  *   右轮        E2B    PE11    TIM1_CH2     AF1
  *
  * 两路都是 CubeMX 里的 Encoder Interface / TIM_ENCODERMODE_TI12，即 AB 相
  * 双边沿四倍频计数：电机转一圈的计数 = 编码器线数 x 4 x 减速比。
  *
  * TIM1 / TIM3 都是 16 位计数器，硬件 CNT 会在 0~65535 之间回绕；本模块在软件
  * 里把回绕补成 32 位累计值，因此 Encoder_GetCount() 或 Encoder_GetDelta()
  * 至少要每 32767 个计数调用一次。
  * MG513 空载 370rpm 时约 345000 计数/秒，对应 95ms —— 常规 5~20ms 的控制
  * 周期有足够余量，但不要把编码器读取放进比这更慢的低优先级任务里。
  ******************************************************************************
  */

/* 输出轴(轮子)转一圈的计数值 = 编码器线数 x 4(四倍频) x 减速比。
   本车：MG513 + GMR 编码器 -> 500ppr * 4 * 28 = 56000。
   若换成同型号的 13ppr 霍尔编码器版本，则为 13 * 4 * 28 = 1456。 */
#define ENCODER_COUNTS_PER_REV      56000

/* 计数方向修正：轮子正转时读到负数，就把对应项改成 1 */
#define ENCODER_LEFT_REVERSED       0
#define ENCODER_RIGHT_REVERSED      1

typedef enum
{
  ENCODER_LEFT  = 0,
  ENCODER_RIGHT = 1,
  ENCODER_NUM
} Encoder_ID;

/* 启动两路编码器计数。须在 MX_TIM1_Init() / MX_TIM3_Init() 之后调用 */
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
