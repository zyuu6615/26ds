#ifndef __VISION_H
#define __VISION_H

#include "main.h"

/* ================= 串口协议 =================
   UART4 / PA0-TX、PA1-RX、115200 8N1

   视觉端每识别成功一次发一帧：
     $BALL,<valid>,<x_cm>,<vx_pixel_s>,<confidence>,<frame_time_ms>*\n
   例：
     $BALL,1,12.34,-5.6,0.87,123456*                    */

#define VISION_RX_RING_SIZE     512
#define VISION_LINE_MAX         64

/* 超过这个时间没收到有效帧就判定视觉断链 */
#define VISION_TIMEOUT_MS       200

/* UART4 中断已由 CubeMX 勾选(STM32CubeMX 生成 IRQHandler 与 NVIC 使能)，
   本模块不再定义一份，否则链接冲突。仅当 CubeMX 未勾选时才设回 1 */
#define VISION_OWN_IRQ_HANDLER  0

/* ================= 数据结构 ================= */

typedef struct
{
  uint8_t  valid;
  float    x_cm;
  float    vx_pixel_s;
  float    confidence;
  uint32_t frame_time_ms;
} Vision_Ball;

/* ================= 接口 ================= */

/* 须在 MX_UART4_Init() 之后调用 */
void Vision_Init(void);

/* 解析缓冲区里已收到的数据，主循环周期调用，非阻塞 */
void Vision_Update(void);

/* 最近一帧的结果。断链时保持为上一帧，用 Vision_IsFresh() 判断可用性 */
const Vision_Ball *Vision_GetBall(void);

uint8_t Vision_IsFresh(void);

/* 距最近一帧过去了多久(ms) */
uint32_t Vision_GetAgeMs(void);

/* 累计成功解析的帧数 / 解析失败与串口错误的次数 */
uint32_t Vision_GetFrameCount(void);
uint32_t Vision_GetErrorCount(void);

#endif /* __VISION_H */
