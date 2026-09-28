#ifndef __KEY_H
#define __KEY_H

#include "main.h"

/**
  ******************************************************************************
  * 4 路按键
  *
  *   KEY1 -> PC2        KEY2 -> PC0        KEY3 -> PF9        KEY4 -> PF7
  *
  * CubeMX 里已配成上拉输入(GPIO_PULLUP)，按键另一端接地，所以按下读到低电平。
  * 引脚与时钟由 MX_GPIO_Init() 初始化，本模块只做消抖与边沿检测。
  ******************************************************************************
  */

typedef enum
{
  KEY1 = 0,
  KEY2,
  KEY3,
  KEY4,
  KEY_NUM
} Key_ID;

/* 连续读到相同电平这么多次才认账。按 Key_Scan() 的调用周期算消抖时间，
   10ms 调用一次 x 3 = 30ms，足够滤掉机械抖动又不影响手感 */
#define KEY_DEBOUNCE_COUNT      3

/* 清空内部状态。可选，Key_Scan() 本身能自行收敛 */
void Key_Init(void);

/* 扫描一次按键。需周期性调用，建议 10ms */
void Key_Scan(void);

/* 按住自动连发：按下超过 DELAY 之后，每 PERIOD 次扫描产生一次事件。
   按 Key_Scan() 10ms 的调用周期算 = 按住 0.4s 后开始，每 10ms 连发一次 */
#define KEY_REPEAT_DELAY_TICKS  40
#define KEY_REPEAT_PERIOD_TICKS 1

/* 取走一次"按下"事件(下降沿)，读到后自动清除，不会重复触发 */
uint8_t Key_WasPressed(Key_ID id);

/* 当前是否按住 */
uint8_t Key_IsDown(Key_ID id);

#endif /* __KEY_H */
