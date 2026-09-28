/**
  ******************************************************************************
  * @file           : vision.c
  * @brief          : 视觉模块串口接收与解析 (UART4)
  ******************************************************************************
  * @note  UART4 与 PA0/PA1 的复用由 CubeMX 生成的 MX_UART4_Init() 完成，
  *        因此 Vision_Init() 必须在它之后调用。
  *
  *        接收走中断而不是轮询：115200 下 10ms 就能来 115 字节，而 F407 的
  *        UART 只有一个数据寄存器、没有 FIFO，主循环 10ms 才看一次的话
  *        绝大部分字节都会被覆盖丢掉。
  ******************************************************************************
  */

#include "vision.h"
#include "usart.h"

#include <string.h>
#include <stdlib.h>

#define VISION_UART             (&huart4)
#define VISION_RING_MASK        (VISION_RX_RING_SIZE - 1U)

/* ---------------- 中断侧 ---------------- */
static volatile uint8_t  s_ring[VISION_RX_RING_SIZE];
static volatile uint16_t s_head = 0;        /* 中断写 */
static volatile uint16_t s_tail = 0;        /* 主循环读 */
static uint8_t           s_rx_byte = 0;     /* HAL 单字节接收的落点 */

/* ---------------- 解析侧 ---------------- */
static char     s_line[VISION_LINE_MAX];
static uint16_t s_line_len = 0;
static uint8_t  s_in_frame = 0;             /* 是否已经见到帧头 '$' */

static Vision_Ball s_ball        = {0};
static uint32_t    s_last_tick   = 0;
static uint32_t    s_frame_count = 0;
static uint32_t    s_error_count = 0;

/* ================= 中断 ================= */

#if VISION_OWN_IRQ_HANDLER
void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(VISION_UART);
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART4)
  {
    return;
  }

  {
    uint16_t next = (uint16_t)((s_head + 1U) & VISION_RING_MASK);

    if (next != s_tail)         /* 满了就丢弃本字节，不覆盖未读数据 */
    {
      s_ring[s_head] = s_rx_byte;
      s_head = next;
    }
  }

  HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != UART4)
  {
    return;
  }

  /* 溢出/帧错误会让 HAL 把接收状态置回 READY 并停止接收 —— 不重新挂上的话
     通信会就此永久中断。这类错误在对端复位、波特率不匹配时很常见 */
  __HAL_UART_CLEAR_OREFLAG(huart);
  s_error_count++;

  HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

/* ================= 解析 ================= */

/**
  * @brief  跳过一个逗号分隔符
  * @retval 下一字段的起点；当前位置不是逗号则返回 NULL
  */
static const char *Vision_NextField(const char *p)
{
  return (*p == ',') ? (p + 1) : NULL;
}

/**
  * @brief  解析一帧 "$BALL,1,12.34,-5.6,0.87,123456"
  * @note   用 strtof/strtoul 而不是 sscanf("%f")：newlib-nano 的 scanf 默认
  *         不带浮点支持，%f 会解析失败，而 strtof 不受这个限制。
  */
static uint8_t Vision_ParseLine(const char *line)
{
  const char *p = line;
  char       *end;
  Vision_Ball b;

  if (strncmp(p, "$BALL,", 6) != 0)
  {
    return 0;
  }
  p += 6;

  b.valid = (uint8_t)strtoul(p, &end, 10);
  if (end == p) { return 0; }
  p = Vision_NextField(end);
  if (p == NULL) { return 0; }

  b.x_cm = strtof(p, &end);
  if (end == p) { return 0; }
  p = Vision_NextField(end);
  if (p == NULL) { return 0; }

  b.vx_pixel_s = strtof(p, &end);
  if (end == p) { return 0; }
  p = Vision_NextField(end);
  if (p == NULL) { return 0; }

  b.confidence = strtof(p, &end);
  if (end == p) { return 0; }
  p = Vision_NextField(end);
  if (p == NULL) { return 0; }

  b.frame_time_ms = (uint32_t)strtoul(p, &end, 10);
  if (end == p) { return 0; }

  /* 整帧都解出来了才提交，避免半截数据污染上一帧的有效值 */
  s_ball      = b;
  s_last_tick = HAL_GetTick();
  s_frame_count++;
  return 1;
}

/* ================= 对外接口 ================= */

void Vision_Init(void)
{
  s_head       = 0;
  s_tail       = 0;
  s_line_len   = 0;
  s_in_frame   = 0;
  s_frame_count = 0;
  s_error_count = 0;
  s_last_tick  = 0;

  memset(&s_ball, 0, sizeof(s_ball));

#if VISION_OWN_IRQ_HANDLER
  /* 优先级设在 SysTick(15) 之上：串口中断很短，抢占它不影响任何时基 */
  HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
#endif

  HAL_UART_Receive_IT(VISION_UART, &s_rx_byte, 1);
}

void Vision_Update(void)
{
  while (s_tail != s_head)
  {
    char c = (char)s_ring[s_tail];
    s_tail = (uint16_t)((s_tail + 1U) & VISION_RING_MASK);

    /* 帧头随时可以重新同步：中间丢字节也不会让解析器一直错下去 */
    if (c == '$')
    {
      s_line_len = 0;
      s_line[s_line_len++] = c;
      s_in_frame = 1;
      continue;
    }

    if (!s_in_frame)
    {
      continue;
    }

    if (c == '*')               /* 帧尾。'\n' 留给下一帧的 '$' 去重新同步 */
    {
      s_line[s_line_len] = '\0';
      if (!Vision_ParseLine(s_line))
      {
        s_error_count++;
      }
      s_in_frame = 0;
      continue;
    }

    if (s_line_len < (VISION_LINE_MAX - 1U))
    {
      s_line[s_line_len++] = c;
    }
    else
    {
      s_in_frame = 0;           /* 超长，整帧丢弃等下一个 '$' */
      s_error_count++;
    }
  }
}

const Vision_Ball *Vision_GetBall(void)
{
  return &s_ball;
}

uint32_t Vision_GetAgeMs(void)
{
  if (s_frame_count == 0U)
  {
    return VISION_TIMEOUT_MS;   /* 一帧都没收到，直接当作超时 */
  }
  return HAL_GetTick() - s_last_tick;
}

uint8_t Vision_IsFresh(void)
{
  return ((s_frame_count != 0U) && (Vision_GetAgeMs() < VISION_TIMEOUT_MS));
}

uint32_t Vision_GetFrameCount(void)
{
  return s_frame_count;
}

uint32_t Vision_GetErrorCount(void)
{
  return s_error_count;
}
