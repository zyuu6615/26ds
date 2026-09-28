/* ================= 视觉模块串口接收与解析 (UART4) ================= */

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

  /* 错误后必须重新挂上接收，否则通信就此中断 */
  __HAL_UART_CLEAR_OREFLAG(huart);
  s_error_count++;

  HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

/* ================= 解析 ================= */

/* 跳过一个逗号分隔符，返回下一字段起点；当前不是逗号则返回 NULL */
static const char *Vision_NextField(const char *p)
{
  return (*p == ',') ? (p + 1) : NULL;
}

/* 解析一帧 "$BALL,1,12.34,-5.6,0.87,123456"
   用 strtof 而非 sscanf %f：newlib-nano 默认不带浮点 scanf */
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

  /* 整帧解完才提交 */
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
  /* 优先级高于 SysTick(15) */
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

    /* 收到 '$' 即重新同步帧头 */
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

    if (c == '*')               /* 帧尾 */
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
      s_in_frame = 0;           /* 超长，整帧丢弃 */
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
    return VISION_TIMEOUT_MS;   /* 一帧都没收到，视为超时 */
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
