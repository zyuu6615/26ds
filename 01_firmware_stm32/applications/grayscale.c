/* ================= 灰度传感器驱动 =================
   感为科技 8 路灰度  I2C1 / PB8 PB9
   复用由 MX_I2C1_Init() 完成，须在其之后调用                      */

#include "grayscale.h"
#include "i2c.h"

#define GRAY_I2C_HANDLE     (&hi2c1)
#define GRAY_IO_TIMEOUT_MS  100     /* 单次 I2C 传输的超时 */
#define GRAY_PING_GAP_MS    5       /* 两次 ping 之间的间隔 */

/* 反复 ping 直到传感器回 0x66 */
static HAL_StatusTypeDef Gray_WaitReady(void)
{
  uint32_t start = HAL_GetTick();
  uint8_t  reply = 0;

  while ((HAL_GetTick() - start) < GRAY_PING_TIMEOUT_MS)
  {
    if (HAL_I2C_Mem_Read(GRAY_I2C_HANDLE, GRAY_I2C_ADDR,
                         GRAY_CMD_PING, I2C_MEMADD_SIZE_8BIT,
                         &reply, 1, GRAY_IO_TIMEOUT_MS) == HAL_OK)
    {
      if (reply == GRAY_PING_REPLY)
      {
        return HAL_OK;
      }
    }

    HAL_Delay(GRAY_PING_GAP_MS);
  }

  return HAL_TIMEOUT;
}

/* 写一个字节到指定命令字 */
static HAL_StatusTypeDef Gray_WriteCmd(uint8_t cmd, uint8_t value)
{
  return HAL_I2C_Mem_Write(GRAY_I2C_HANDLE, GRAY_I2C_ADDR,
                           cmd, I2C_MEMADD_SIZE_8BIT,
                           &value, 1, GRAY_IO_TIMEOUT_MS);
}

HAL_StatusTypeDef Gray_Init(void)
{
  HAL_StatusTypeDef status;

  /* ---------- 1. ping 同步，等传感器上电就绪 ---------- */
  status = Gray_WaitReady();
  if (status != HAL_OK)
  {
    return status;
  }

  /* ---------- 2. 8 路传输通道全部使能 ---------- */
  /* 写 0xFF = 8 路全开 */
  status = Gray_WriteCmd(GRAY_CMD_CH_ENABLE, 0xFF);
  if (status != HAL_OK)
  {
    return status;
  }

  /* ---------- 3. 8 路归一化全部开启 ---------- */
  /* 写 0xFF = 8 路归一化开，白场 255 黑场 0 */
  (void)Gray_WriteCmd(GRAY_CMD_NORMALIZE, 0xFF);

  return HAL_OK;
}

HAL_StatusTypeDef Gray_ReadAll(uint8_t *values)
{
  if (values == NULL)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(GRAY_I2C_HANDLE, GRAY_I2C_ADDR,
                          GRAY_CMD_READ_ANALOG, I2C_MEMADD_SIZE_8BIT,
                          values, GRAY_CHANNEL_NUM, GRAY_IO_TIMEOUT_MS);
}
