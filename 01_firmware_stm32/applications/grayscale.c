/**
  ******************************************************************************
  * @file           : grayscale.c
  * @brief          : 感为科技 8 路灰度传感器驱动 (I2C1)
  ******************************************************************************
  * @note  I2C1 与 PB8/PB9 的复用由 CubeMX 生成的 MX_I2C1_Init() 完成，
  *        因此 Gray_Init() 必须在它之后调用。
  ******************************************************************************
  */

#include "grayscale.h"
#include "i2c.h"

#define GRAY_I2C_HANDLE     (&hi2c1)
#define GRAY_IO_TIMEOUT_MS  100     /* 单次 I2C 传输的超时 */
#define GRAY_PING_GAP_MS    5       /* 两次 ping 之间的间隔 */

/**
  * @brief  反复 ping 直到传感器回 0x66
  * @note   上电后传感器需要一段时间才能正常应答，这期间发命令会丢数据。
  *         读不到 0x66 时 HAL 会返回错误，属于预期情况，继续重试即可。
  */
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

/**
  * @brief  写一个字节到指定命令字
  */
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
  /* 默认值本来就是 0xFF，显式写一次更保险 */
  status = Gray_WriteCmd(GRAY_CMD_CH_ENABLE, 0xFF);
  if (status != HAL_OK)
  {
    return status;
  }

  /* ---------- 3. 8 路归一化全部开启 ---------- */
  /* 开启后白场输出 255、黑场输出 0，跨探头一致性更好。
     此功能仅 V3.6 及以上固件支持，老固件写失败不影响基本使用，故不返回错误 */
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
