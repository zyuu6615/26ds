#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include "main.h"

/**
  ******************************************************************************
  * 感为科技 8 路灰度传感器 (I2C1)
  *
  *   SCL  -> PB8         SDA -> PB9         供电 5V
  *   总线上拉由传感器板上的跳线帽提供(上拉到 5V)，STM32 侧不开内部上拉。
  *   PB8/PB9 是 5V 容忍(FT)引脚，且 I2C 为开漏，接 5V 上拉没有风险。
  *
  *   注意 PB8/PB9 在 F407 上只能复用为 I2C1 —— I2C2 的可选引脚是
  *   PB10/PB11、PF0/PF1、PH4/PH5，其中 PF0/PF1 已被电机方向占用。
  *   所以从 PB10/PB11 换到 PB8/PB9 同时也换了外设，句柄要跟着改成 hi2c1。
  *
  *   地址跳线 AD1/AD0 均未插 -> 硬件地址位 00 -> 7 位地址 0b1001100 = 0x4C
  *   HAL 的 I2C 接口要求传入左移一位后的地址，故常量已经左移。
  ******************************************************************************
  */

#define GRAY_I2C_ADDR           (0x4C << 1)
#define GRAY_CHANNEL_NUM        8

/* ---------------- 命令字 ---------------- */
#define GRAY_CMD_PING           0xAA    /* 同步，正常返回 0x66 */
#define GRAY_PING_REPLY         0x66
#define GRAY_CMD_CH_ENABLE      0xCE    /* 传输通道使能，写 0xFF = 8 路全开 */
#define GRAY_CMD_NORMALIZE      0xCF    /* 通道归一化，写 0xFF = 8 路全开 */

/* !! 待确认 !! 连续读取 8 路归一化模拟值的命令字。
   手册没拿到，这里按最常见的 0xB0 实现：从 0xB0 起连续读 8 字节，
   依次对应 CH0~CH7，归一化后白场 255、黑场 0。
   若手册写的是别的值，只改这一行即可。 */
#define GRAY_CMD_READ_ANALOG    0xB0

/**
  * @brief  初始化传感器：ping 同步 -> 通道使能 -> 归一化使能
  * @retval HAL_OK 成功；HAL_TIMEOUT 表示等不到 0x66(接线/供电/地址问题)
  * @note   上电到能正常应答需要一点时间，函数内部会阻塞轮询，
  *         最长等待 GRAY_PING_TIMEOUT_MS 毫秒。
  */
HAL_StatusTypeDef Gray_Init(void);

/**
  * @brief  一次读回 8 路数据
  * @param  values 长度至少为 GRAY_CHANNEL_NUM 的缓冲区，CH0 在下标 0
  */
HAL_StatusTypeDef Gray_ReadAll(uint8_t *values);

/* ping 同步的最长等待时间。手册要求"一直等到 0x66"，这里加了上限，
   否则传感器没接好时程序会永远卡在 App_Init() 里，没法调试 */
#define GRAY_PING_TIMEOUT_MS    3000

#endif /* __GRAYSCALE_H */
