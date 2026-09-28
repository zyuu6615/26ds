#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include "main.h"

/* ================= 感为科技 8 路灰度传感器 (I2C1) =================
   SCL -> PB8    SDA -> PB9    供电 5V
   上拉由传感器板跳线提供，STM32 侧不开内部上拉。
   AD1/AD0 均未插 -> 7 位地址 0x4C(HAL 要求左移一位) */

#define GRAY_I2C_ADDR           (0x4C << 1)
#define GRAY_CHANNEL_NUM        8

/* ---------------- 命令字 ---------------- */
#define GRAY_CMD_PING           0xAA
#define GRAY_PING_REPLY         0x66
#define GRAY_CMD_CH_ENABLE      0xCE    /* 写 0xFF = 8 路全开 */
#define GRAY_CMD_NORMALIZE      0xCF    /* 写 0xFF = 8 路全开 */

/* !! 待确认 !! 连续读取 8 路归一化模拟值的命令字，手册未拿到，
   暂按 0xB0 实现：从 0xB0 起连续读 8 字节，对应 CH0~CH7 */
#define GRAY_CMD_READ_ANALOG    0xB0

/* ping 同步的最长等待时间 */
#define GRAY_PING_TIMEOUT_MS    3000

/* ================= 接口 ================= */

/* ping 同步 -> 通道使能 -> 归一化使能。
   HAL_TIMEOUT 表示等不到 0x66(接线/供电/地址问题) */
HAL_StatusTypeDef Gray_Init(void);

/* 一次读回 8 路数据，CH0 在下标 0 */
HAL_StatusTypeDef Gray_ReadAll(uint8_t *values);

#endif /* __GRAYSCALE_H */
