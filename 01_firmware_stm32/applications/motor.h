#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* ================= TB6612FNG 双路直流电机驱动 =================
   PWMA 左调速 PC6  TIM8_CH1     AIN1/AIN2 左方向 PF0/PF1
   PWMB 右调速 PC7  TIM8_CH2     BIN1/BIN2 右方向 PF2/PF3
   STBY 芯片使能 PF4(低电平整片待机)

   真值表(单路)：
     IN1=1 IN2=0 正转    IN1=0 IN2=1 反转
     IN1=1 IN2=1 短路刹车 IN1=0 IN2=0 停止(滑行) */

/* PWM 载波频率。TB6612 手册上限 100kHz，取 20kHz 在人耳之上 */
#define MOTOR_PWM_FREQ_HZ       20000u

/* 速度量程：Motor_SetSpeed() 的入参范围 */
#define MOTOR_SPEED_MAX         1000

/* 安装方向修正：电机装成镜像时把对应项改为 1 即可反向，无需改接线 */
#define MOTOR_LEFT_REVERSED     0
#define MOTOR_RIGHT_REVERSED    0

/* ================= 类型 ================= */

typedef enum
{
  MOTOR_LEFT  = 0,
  MOTOR_RIGHT = 1,
  MOTOR_NUM
} Motor_ID;

/* 速度为 0 时的默认行为 */
typedef enum
{
  MOTOR_STOP_COAST = 0,   /* 断电滑行 */
  MOTOR_STOP_BRAKE        /* 短路刹车，停得更快 */
} Motor_StopMode;

/* ================= 接口 ================= */

/* 初始化 TIM8 PWM 与方向引脚，完成后处于待机(STBY=0)、速度 0 */
void Motor_Init(void);

/* STBY 控制：Disable 后输出高阻，Enable 恢复上一次设定的速度 */
void Motor_Enable(void);
void Motor_Disable(void);

/* speed 正为正转、负为反转，超量程自动限幅 */
void Motor_SetSpeed(Motor_ID id, int16_t speed);

void Motor_SetSpeeds(int16_t left, int16_t right);

/* 读回当前设定速度(已限幅、未做方向取反) */
int16_t Motor_GetSpeed(Motor_ID id);

/* 速度归零时用哪种方式停车，默认 MOTOR_STOP_COAST */
void Motor_SetStopMode(Motor_StopMode mode);

/* 立即停止，不受 Motor_SetStopMode() 影响 */
void Motor_Brake(Motor_ID id);
void Motor_Coast(Motor_ID id);
void Motor_BrakeAll(void);
void Motor_CoastAll(void);

#endif /* __MOTOR_H */
