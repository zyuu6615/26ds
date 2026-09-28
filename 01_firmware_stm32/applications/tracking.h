#ifndef __TRACKING_H
#define __TRACKING_H

#include "main.h"

/* ================= 传感器几何 ================= */

/* 8 路探头从左到右的安装坐标(mm)，车体中线为 0、左负右正。
   实测间距 12mm，阵列总跨度 84mm */
#define TRACK_SENSOR_POSITIONS  { -42.0f, -30.0f, -18.0f, -6.0f, \
                                    6.0f,  18.0f,  30.0f,  42.0f }

/* 偏差低通。1.0 = 不滤波，建议不要低于 0.7 */
#define TRACK_OFFSET_LPF        0.8f

/* 赛道是白底黑线填 1；黑底白线填 0 */
#define TRACK_LINE_IS_BLACK     1

/* 单路权重低于此值视为噪声，该路不参与质心计算 */
#define TRACK_WEIGHT_NOISE_TH   50

/* 8 路权重总和低于此值判定为丢线 */
#define TRACK_LOST_TH           30

/* ---------------- 横线(起点/终点胶带)检测 ---------------- */

/* 单路权重超过此值算"这一路是黑的" */
#define TRACK_CROSS_WEIGHT_TH   150

/* 至少这么多路同时黑才判定为横线 */
#define TRACK_CROSS_MIN_CH      4

/* 判横线时额外要求偏差足够小(mm)，用来和急弯区分 */
#define TRACK_CROSS_MAX_OFFSET_MM   20.0f

/* ================= 运行参数 ================= */

/* 外环周期，须与 app.c 里调用 Track_Update() 的实际间隔一致 */
#define TRACK_PERIOD_MS         20
#define TRACK_PERIOD_S          (TRACK_PERIOD_MS / 1000.0f)

/* 直道基准速度(输出轴 rpm) */
#define TRACK_BASE_RPM          150.0f

/* 转向 PID */
#define TRACK_STEER_KP          2.0f
#define TRACK_STEER_KI          0.0f
#define TRACK_STEER_KD          0.25f

/* 差速上限(rpm)，限制转向环最多能让两轮差多少 */
#define TRACK_STEER_LIMIT_RPM   100.0f

/* 弯道减速：每 1mm 偏差降低多少 rpm */
#define TRACK_CURVE_SLOWDOWN    1.2f
#define TRACK_MIN_RPM           50.0f

/* 丢线时把偏差钉到这个值，让转向环给出最大修正 */
#define TRACK_LOST_OFFSET_MM    70.0f

/* 单轮目标转速上限 */
#define TRACK_MAX_RPM           250.0f

/* 连续丢线超过这个时间就停车 */
#define TRACK_LOST_STOP_MS      1000

/* ================= 接口 ================= */

/* 须在 Gray_Init() 之后调用 */
void Track_Init(void);

/* 跑一拍外环，须每 TRACK_PERIOD_MS 稳定调用一次 */
void Track_Update(void);

/* 取出给内环速度环用的左右目标转速 */
void Track_GetTargets(float *left_rpm, float *right_rpm);

/* 最近一次算出的横向偏差(mm)，正数表示线在车体右侧 */
float Track_GetOffset(void);

uint8_t Track_IsLost(void);

/* 当前是否压在横线上(起点/终点胶带) */
uint8_t Track_IsCrossLine(void);

/* 当前有多少路探头看到黑色 */
uint8_t Track_GetDarkCount(void);

const uint8_t *Track_GetRaw(void);

HAL_StatusTypeDef Track_GetStatus(void);

/* 改基准速度。设为 0 即停车但仍保持循迹计算 */
void Track_SetBaseSpeed(float rpm);

void Track_SetTunings(float kp, float ki, float kd);

/* 直线任务可以设 0 换取绝对匀速 */
void Track_SetCurveSlowdown(float rpm_per_mm);

/* 载球任务需要收紧差速上限换取平顺 */
void Track_SetSteerLimit(float rpm);

/* 立即停车并清空 PID 的积分与微分历史 */
void Track_Stop(void);

#endif /* __TRACKING_H */
