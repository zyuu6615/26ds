#ifndef __TASK_H
#define __TASK_H

#include "main.h"

/* ================= 按键 =================
   KEY1  切换任务(仅在未运行时有效)
   KEY2  开始当前任务；运行中再按一次立即停止并刹车 */

/* ================= 车体参数 ================= */

#define TASK_WHEEL_DIAMETER_MM      67.0f

/* ================= 任务一参数 ================= */

/* 舵机安全行程内往复摆动，车全程不动。
   行程端点与扫描速度见 servo.h，这里不另设参数 */

/* ================= 任务二参数 ================= */

/* 小车置于 A 点，顺时针巡线一圈停回 A 点，≤20s、停车偏差 ≤2cm */

#define TASK_LAP_LENGTH_M          6.14f
#define TASK2_MIN_LAP_M             (TASK_LAP_LENGTH_M * 0.5f)

/* 终点减速 */
#define TASK2_CREEP_ENABLE          1
#define TASK2_CREEP_START_M         (TASK_LAP_LENGTH_M - 0.35f)
#define TASK2_CREEP_RPM             40.0f

/* 里程兜底停车 */
#define TASK2_DIST_STOP_ENABLE      1
#define TASK2_DIST_STOP_M           (TASK_LAP_LENGTH_M + 0.30f)

/* ================= 任务三参数 ================= */

/* 小车静止，摆杆把钢球从中心 O 送到 +5cm，折返再送到 -5cm 并稳定。
   ≤5s，±5cm 处最大误差 ≤1cm */

#define TASK3_CENTER_CM         12.5f
#define TASK3_PLUS_CM           (TASK3_CENTER_CM + 5.0f)
#define TASK3_MINUS_CM          (TASK3_CENTER_CM - 5.0f)

#define TASK3_ARRIVE_CM         0.6f
#define TASK3_SETTLE_MS         500
#define TASK3_RUN_TIME_MS       15000

/* ================= 任务四参数 ================= */

/* 小车置于 A 点、球在中心 O，顺时针巡线通过 B。
   AB 段 ≤8s，球稳定在中心附近，误差 ≤1cm */

#define TASK4_B_DISTANCE_M          1.6f
#define TASK4_BASE_RPM              90.0f
#define TASK4_STOP_DIST_M           (TASK4_B_DISTANCE_M + 0.5f)
#define TASK4_ACCEL_RPM_PER_S       60.0f
#define TASK4_DECEL_RPM_PER_S       120.0f
#define TASK4_RUN_TIME_MS           12000

/* 任务四专用转向参数：直线段，转向调软、关掉弯道减速 */
#define TASK4_STEER_KP              1.0f
#define TASK4_STEER_KI              0.0f
#define TASK4_STEER_KD              0.15f
#define TASK4_CURVE_SLOWDOWN        0.0f

/* ================= 任务五 / 任务六 共用参数 ================= */

/* 任务五：球在中心 O，顺时针跑完整圈并通过 A，≤30s，球稳定在中心，误差 ≤1cm。
   任务六：同上，但球置于摆杆上任意指定位置。
   两者对小车的要求一致，只有小球闭环的目标值不同 */

#define TASK56_BASE_RPM              80.0f
#define TASK56_ACCEL_RPM_PER_S       60.0f
#define TASK56_DECEL_RPM_PER_S       60.0f

/* 整圈有弯道，转向不能像任务四那么软 */
#define TASK56_STEER_KP              1.5f
#define TASK56_STEER_KI              0.0f
#define TASK56_STEER_KD              0.20f
#define TASK56_CURVE_SLOWDOWN        0.0f
#define TASK56_STEER_LIMIT_RPM       50.0f

#define TASK56_MIN_LAP_M             (TASK_LAP_LENGTH_M * 0.5f)
#define TASK56_DIST_STOP_M           (TASK_LAP_LENGTH_M + 0.15f)

/* 通过 A 之后继续匀速跑这么久再减速，把减速移出评分时刻 */
#define TASK56_COAST_MS              2000

#define TASK56_RUN_TIME_MS           35000

/* ================= 类型 ================= */

typedef enum
{
  TASK_1 = 0,
  TASK_2,
  TASK_3,
  TASK_4,
  TASK_5,
  TASK_6,
  TASK_NUM
} Task_ID;

typedef enum
{
  TASK_STATE_IDLE = 0,
  TASK_STATE_RUN,
  TASK_STATE_DONE
} Task_State;

/* ================= 接口 ================= */

void Task_Init(void);
void Task_Update(void);

Task_ID    Task_GetId(void);
Task_State Task_GetState(void);
uint8_t    Task_IsRunning(void);

/* 当前任务是否需要小车行驶。0 = 静止任务(如任务三) */
uint8_t Task_UsesVehicle(void);

/* 行驶总时间(ms)，完成或停止后定格 */
uint32_t Task_GetElapsedMs(void);

/* 到达评分点时锁存的用时(ms)，0 = 还没到。
   任务四 = 通过 B；任务五/六 = 通过 A */
uint32_t Task_GetSplitMs(void);

/* 本次任务已行驶的距离(m) */
float Task_GetDistanceM(void);

#endif /* __TASK_H */
