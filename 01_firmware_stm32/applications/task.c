#include "task.h"
#include "key.h"
#include "tracking.h"
#include "encoder.h"
#include "motor.h"
#include "ball.h"
#include "servo.h"

#include <math.h>

#define TASK_PI                 3.14159265f

/* 一圈的行驶距离(mm) = 轮子周长 */
#define TASK_WHEEL_CIRC_MM      (TASK_PI * TASK_WHEEL_DIAMETER_MM)

/* ================= 运行时状态 ================= */

static Task_ID    s_task      = TASK_2;      /* 开机默认停在任务二 */
static Task_State s_state     = TASK_STATE_IDLE;

static uint32_t s_start_tick   = 0;
static uint32_t s_elapsed_ms   = 0;          /* 完成/停止后定格 */
static int32_t  s_start_count  = 0;
static float    s_distance_m   = 0.0f;
static float    s_ramp_rpm     = 0.0f;
static uint32_t s_split_ms     = 0;          /* 到达评分点的用时，0 = 还没到 */
static uint8_t  s_lap_done     = 0;
static uint32_t s_lap_done_ms  = 0;

/* 任务三的分段状态 */
typedef enum
{
  TASK3_GO_PLUS = 0,      /* 从中心送到 +5cm */
  TASK3_GO_MINUS,         /* 折返送到 -5cm */
  TASK3_SETTLING          /* 已够到 -5cm，等它稳住 */
} Task3_Phase;

static Task3_Phase s_t3_phase    = TASK3_GO_PLUS;
static uint32_t    s_t3_settle_ms = 0;

/* 任务一：进入前球杆闭环是否开着，结束时原样还回去 */
static uint8_t s_t1_ball_en = 0;

/* ================= 工具函数 ================= */

static int32_t Task_AvgCount(void)
{
  int32_t left  = Encoder_GetCount(ENCODER_LEFT);
  int32_t right = Encoder_GetCount(ENCODER_RIGHT);

  return (left + right) / 2;
}

static void Task_UpdateOdometry(void)
{
  s_distance_m = (float)(Task_AvgCount() - s_start_count)
                 / (float)ENCODER_COUNTS_PER_REV
                 * TASK_WHEEL_CIRC_MM / 1000.0f;
}

/* 把基准速度按限定的加减速率往目标值靠，顺带把本拍施加的加速度
   告诉球杆控制器做前馈 */
static void Task_RampBase(float target_rpm, float accel, float decel)
{
  float applied = 0.0f;

  if (s_ramp_rpm < target_rpm)
  {
    s_ramp_rpm += accel * TRACK_PERIOD_S;
    if (s_ramp_rpm > target_rpm)
    {
      s_ramp_rpm = target_rpm;
    }
    applied = accel;
  }
  else if (s_ramp_rpm > target_rpm)
  {
    s_ramp_rpm -= decel * TRACK_PERIOD_S;
    if (s_ramp_rpm < target_rpm)
    {
      s_ramp_rpm = target_rpm;
    }
    applied = -decel;
  }

  Track_SetBaseSpeed(s_ramp_rpm);
  Ball_SetAccelFF(applied);
}

/* 把过弯的向心加速度前馈给球杆。a = v x omega，omega 正比于左右轮速差，
   两者都是已知的指令值。比例常数并进 BALL_FF_CURVE_GAIN */
static void Task_UpdateCurveFF(void)
{
  float left  = 0.0f;
  float right = 0.0f;

  Track_GetTargets(&left, &right);
  Ball_SetCurveFF(0.5f * (left + right) * (right - left));
}

/* ================= 启动与收尾 ================= */

static void Task_Start(void)
{
  s_start_tick  = HAL_GetTick();
  s_elapsed_ms  = 0;
  s_start_count = Task_AvgCount();
  s_distance_m  = 0.0f;
  s_ramp_rpm    = 0.0f;
  s_split_ms    = 0;
  s_lap_done    = 0;
  s_lap_done_ms = 0;
  s_state       = TASK_STATE_RUN;

  /* 清掉上一轮残留的转向积分与微分历史 */
  Track_Init();

  Ball_SetAccelFF(0.0f);

  if (s_task == TASK_1)
  {
    s_t1_ball_en = Ball_IsEnabled();
    Ball_Enable(0);

    Track_SetBaseSpeed(0.0f);
    Servo_DemoInit();
  }
  else if (s_task == TASK_3)
  {
    s_t3_phase     = TASK3_GO_PLUS;
    s_t3_settle_ms = HAL_GetTick();

    Track_SetBaseSpeed(0.0f);
    Ball_SetTarget(TASK3_PLUS_CM);
  }
  else if (s_task == TASK_4)
  {
    Track_SetTunings(TASK4_STEER_KP, TASK4_STEER_KI, TASK4_STEER_KD);
    Track_SetCurveSlowdown(TASK4_CURVE_SLOWDOWN);
    Track_SetBaseSpeed(0.0f);
  }
  else if ((s_task == TASK_5) || (s_task == TASK_6))
  {
    Track_SetTunings(TASK56_STEER_KP, TASK56_STEER_KI, TASK56_STEER_KD);
    Track_SetCurveSlowdown(TASK56_CURVE_SLOWDOWN);
    Track_SetSteerLimit(TASK56_STEER_LIMIT_RPM);
    Track_SetBaseSpeed(0.0f);
  }
}

static void Task_Finish(Task_State end_state)
{
  s_elapsed_ms = HAL_GetTick() - s_start_tick;
  s_state      = end_state;

  if (s_task == TASK_1)
  {
    Servo_SetPulseUs(SERVO_LEVEL_US);
    Ball_Enable(s_t1_ball_en);
  }

  Track_Stop();

  /* 真正把车拽停的是 app.c 的 App_Idle()，短路刹车低速时几乎不起作用 */
  Motor_BrakeAll();
}

/* ================= 各任务运行体 ================= */

static void Task1_Run(void)
{
  Servo_DemoUpdate();
}

/* 任务二：巡线一圈，回到 A 点横线处停车 */
static void Task2_Run(void)
{
  Task_UpdateOdometry();

  /* ---------- 终点前减速 ---------- */
#if TASK2_CREEP_ENABLE
  if (s_distance_m >= TASK2_CREEP_START_M)
  {
    Track_SetBaseSpeed(TASK2_CREEP_RPM);
  }
#endif

  /* ---------- 终点判定 ---------- */
  /* 起跑时车就压在 A 点横线上，必须先跑出一段才开始检测 */
  if ((s_distance_m >= TASK2_MIN_LAP_M) && Track_IsCrossLine())
  {
    Task_Finish(TASK_STATE_DONE);
    return;
  }

  /* ---------- 里程兜底 ---------- */
#if TASK2_DIST_STOP_ENABLE
  if (s_distance_m >= TASK2_DIST_STOP_M)
  {
    Task_Finish(TASK_STATE_DONE);
  }
#endif
}

/* 任务三：车静止，摆杆把球送到 +5cm、折返、再送到 -5cm 并稳住 */
static void Task3_Run(void)
{
  float err = fabsf(Ball_GetPosCm() - Ball_GetTarget());

  switch (s_t3_phase)
  {
    case TASK3_GO_PLUS:
      if (err <= TASK3_ARRIVE_CM)
      {
        s_t3_phase = TASK3_GO_MINUS;
        Ball_SetTarget(TASK3_MINUS_CM);
      }
      break;

    case TASK3_GO_MINUS:
      if (err <= TASK3_ARRIVE_CM)
      {
        s_t3_phase     = TASK3_SETTLING;
        s_t3_settle_ms = HAL_GetTick();

        /* 评分看的是跑完全程的用时，即够到 -5cm 的这一刻 */
        s_split_ms = s_elapsed_ms;
      }
      break;

    default:    /* TASK3_SETTLING */
      if (err > TASK3_ARRIVE_CM)
      {
        s_t3_settle_ms = HAL_GetTick();     /* 又跑出去了，重新计时 */
      }
      else if ((HAL_GetTick() - s_t3_settle_ms) >= TASK3_SETTLE_MS)
      {
        /* 结束但不关闭球杆闭环，规则要求稳定在该点附近 */
        Task_Finish(TASK_STATE_DONE);
        return;
      }
      break;
  }

  /* ---------- 时间兜底 ---------- */
  if (s_elapsed_ms >= TASK3_RUN_TIME_MS)
  {
    Task_Finish(TASK_STATE_DONE);
  }
}

/* 任务四：慢速巡线通过 B 位置，到时停车 */
static void Task4_Run(void)
{
  float target_rpm;

  Task_UpdateOdometry();

  /* ---------- 锁存 A->B 用时 ---------- */
  if ((s_split_ms == 0U) && (s_distance_m >= TASK4_B_DISTANCE_M))
  {
    s_split_ms = s_elapsed_ms;
  }

  /* 用里程而不是时间做判据，时间会随电池电量漂移 */
  target_rpm = (s_distance_m >= TASK4_STOP_DIST_M) ? 0.0f : TASK4_BASE_RPM;

  Task_RampBase(target_rpm, TASK4_ACCEL_RPM_PER_S, TASK4_DECEL_RPM_PER_S);
  Task_UpdateCurveFF();

  /* 斜坡走完才正式结束，避免最后再补一脚硬刹 */
  if ((target_rpm <= 0.0f) && (s_ramp_rpm <= 0.0f))
  {
    Task_Finish(TASK_STATE_DONE);
    return;
  }

  /* ---------- 时间兜底 ---------- */
  if (s_elapsed_ms >= TASK4_RUN_TIME_MS)
  {
    Task_Finish(TASK_STATE_DONE);
  }
}

/* 任务五/六：慢速匀速跑完整圈，通过 A 位置后平缓停车 */
static void TaskBallLap_Run(void)
{
  float target_rpm;

  Task_UpdateOdometry();

  /* ---------- 整圈完成判定 ---------- */
  if (!s_lap_done)
  {
    if (((s_distance_m >= TASK56_MIN_LAP_M) && Track_IsCrossLine()) ||
        (s_distance_m >= TASK56_DIST_STOP_M))
    {
      s_lap_done    = 1;
      s_lap_done_ms = HAL_GetTick();
      s_split_ms    = s_elapsed_ms;  /* 通过 A 的时刻，评分看这个数 */
    }
  }

  /* ---------- 速度斜坡 ---------- */
  /* 过 A 之后再匀速跑一段才开始减速，把减速移出评分时刻 */
  target_rpm = TASK56_BASE_RPM;
  if (s_lap_done && ((HAL_GetTick() - s_lap_done_ms) >= TASK56_COAST_MS))
  {
    target_rpm = 0.0f;
  }
  Task_RampBase(target_rpm, TASK56_ACCEL_RPM_PER_S, TASK56_DECEL_RPM_PER_S);
  Task_UpdateCurveFF();

  if (s_lap_done && (s_ramp_rpm <= 0.0f))
  {
    Task_Finish(TASK_STATE_DONE);
    return;
  }

  /* ---------- 时间兜底 ---------- */
  if (s_elapsed_ms >= TASK56_RUN_TIME_MS)
  {
    Task_Finish(TASK_STATE_DONE);
  }
}

/* ================= 调度 ================= */

void Task_Init(void)
{
  Key_Init();

  s_task       = TASK_2;
  s_state      = TASK_STATE_IDLE;
  s_elapsed_ms = 0;
  s_distance_m = 0.0f;
}

void Task_Update(void)
{
  /* ---------- KEY1：切换任务(运行中屏蔽) ---------- */
  if (Key_WasPressed(KEY1))
  {
    if (s_state != TASK_STATE_RUN)
    {
      s_task  = (Task_ID)((s_task + 1) % TASK_NUM);
      s_state = TASK_STATE_IDLE;
      s_elapsed_ms = 0;
      s_distance_m = 0.0f;

      /* 载球任务先把球送回中心 O 待命。任务六起点是任意指定位置，不归位 */
      if ((s_task == TASK_3) || (s_task == TASK_4) || (s_task == TASK_5))
      {
        Ball_SetTarget(TASK3_CENTER_CM);
      }
    }
  }

  /* ---------- KEY2：启动 / 中途停止 ---------- */
  if (Key_WasPressed(KEY2))
  {
    if (s_state == TASK_STATE_RUN)
    {
      Task_Finish(TASK_STATE_IDLE);       /* 手动叫停，计时定格 */
    }
    else
    {
      Task_Start();
    }
  }

  /* ---------- 运行中的任务体 ---------- */
  if (s_state != TASK_STATE_RUN)
  {
    return;
  }

  s_elapsed_ms = HAL_GetTick() - s_start_tick;

  switch (s_task)
  {
    case TASK_1:
      Task1_Run();
      break;

    case TASK_2:
      Task2_Run();
      break;

    case TASK_3:
      Task3_Run();
      break;

    case TASK_4:
      Task4_Run();
      break;

    /* 任务五与任务六的行车部分完全一致，共用同一套逻辑与参数 */
    case TASK_5:
    case TASK_6:
      TaskBallLap_Run();
      break;

    default:
      Task_UpdateOdometry();
      break;
  }
}

/* ================= 查询接口 ================= */

Task_ID Task_GetId(void)
{
  return s_task;
}

Task_State Task_GetState(void)
{
  return s_state;
}

uint8_t Task_IsRunning(void)
{
  return (s_state == TASK_STATE_RUN);
}

uint8_t Task_UsesVehicle(void)
{
  /* 静止任务：任务一只摆舵机，任务三全靠摆杆把球送到位 */
  return ((s_task != TASK_1) && (s_task != TASK_3));
}

uint32_t Task_GetElapsedMs(void)
{
  return s_elapsed_ms;
}

uint32_t Task_GetSplitMs(void)
{
  return s_split_ms;
}

float Task_GetDistanceM(void)
{
  return s_distance_m;
}
