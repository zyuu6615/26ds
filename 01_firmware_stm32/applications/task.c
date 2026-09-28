/**
  ******************************************************************************
  * @file           : task.c
  * @brief          : 任务调度 —— 按键切换/启停，任务二的一圈计时与终点停车
  ******************************************************************************
  */

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

/* ---------------- 运行时状态 ---------------- */
static Task_ID    s_task      = TASK_2;      /* 开机默认停在任务二 */
static Task_State s_state     = TASK_STATE_IDLE;

static uint32_t s_start_tick   = 0;
static uint32_t s_elapsed_ms   = 0;          /* 完成/停止后定格 */
static int32_t  s_start_count  = 0;          /* 启动时的编码器基准 */
static float    s_distance_m   = 0.0f;
static float    s_ramp_rpm     = 0.0f;       /* 载球任务的速度斜坡当前值 */
static uint32_t s_split_ms     = 0;          /* 到达评分点的用时，0 = 还没到 */
static uint8_t  s_lap_done     = 0;          /* 任务五/六：整圈已跑完 */
static uint32_t s_lap_done_ms  = 0;          /* 通过 A 的时刻，用来算滑行时长 */

/* 任务三的分段状态 */
typedef enum
{
  TASK3_GO_PLUS = 0,      /* 从中心送到 +5cm */
  TASK3_GO_MINUS,         /* 折返送到 -5cm */
  TASK3_SETTLING          /* 已够到 -5cm，等它稳住 */
} Task3_Phase;

static Task3_Phase s_t3_phase    = TASK3_GO_PLUS;
static uint32_t    s_t3_settle_ms = 0;       /* 进入容差圈的时刻 */

/* 任务一：进入前球杆闭环是否开着，结束时原样还回去。
   舵机脉宽只有一个出口，摆动演示跑起来时球杆闭环必须让位，
   否则两边每拍互相覆盖，舵机会抖成一团 */
static uint8_t s_t1_ball_en = 0;

/**
  * @brief  取左右轮的平均累计计数
  * @note   Encoder_GetCount() 与速度环的 Encoder_GetRPM() 各自维护标记，
  *         同时调用互不干扰
  */
static int32_t Task_AvgCount(void)
{
  int32_t left  = Encoder_GetCount(ENCODER_LEFT);
  int32_t right = Encoder_GetCount(ENCODER_RIGHT);

  return (left + right) / 2;
}

/**
  * @brief  刷新本次任务已行驶的距离
  */
static void Task_UpdateOdometry(void)
{
  s_distance_m = (float)(Task_AvgCount() - s_start_count)
                 / (float)ENCODER_COUNTS_PER_REV
                 * TASK_WHEEL_CIRC_MM / 1000.0f;
}

/**
  * @brief  把基准速度按限定的加减速率往目标值靠
  * @note   载球的任务(四、五、六)全靠它避免速度突变 —— 球感受到的是加速度，
  *         斜坡率就是加速度的上限。加速和减速分开设，两端都不能有冲击。
  *         顺带把本拍施加的加速度告诉球杆控制器做前馈 —— 球杆据此在球被
  *         惯性拽走【之前】就把杆预先倾好，而不是等球动了再补救
  */
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

/**
  * @brief  把过弯产生的向心加速度前馈给球杆
  * @note   稳态过弯的向心加速度 a = v x omega，而 omega 正比于左右轮速差。
  *         车速和轮速差都是我们自己下达的指令，因此完全已知 —— 和起步加速
  *         一个道理，应该提前把杆倾好，而不是等球被甩出去再修。
  *         比例常数(轮径、轮距、连杆传动比)全部并进 BALL_FF_CURVE_GAIN。
  */
static void Task_UpdateCurveFF(void)
{
  float left  = 0.0f;
  float right = 0.0f;

  Track_GetTargets(&left, &right);
  Ball_SetCurveFF(0.5f * (left + right) * (right - left));
}

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

  /* 清掉上一轮残留的转向积分与微分历史，并把基准速度恢复成 TRACK_BASE_RPM */
  Track_Init();

  /* 前馈归零。静止任务(任务三)不会调 Task_RampBase()，不清的话会一直
     用着上一个任务残留的加速度值，把杆莫名其妙地倾着 */
  Ball_SetAccelFF(0.0f);

  /* 任务一：车不动，只让舵机在标定出的机构安全行程内往复摆动，
     用来验收行程、方向和连杆装配。球杆闭环先摘掉 —— 它和摆动演示都在
     每拍写脉宽，同时开就是互相抢舵机 */
  if (s_task == TASK_1)
  {
    s_t1_ball_en = Ball_IsEnabled();
    Ball_Enable(0);

    Track_SetBaseSpeed(0.0f);
    Servo_DemoInit();
  }
  /* 任务三：车不动，摆杆先把球送到 +5cm */
  else if (s_task == TASK_3)
  {
    s_t3_phase     = TASK3_GO_PLUS;
    s_t3_settle_ms = HAL_GetTick();

    Track_SetBaseSpeed(0.0f);
    Ball_SetTarget(TASK3_PLUS_CM);
  }
  /* 任务四：直线段慢速跑，转向调软、关掉弯道减速，起步交给斜坡 */
  else if (s_task == TASK_4)
  {
    Track_SetTunings(TASK4_STEER_KP, TASK4_STEER_KI, TASK4_STEER_KD);
    Track_SetCurveSlowdown(TASK4_CURVE_SLOWDOWN);
    Track_SetBaseSpeed(0.0f);
  }
  /* 任务五/六：整圈慢速匀速跑，转向比任务四硬一点(要过弯)、差速上限收紧。
     两个任务的行车部分完全相同，差别只在小球的目标位置(中心 / 任意指定点)，
     那是小球闭环的 setpoint，与车怎么开无关 */
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

  /* 任务一收尾：摆动停在哪儿就是哪儿，先把杆放回水平点再把球杆闭环还回去。
     Ball_Enable(1) 内部也会回水平点，但闭环原本没开时(演示/标定模式)
     就没人管这根杆了，所以这一句不能省 */
  if (s_task == TASK_1)
  {
    Servo_SetPulseUs(SERVO_LEVEL_US);
    Ball_Enable(s_t1_ball_en);
  }

  Track_Stop();

  /* 先给一脚短路刹车。真正把车拽停的是 app.c 的 App_Idle() —— 它会用速度环
     主动反拖，因为短路刹车的制动力矩正比于转速，低速时几乎不起作用 */
  Motor_BrakeAll();
}

/**
  * @brief  任务一：舵机在 SERVO_SAFE_MIN_US ~ SERVO_SAFE_MAX_US 之间往复摆动
  * @note   车全程不动(见 Task_UsesVehicle())。摆动本身由 Servo_DemoUpdate()
  *         按 tick 匀速推进，与本函数被调用的频率无关，所以 20ms 一拍够用。
  *
  *         不设时间兜底：这是个用来看机构的演示，跑到按 KEY2 为止。
  */
static void Task1_Run(void)
{
  Servo_DemoUpdate();
}

/**
  * @brief  任务二：巡线一圈，回到 A 点横线处停车
  */
static void Task2_Run(void)
{
  Task_UpdateOdometry();

  /* ---------- 终点前减速(可选) ---------- */
#if TASK2_CREEP_ENABLE
  if (s_distance_m >= TASK2_CREEP_START_M)
  {
    Track_SetBaseSpeed(TASK2_CREEP_RPM);
  }
#endif

  /* ---------- 终点判定 ---------- */
  /* 起跑时车就压在 A 点横线上，所以必须先跑出去一段才开始检测，
     否则按下启动的瞬间就会判定为"已完成一圈" */
  if ((s_distance_m >= TASK2_MIN_LAP_M) && Track_IsCrossLine())
  {
    Task_Finish(TASK_STATE_DONE);
    return;
  }

  /* ---------- 里程兜底 ---------- */
  /* 横线没识别到时的最后一道保险，免得车一路跑第二圈 */
#if TASK2_DIST_STOP_ENABLE
  if (s_distance_m >= TASK2_DIST_STOP_M)
  {
    Task_Finish(TASK_STATE_DONE);
  }
#endif
}

/**
  * @brief  任务三：小车静止，摆杆把球送到 +5cm、折返、再送到 -5cm 并稳住
  * @note   到达 +5cm 时【不停留】直接换目标。5 秒要走完 15cm，等球在 +5cm
  *         彻底停稳会白白吃掉一两秒；而规则只要求"够到 ±5cm 附近"，
  *         一进容差圈就折返，控制器会立刻反向刹车，冲过的那点余量仍在 1cm 内。
  */
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

        /* 评分看的是"跑完全程"的用时，就是够到 -5cm 的这一刻 */
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
        /* 稳住了。任务结束但【不关闭球杆闭环】—— 规则要求"稳定在该点附近"，
           松手就散的话不算稳定 */
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

/**
  * @brief  任务四：慢速巡线通过 B 位置，到时停车
  * @note   钢球的稳定性取决于车体的【加速度】而不是速度 —— 匀速行驶时球是
  *         静止的，只有加减速和转向才会让它晃。所以这里做两件事：
  *         降低速度(让转向带来的横向加速度变小)、起步走斜坡(消掉启动冲击)。
  */
static void Task4_Run(void)
{
  float target_rpm;

  Task_UpdateOdometry();

  /* ---------- 锁存 A->B 用时 ---------- */
  /* 评分看的是这一段(要求 ≤8s)，不是行驶总时间 */
  if ((s_split_ms == 0U) && (s_distance_m >= TASK4_B_DISTANCE_M))
  {
    s_split_ms = s_elapsed_ms;
  }

  /* 过了 B 再走一小段就开始减速。用里程而不是时间做判据：
     时间会随电池电量变化而漂移，里程不会 */
  target_rpm = (s_distance_m >= TASK4_STOP_DIST_M) ? 0.0f : TASK4_BASE_RPM;

  Task_RampBase(target_rpm, TASK4_ACCEL_RPM_PER_S, TASK4_DECEL_RPM_PER_S);
  Task_UpdateCurveFF();

  /* 斜坡走完(速度已经归零)才正式结束，避免最后再补一脚硬刹 */
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

/**
  * @brief  任务五/六：慢速匀速跑完整圈，通过 A 位置后平缓停车
  * @note   和任务二跑同一圈，但一切设定都为"不晃球"服务：
  *         全程匀速(关掉弯道减速)、低速过弯(横向加速度按 v² 下降)、
  *         差速上限收紧(减小横摆角加速度)、两端走斜坡(消掉起停冲击)。
  *
  *         任务五要求球稳在摆杆中心、任务六要求稳在任意指定位置 —— 对车而言
  *         没有任何区别，差别只是小球闭环的目标值，所以两个任务共用本函数。
  */
static void TaskBallLap_Run(void)
{
  float target_rpm;

  Task_UpdateOdometry();

  /* ---------- 整圈完成判定 ---------- */
  /* 与任务二同一套判据：跑过半圈才开始看横线，避免起跑时压在 A 上就误判 */
  if (!s_lap_done)
  {
    if (((s_distance_m >= TASK56_MIN_LAP_M) && Track_IsCrossLine()) ||
        (s_distance_m >= TASK56_DIST_STOP_M))
    {
      s_lap_done    = 1;
      s_lap_done_ms = HAL_GetTick();
      s_split_ms    = s_elapsed_ms;  /* 通过 A 的时刻，评分看这个数(≤30s) */
    }
  }

  /* ---------- 速度斜坡 ---------- */
  /* 过 A 之后【先匀速再跑一段】才开始减速：判定点前后都保持匀速，
     减速带来的纵向加速度就不会在评委看球的那一刻把球拽走。
     滑行这几秒不计入评分时间(s_split_ms 已经在过 A 时锁存了)。

     减速本身也走斜坡而不是刹车 —— 任务五不要求停车精度，
     而硬刹那一下足够把球甩出 1cm */
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
  /* ---------- KEY1：切换任务 ---------- */
  /* 运行中屏蔽，避免跑着跑着被误触切走 */
  if (Key_WasPressed(KEY1))
  {
    if (s_state != TASK_STATE_RUN)
    {
      s_task  = (Task_ID)((s_task + 1) % TASK_NUM);
      s_state = TASK_STATE_IDLE;
      s_elapsed_ms = 0;
      s_distance_m = 0.0f;

      /* 切到载球任务时，先让摆杆把球送回中心 O 待命。
         任务三/四/五的规则都是"钢球置于中心点 O"再启动 —— 与其靠手摆，
         不如切过去就自动归位，按 KEY2 时球已经在起点上了。
         任务六不归位：它的起点本来就是任意指定位置。 */
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
  /* 静止任务：车原地不动。
     任务一只摆舵机(看机构行程)，任务三全靠摆杆把球送到位 */
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
