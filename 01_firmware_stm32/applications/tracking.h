#ifndef __TRACKING_H
#define __TRACKING_H

#include "main.h"

/**
  ******************************************************************************
  * 循迹（巡线）—— 加权质心求横向偏差 + 转向 PID 输出差速
  *
  * 串级结构：
  *   外环(本模块, 20ms)  灰度8路 -> 加权质心 -> 偏差mm -> 转向PID -> 左右目标转速
  *   内环(app.c,   10ms)  目标转速 -> 速度PID -> PWM
  *
  * 内环必须比外环快，否则外环发出的指令还没执行完就被改了，整个系统会震荡。
  ******************************************************************************
  */

/* ================= 传感器几何 ================= */

/* 8 路探头从左到右的安装坐标，单位 mm，车体中线为 0、左负右正。
   实测探头间距 12mm，阵列总跨度 84mm。
   注意：只要间距是均匀的，这里填的绝对数值只相当于换算单位，
   改动它等于按比例缩放 Kp。 */
#define TRACK_SENSOR_POSITIONS  { -42.0f, -30.0f, -18.0f, -6.0f, \
                                    6.0f,  18.0f,  30.0f,  42.0f }

/* ---------------- 偏差低通 ----------------
   实测手动平移时质心是连续变化的(没有死区)，所以这里只做很轻的平滑，
   压一压远端探头的随机噪声即可。
   系数越小滤得越狠，但相位滞后越大 —— 循迹环对滞后很敏感，滤过头反而更容易摆。
   1.0 = 不滤波，建议不要低于 0.7。 */
#define TRACK_OFFSET_LPF        0.8f

/* 赛道是白底黑线填 1；黑底白线填 0 */
#define TRACK_LINE_IS_BLACK     1

/* 单路权重低于此值视为噪声，该路不参与质心计算。
   18mm 宽线 + 22mm 离地(光斑大而模糊)，黑度会摊开到远端探头上，把 ±30/±42
   这些位置也拉出 25~40 的弱权重。这些通道信噪比很差，可力臂又长，一点不对称
   乘上 42mm 就是好几毫米的质心偏移 —— 等效环路增益被凭空抬高，车就开始摆。
   阈值抬到 50 把它们挡在计算之外，比加滤波对症，而且不引入相位滞后。 */
#define TRACK_WEIGHT_NOISE_TH   50

/* 8 路权重总和低于此值判定为丢线 */
#define TRACK_LOST_TH           30

/* ---------------- 横线(起点/终点胶带)检测 ----------------
   横线是一道垂直于赛道的黑胶带，压上去时几乎所有探头同时变黑。
   阈值取得高(7/8)是为了避开急弯 —— 弯道时线斜穿阵列也会点亮好几路，
   但很难同时点亮 7 路。 */
#define TRACK_CROSS_WEIGHT_TH   150     /* 单路权重超过此值算"这一路是黑的" */

/* 至少这么多路同时黑才判定为横线。
   探头间距 12mm，位置 ±6/±18/±30/±42。A 点标记横向只有 50mm，居中时覆盖
   -25~+25，正好框住 ±6/±18 这 4 路(±30 在 25 之外)，稍偏才碰得到第 5 路。
   所以门槛必须是 4 —— 原来设 7 永远凑不满，这正是车到 A 点停不下来的原因。
   屏幕上的 DK 显示当前有几路黑，推着车过 A 点看那个峰值可以复核。 */
#define TRACK_CROSS_MIN_CH      4

/* 判横线时额外要求偏差足够小(mm)。
   门槛降到 4 之后，急弯时 18mm 的线斜穿阵列(横向投影可达 36mm)也可能点亮
   3~4 路，光看数量会误判成终点。但两者有个明显区别：压在 A 点横线上时车是
   对正的、质心接近 0；而急弯时线明显偏向一侧、质心很大。用这一条把它们分开。 */
#define TRACK_CROSS_MAX_OFFSET_MM   20.0f

/* ================= 运行参数 ================= */

/* 外环周期，必须与 app.c 里调用 Track_Update() 的实际间隔一致 */
#define TRACK_PERIOD_MS         20
#define TRACK_PERIOD_S          (TRACK_PERIOD_MS / 1000.0f)

/* 直道基准速度(输出轴 rpm)，左右轮在此基础上加减差速。
   6.14m 赛道：100rpm 跑出 20s(平均 87rpm，弯道减速吃掉约 13%)，
   要进 15s 需要平均 117rpm 以上，基准至少 134；取 150 留出余量 */
#define TRACK_BASE_RPM          150.0f

/* 转向 PID。整定方法见 tracking.c 末尾。
   探头坐标从 10mm 间距改成实测的 12mm 后，同样的位置偏差算出来的 mm 数
   会大 1.2 倍，Kp/Kd 都要按比例缩小才能保持原来的环路增益；
   宽线带来的跳变又需要再温柔一些，所以在此基础上又降了一档 */
#define TRACK_STEER_KP          2.0f
#define TRACK_STEER_KI          0.0f
#define TRACK_STEER_KD          0.25f

/* 差速上限(rpm)。限制住转向环最多能让两轮差多少，防止过弯直接原地打转。
   基准速度提高后这个值要同步加大，否则急弯的转向力度会不够 */
#define TRACK_STEER_LIMIT_RPM   100.0f

/* ---------------- 弯道减速 ----------------
   偏差越大说明弯越急，按比例压低基准速度：直道全速、弯道自动慢下来。
   这是高速循迹能过弯的关键 —— 任何转向力度都有物理极限，
   速度到了一定程度就只能靠减速来换转向半径。 */
#define TRACK_CURVE_SLOWDOWN    1.2f    /* 每 1mm 偏差降低多少 rpm */
#define TRACK_MIN_RPM           50.0f   /* 减速下限，别慢到失去惯性 */

/* 丢线时把偏差钉到这个值(阵列边缘 42mm 之外)，让转向环给出最大修正 */
#define TRACK_LOST_OFFSET_MM    70.0f

/* 单轮目标转速的上限，防止基准速度 + 差速超出电机能力。
   必须留够 TRACK_BASE_RPM + TRACK_STEER_LIMIT_RPM 的余量，
   否则急弯时外轮会被削顶，转向能力在最需要的时候反而被限住 */
#define TRACK_MAX_RPM           250.0f

/* 连续丢线超过这个时间就停车，避免车失控冲出赛道 */
#define TRACK_LOST_STOP_MS      1000

/* ================= 接口 ================= */

/* 初始化转向 PID。须在 Gray_Init() 之后调用 */
void Track_Init(void);

/* 跑一拍外环：读灰度 -> 算质心 -> 转向 PID -> 更新左右目标转速。
   必须每 TRACK_PERIOD_MS 稳定调用一次 */
void Track_Update(void);

/* 取出给内环速度环用的左右目标转速 */
void Track_GetTargets(float *left_rpm, float *right_rpm);

/* 最近一次算出的横向偏差(mm)，正数表示线在车体右侧 */
float Track_GetOffset(void);

/* 当前是否丢线 */
uint8_t Track_IsLost(void);

/* 当前是否压在横线上(起点/终点胶带) */
uint8_t Track_IsCrossLine(void);

/* 当前有多少路探头看到黑色，调横线阈值时用 */
uint8_t Track_GetDarkCount(void);

/* 最近一次读到的 8 路原始值，供显示/调试 */
const uint8_t *Track_GetRaw(void);

/* 最近一次灰度 I2C 传输的结果，供状态显示用 */
HAL_StatusTypeDef Track_GetStatus(void);

/* 改基准速度。设为 0 即停车但仍保持循迹计算 */
void Track_SetBaseSpeed(float rpm);

/* 在线改转向 PID 参数，方便用调试器整定，也用于不同任务切换软硬 */
void Track_SetTunings(float kp, float ki, float kd);

/* 在线改弯道减速系数。直线任务可以设 0 换取绝对匀速 */
void Track_SetCurveSlowdown(float rpm_per_mm);

/**
  * @brief  在线改差速上限(rpm)
  * @note   差速越大，两轮转速差越悬殊，车body的横摆角加速度也越大。
  *         载着钢球的任务需要收紧这个值换取平顺，代价是急弯转向力度变小。
  */
void Track_SetSteerLimit(float rpm);

/* 立即停车并清空转向 PID 的积分与微分历史 */
void Track_Stop(void);

#endif /* __TRACKING_H */
