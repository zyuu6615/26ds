#ifndef __BALL_H
#define __BALL_H

#include "main.h"

#define BALL_TARGET_CM          12.5f

#define BALL_KP                 25.0f
#define BALL_KI                 0.0f
#define BALL_KD                 45.0f

#define BALL_OUTPUT_LIMIT_US    300.0f
#define BALL_INTEGRAL_LIMIT_US  40.0f
#define BALL_OUTPUT_SIGN        (+1.0f)

#define BALL_VEL_LPF            0.30f

#define BALL_FF_ENABLE          1
#define BALL_FF_US_PER_RPMS     2.0f

#define BALL_FF_CURVE_ENABLE    0
#define BALL_FF_CURVE_GAIN      0.04f

#define BALL_MIN_CONFIDENCE     0.50f
#define BALL_TIMEOUT_MS         200
#define BALL_DT_MIN_S           0.010f
#define BALL_DT_MAX_S           0.150f

void Ball_Init(void);
void Ball_Update(void);

void Ball_Enable(uint8_t on);
uint8_t Ball_IsEnabled(void);

void Ball_SetTarget(float cm);
float Ball_GetTarget(void);

void Ball_SetAccelFF(float rpm_per_s);
void Ball_SetCurveFF(float v_times_diff);

float   Ball_GetPosCm(void);
float   Ball_GetVelCmS(void);
float   Ball_GetOutputUs(void);
uint8_t Ball_IsTracking(void);

#endif /* __BALL_H */
