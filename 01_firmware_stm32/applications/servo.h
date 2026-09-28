#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

#define SERVO_MIN_PULSE_US      500
#define SERVO_MAX_PULSE_US      2500
#define SERVO_MAX_ANGLE_DEG     180.0f

#define SERVO_SAFE_MIN_US       1050
#define SERVO_SAFE_MAX_US       2400

#define SERVO_LEVEL_US          1588

#define SERVO_STEP_US           1

#define SERVO_DEMO_HOLD_MS      1000
#define SERVO_DEMO_SPEED_UPS    600.0f

void Servo_Init(void);

void Servo_SetAngle(float deg);
float Servo_GetAngle(void);

void Servo_SetPulseUs(uint16_t us);
uint16_t Servo_GetPulseUs(void);

uint8_t Servo_IsAtLimit(void);

void Servo_StepUs(int16_t delta_us);

void Servo_DemoInit(void);
void Servo_DemoUpdate(void);

#endif /* __SERVO_H */
