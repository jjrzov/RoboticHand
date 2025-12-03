#ifndef PID_H
#define PID_H

#include "Constants.h"

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float out_min;
  float out_max;
} PID_t;

void PositionControlFinger(uint32_t enc, uint32_t target,
                           PID_t *pid, int pwm_channel, int phase_pin,
                           float dt);

void SetPhase(float u, uint32_t motor_phase_pin);
bool HomeFinger(uint32_t enc, uint32_t home_target,
                int pwm_channel, int phase_pin);

void ResetAllPIDs();
void PID_Reset(PID_t *pid);
void InitPIDs();

extern PID_t pid_thumb;

#endif