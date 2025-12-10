/**
 * @file pid.h
 * @brief PID control and motor control API for robotic hand fingers.
 *
 * This header defines:
 *  - The PID controller data structure
 *  - Public functions for PID initialization, reset, and stepping
 *  - Motor control helpers for homing and position control
 *
 * Implementation details are provided in pid.cpp.
 */

#ifndef PID_H
#define PID_H

#include "Constants.h"

/**
 * @brief PID controller state structure.
 *
 * Stores controller gains, internal state, and output limits.
 */
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

void PID_Reset(PID_t *pid);
void InitPIDs();

/** @name Global PID controller instances
 *  @brief PID controllers for each finger and palm motor.
 *  @{
 */
extern PID_t pid_thumb;
extern PID_t pid_index;
extern PID_t pid_middle;
extern PID_t pid_ring;
extern PID_t pid_pinkie;
extern PID_t pid_palm;
/** @} */

#endif