/**
 * @file pid.cpp
 * @brief PID control and motor control helpers for robotic hand fingers.
 *
 * This module provides:
 *  - Generic PID initialization, reset, and step functions
 *  - Homing control logic for DC motors with encoders
 *  - Position control using PID with PWM + phase output
 *
 * Designed for use with FreeRTOS-based embedded systems.
 */

#include "pid.h"


/**
 * @brief Initialize a PID controller structure.
 *
 * Sets controller gains, output limits, and clears internal state.
 *
 * @param pid      Pointer to PID controller structure.
 * @param kp       Proportional gain.
 * @param ki       Integral gain.
 * @param kd       Derivative gain.
 * @param out_min  Minimum allowable output.
 * @param out_max  Maximum allowable output.
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->out_min = out_min;
  pid->out_max = out_max;
  return;
}

/**
 * @brief Initialize all PID controllers used by the robotic hand.
 *
 * Sets gains and output limits for all fingers and palm.
 */
void InitPIDs() {
  // output range: -1023..1023 for 10-bit PWM
  PID_Init(&pid_thumb,  2.0f, 0.2f, 0.5f, -1023.0f, 1023.0f);
  PID_Init(&pid_index,  2.0f, 0.2f, 0.5f, -1023.0f, 1023.0f);
  PID_Init(&pid_middle,  2.0f, 0.2f, 0.5f, -1023.0f, 1023.0f);
  PID_Init(&pid_ring,  2.0f, 0.2f, 0.5f, -1023.0f, 1023.0f);
  PID_Init(&pid_pinkie,  2.0f, 0.2f, 0.5f, -1023.0f, 1023.0f);
  PID_Init(&pid_palm,  2.0f, 0.2f, 0.5f, -1023.0f, 1023.0f);
  return;
}

/**
 * @brief Reset a PID controller's internal state.
 *
 * Clears the integral accumulator and previous error term.
 *
 * @param pid Pointer to PID controller structure.
 */
void PID_Reset(PID_t *pid) {
  // Reset integral and prev terms of pid
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  return;
}

/**
 * @brief Perform one PID control step.
 *
 * Computes the control output using proportional, integral,
 * and derivative terms and applies output clamping.
 *
 * @param pid    Pointer to PID controller structure.
 * @param error  Current control error (setpoint - measurement).
 * @param dt     Time step in seconds since last update.
 *
 * @return Saturated control output value.
 */
float PID_Step(PID_t *pid, float error, float dt) {
  pid->integral += error * dt;

  float derivative = (error - pid->prev_error) / dt;
  float out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

  if (out > pid->out_max) out = pid->out_max;
  if (out < pid->out_min) out = pid->out_min;

  pid->prev_error = error;
  return out;
}

/**
 * @brief Set motor direction based on signed control effort.
 *
 * Uses the sign of the control signal to determine the motor
 * rotation direction via the phase pin.
 *
 * @param u               Signed control output.
 * @param motor_phase_pin GPIO pin controlling motor direction.
 */
void SetPhase(float u, uint32_t motor_phase_pin) {
  // Find which direction the motor should be moved to get to desired encoder value
  // Set phase pin to match calculated direction
  digitalWrite(motor_phase_pin,(u >= 0.0f) ? HIGH : LOW);
}

/**
 * @brief Perform PID-based position control for a single finger motor.
 *
 * Computes the control effort using PID, sets motor direction,
 * enforces a minimum PWM threshold, and applies the PWM output.
 *
 * @param enc          Current encoder reading.
 * @param target       Desired encoder target value.
 * @param pid          Pointer to PID controller for this finger.
 * @param pwm_channel  LEDC PWM channel controlling the motor.
 * @param phase_pin    GPIO pin controlling motor direction.
 * @param dt           Control loop timestep in seconds.
 */
void PositionControlFinger(uint32_t enc, uint32_t target,
                           PID_t *pid, int pwm_channel, int phase_pin,
                           float dt)
{
  float error = (float)target - (float)enc;
  float u = PID_Step(pid, error, dt);  // signed

  SetPhase(u, phase_pin); // Find direction to move in

  float mag = fabsf(u);
  int pwm = (int)mag;

  // Add a minimum like homing
  const int POS_PWM_MIN = 300;   // tune this
  const int POS_PWM_MAX = 1023; // full scale

  if (pwm > POS_PWM_MAX) pwm = POS_PWM_MAX;
  if (pwm > 0 && pwm < POS_PWM_MIN) pwm = POS_PWM_MIN;  // prevent dead zone

  ledcWrite(pwm_channel, pwm);
  return;
}


/**
 * @brief Home a single finger motor to a reference position.
 *
 * Drives the motor toward a known home encoder value using
 * a proportional controller until the error falls below
 * ::HOMING_THRESHOLD.
 *
 * @param enc          Current encoder reading.
 * @param home_target  Desired home encoder value.
 * @param pwm_channel  LEDC PWM channel controlling the motor.
 * @param phase_pin    GPIO pin controlling motor direction.
 *
 * @return true if the finger is homed, false otherwise.
 */
bool HomeFinger(uint32_t enc, uint32_t home_target,
                int pwm_channel, int phase_pin)
{
  int32_t err = (int32_t)home_target - (int32_t)enc;
  int32_t mag = abs(err);

  if (mag <= HOMING_THRESHOLD) {
    ledcWrite(pwm_channel, 0);  // Close enough stop this motor
    return true;
  }

  float u = HOMING_KP * (float)err;
  SetPhase(u, phase_pin);

  float mag_u = fabsf(u);
  int pwm = (int)mag_u; // Get magnitude for duty cycle

  // Clamp output
  if (pwm > HOMING_PWM_MAX) pwm = HOMING_PWM_MAX;
  if (pwm < HOMING_PWM_MIN) pwm = HOMING_PWM_MIN;

  ledcWrite(pwm_channel, pwm);
  return false;
}