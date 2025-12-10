#include "pid.h"

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

void InitPIDs() {
  // output range: -1023..1023 for 10-bit PWM
  PID_Init(&pid_thumb,  2.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  PID_Init(&pid_index,  2.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  PID_Init(&pid_middle,  2.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  PID_Init(&pid_ring,  2.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  PID_Init(&pid_pinkie,  2.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  PID_Init(&pid_palm,  2.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  return;
}

void PID_Reset(PID_t *pid) {
  // Reset integral and prev terms of pid
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  return;
}

void ResetAllPIDs() {
  // Reset the prev and integral terms of all fingers
  PID_Reset(&pid_thumb);
  PID_Reset(&pid_index);
  PID_Reset(&pid_middle);
  PID_Reset(&pid_ring);
  PID_Reset(&pid_pinkie);
  PID_Reset(&pid_palm);
  return;
}

float PID_Step(PID_t *pid, float error, float dt) {
  pid->integral += error * dt;

  float derivative = (error - pid->prev_error) / dt;
  float out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

  if (out > pid->out_max) out = pid->out_max;
  if (out < pid->out_min) out = pid->out_min;

  pid->prev_error = error;
  return out;
}

void SetPhase(float u, uint32_t motor_phase_pin) {
  // Find which direction the motor should be moved to get to desired encoder value
  // Set phase pin to match calculated direction
  digitalWrite(motor_phase_pin,(u >= 0.0f) ? HIGH : LOW);
}

void PositionControlFinger(uint32_t enc, uint32_t target,
                           PID_t *pid, int pwm_channel, int phase_pin,
                           float dt)
{
  float error = (float)target - (float)enc;
  // Serial.print("PID Error: ");
  // Serial.println(error);
  float u = PID_Step(pid, error, dt);  // signed

  SetPhase(u, phase_pin); // Find direction to move in
  // Serial.print("UUUUUUUUUUUUUUUUUUUUUU: ");
  // Serial.println(u);

  float mag = fabsf(u);
  int pwm = (int)mag;

  // Serial.print("PWM: ");
  // Serial.println(pwm);

  // Add a minimum like homing
  const int POS_PWM_MIN = 300;   // tune this
  const int POS_PWM_MAX = 1023; // full scale

  if (pwm > POS_PWM_MAX) pwm = POS_PWM_MAX;
  if (pwm > 0 && pwm < POS_PWM_MIN) pwm = POS_PWM_MIN;  // prevent dead zone

  ledcWrite(pwm_channel, pwm);
  return;
}

bool HomeFinger(uint32_t enc, uint32_t home_target,
                int pwm_channel, int phase_pin)
{
  int32_t err = (int32_t)home_target - (int32_t)enc;
  int32_t mag = abs(err);
  // Serial.print("PID Error: ");
  // Serial.println(err);


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