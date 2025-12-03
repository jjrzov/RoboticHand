#include <Arduino.h>
#include <Adafruit_ADS7830.h>
#include <esp_now.h>
#include <WiFi.h>

// External ADC
const uint32_t EXT_ADC_PIN = GPIO_NUM_36;
const uint8_t EXT_ADC_THUMB = 0;
// const uint8_t EXT_ADC_INDEX = 1;
// const uint8_t EXT_ADC_MIDDLE = 2;
// const uint8_t EXT_ADC_RING = 3;
// const uint8_t EXT_ADC_PINKIE = 4;
// const uint8_t EXT_ADC_PALM = 5;

// ESP-NOW
esp_now_peer_info_t peerInfo;

// Homing
const float HOMING_KP = 0.5f;
const uint32_t HOMING_PWM_MIN = 150;
const uint32_t HOMING_PWM_MAX = 800;
const uint32_t HOMING_THRESHOLD = 5;

// Motor GPIOs
const int MOTOR_PWM_FREQ = 15000;     // 20kHz PWM Signal
const int MOTOR_PWM_RESOLUTION = 10;  // 10 bit PWM Resolution

const int THUMB_PWM_CHANNEL = 0;      // LED PWM Channel 0
const int THUMB_PWM_PIN = 25;  // GPIO 25
const int THUMB_PH_PIN = 33;   // GPIO 33

// const int INDEX_PWM_CHANNEL = 1;      // LED PWM Channel 1
// const int INDEX_PWM_PIN = 25;  // GPIO 25
// const int INDEX_PH_PIN = 33;   // GPIO 33

// const int MIDDLE_PWM_CHANNEL = 2;      // LED PWM Channel 1
// const int MIDDLE_PWM_PIN = 25;  // GPIO 25
// const int MIDDLE_PH_PIN = 33;   // GPIO 33

// const int RING_PWM_CHANNEL = 3;      // LED PWM Channel 1
// const int RING_PWM_PIN = 25;  // GPIO 25
// const int RING_PH_PIN = 33;   // GPIO 33

// const int PINKIE_PWM_CHANNEL = 4;      // LED PWM Channel 1
// const int PINKIE_PWM_PIN = 25;  // GPIO 25
// const int PINKIE_PH_PIN = 33;   // GPIO 33

// const int PALM_PWM_CHANNEL = 5;      // LED PWM Channel 1
// const int PALM_PWM_PIN = 25;  // GPIO 25
// const int PALM_PH_PIN = 33;   // GPIO 33

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float out_min;
  float out_max;
} PID_t;

PID_t pid_thumb;
// , pid_index, pid_middle, pid_ring, pid_pinkie, pid_palm;

struct Hand_Data {
  uint32_t thumb_data;
  uint32_t index_data;
  uint32_t middle_data;
  uint32_t ring_data;
  uint32_t pinkie_data;
  uint32_t palm_data;
};

class Hand_Queues {
    public:
        QueueHandle_t thumb_queue;
        // QueueHandle_t index_queue;
        // QueueHandle_t middle_queue;
        // QueueHandle_t ring_queue;
        // QueueHandle_t pinkie_queue;
        // QueueHandle_t palm_queue;

        Hand_Queues() {
          thumb_queue = xQueueCreate(1, sizeof(uint32_t));
          // index_queue = xQueueCreate(1, sizeof(uint32_t));
          // middle_queue = xQueueCreate(1, sizeof(uint32_t));
          // ring_queue = xQueueCreate(1, sizeof(uint32_t));
          // pinkie_queue = xQueueCreate(1, sizeof(uint32_t));
          // palm_queue = xQueueCreate(1, sizeof(uint32_t));
        }
};

Hand_Queues glove = Hand_Queues();

// Receiver Queue
QueueHandle_t packet_queue = xQueueCreate(1, sizeof(Hand_Data));

// External ADC Instance
Adafruit_ADS7830 ad7830;

typedef enum {
    HOMING_STATE,
    POSITION_CONTROL_STATE,
    ERROR_STATE
} HandState;

void ReceiverTask(void *p_params) {
  Hand_Data data;
  
  while(1) {
    if (xQueueReceive(packet_queue, &data, portMAX_DELAY) == pdTRUE) {
      // TODO Go from flex reading to an encoder value then add to queue

      xQueueSend(glove.thumb_queue, &data.thumb_data, 0);
      // xQueueSend(glove.index_queue, &data.index_data, 0);
      // xQueueSend(glove.middle_queue, &data.middle_data, 0);
      // xQueueSend(glove.ring_queue, &data.ring_data, 0);
      // xQueueSend(glove.pinkie_queue, &data.pinkie_data, 0);
      // xQueueSend(glove.palm_queue, &data.palm_data, 0);
    }
  }
}

void loop() {
  vTaskDelay(1000000);
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {  
  Hand_Data data;
  memcpy(&data, incomingData, sizeof(data));
  xQueueSend(packet_queue, &data, 0);
  // Serial.println("Packet Received\n");
  return;
}

bool InitComms() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to get recv packet info
  esp_now_register_recv_cb(OnDataRecv);

  return true;
}

bool InitADC() {
  // Initialize external ADC
  if (!ad7830.begin()) {
    Serial.println("Failed to initialize ADS7830");
    return false;
  }
  return true;
}

void InitMotors() {
  // Initialize pins to control motors

  // Thumb
  ledcSetup(THUMB_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(THUMB_PWM_PIN, THUMB_PWM_CHANNEL);
  pinMode(THUMB_PH_PIN, OUTPUT);

  // // Index
  // ledcSetup(INDEX_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  // ledcAttachPin(INDEX_PWM_PIN, INDEX_PWM_CHANNEL);
  // pinMode(INDEX_PH_PIN, OUTPUT);

  // // Middle
  // ledcSetup(MIDDLE_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  // ledcAttachPin(MIDDLE_PWM_PIN, MIDDLE_PWM_CHANNEL);
  // pinMode(MIDDLE_PH_PIN, OUTPUT);

  // // Ring
  // ledcSetup(RING_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  // ledcAttachPin(RING_PWM_PIN, RING_PWM_CHANNEL);
  // pinMode(RING_PH_PIN, OUTPUT);

  // // Pinkie
  // ledcSetup(PINKIE_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  // ledcAttachPin(PINKIE_PWM_PIN, PINKIE_PWM_CHANNEL);
  // pinMode(PINKIE_PH_PIN, OUTPUT);

  // // Palm
  // ledcSetup(PALM_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  // ledcAttachPin(PALM_PWM_PIN, PALM_PWM_CHANNEL);
  // pinMode(PALM_PH_PIN, OUTPUT);
}

uint32_t ReadEncoder(uint8_t ext_adc_num) {
  // Read encoder value from rotary potentiometer
  return ad7830.readADCsingle(ext_adc_num);
}

void SetPhase(uint32_t error, uint32_t motor_phase_pin) {
  // Find which direction the motor should be moved to get to desired encoder value
  // Set phase pin to match calculated direction
  if (error > 0) {
    digitalWrite(motor_phase_pin, HIGH);  // Go CCW
  } else {
    digitalWrite(motor_phase_pin, LOW);  // Go CW
  } 
  return;
}

void StopAllMotors() {
  // Turn off all motors by setting pwm to zero
  ledcWrite(THUMB_PWM_CHANNEL, 0);
  // ledcWrite(INDEX_PWM_CHANNEL, 0);
  // ledcWrite(MIDDLE_PWM_CHANNEL, 0);
  // ledcWrite(RING_PWM_CHANNEL, 0);
  // ledcWrite(PINKIE_PWM_CHANNEL, 0);
  // ledcWrite(PALM_PWM_CHANNEL, 0);
  return;
}

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
  PID_Init(&pid_thumb,  1.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  // PID_Init(&pid_index,  1.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  // PID_Init(&pid_middle, 1.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  // PID_Init(&pid_ring,   1.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  // PID_Init(&pid_pinkie, 1.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  // PID_Init(&pid_palm,   1.0f, 0.0f, 0.0f, -1023.0f, 1023.0f);
  return;
}

void PID_Reset(PID_t *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  return;
}

float PID_Step(PID_t *pid, float error, float dt) {
  pid->integral += error * dt;

  float derivative = (error - pid->prev_error) / dt;
  float out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

  if (out > pid->out_max) out = pid->out_max;
  if (out < pid->out_min) out = pid->out_min;

  pid->prev_error = error;
  return out;  // signed: negative = one direction, positive = the other
}

void ResetAllPIDs() {
  PID_Reset(&pid_thumb);
  // PID_Reset(&pid_index);
  // PID_Reset(&pid_middle);
  // PID_Reset(&pid_ring);
  // PID_Reset(&pid_pinkie);
  // PID_Reset(&pid_palm);
  return;
}

bool HomeFinger(uint32_t enc, uint32_t home_target,
                int pwm_channel, int phase_pin)
{
  int32_t err = (int32_t)home_target - (int32_t)enc;
  int32_t mag = abs(err);

  if (mag <= HOMING_THRESHOLD) {
    // Close enough: stop this motor
    ledcWrite(pwm_channel, 0);
    return true;
  }

  int dir = (err > 0) ? HIGH : LOW;
  digitalWrite(phase_pin, dir);

  int pwm = (int)(HOMING_KP * mag);
  if (pwm > HOMING_PWM_MAX) pwm = HOMING_PWM_MAX;
  if (pwm < HOMING_PWM_MIN) pwm = HOMING_PWM_MIN;

  ledcWrite(pwm_channel, pwm);
  return false;
}

void PositionControlFinger(uint32_t enc, uint32_t target,
                           PID_t *pid, int pwm_channel, int phase_pin,
                           float dt)
{
  float error = (float)target - (float)enc;
  float u = PID_Step(pid, error, dt);  // signed

  int dir = (u >= 0.0f) ? HIGH : LOW;
  uint32_t pwm = (uint32_t)fabs(u);
  if (pwm > 1023) pwm = 1023;         // 10-bit limit

  digitalWrite(phase_pin, dir);
  ledcWrite(pwm_channel, pwm);
  return;
}

void ControllerTask(void *p_params) {
  HandState state = HOMING_STATE;

  // home encoder counts (fill these with your real home values)
  Hand_Data home = { 100, 200, 300, 400, 500, 600 };

  // current target positions (for POSITION_CONTROL)
  Hand_Data targets = home;  // start at home

  InitPIDs();
  ResetAllPIDs();

  const TickType_t period = pdMS_TO_TICKS(5);  // 5 ms → 200 Hz
  TickType_t last_wake = xTaskGetTickCount();
  const float dt = 0.005f;                    // 5 ms in seconds

  for (;;) {
    // 1) Read actual encoder values (from your ADS7830)
    uint32_t enc_thumb  = ReadEncoder(EXT_ADC_THUMB);
    // uint32_t enc_index  = ReadEncoder(EXT_ADC_INDEX);
    // uint32_t enc_middle = ReadEncoder(EXT_ADC_MIDDLE);
    // uint32_t enc_ring   = ReadEncoder(EXT_ADC_RING);
    // uint32_t enc_pinkie = ReadEncoder(EXT_ADC_PINKIE);
    // uint32_t enc_palm   = ReadEncoder(EXT_ADC_PALM);

    switch (state) {
      case HOMING_STATE: {
        bool thumb_ok  = HomeFinger(enc_thumb,  home.thumb_data,
                                    THUMB_PWM_CHANNEL,  THUMB_PH_PIN);
        // bool index_ok  = HomeFinger(enc_index,  home.index_data,
        //                             INDEX_PWM_CHANNEL,  INDEX_PH_PIN);
        // bool middle_ok = HomeFinger(enc_middle, home.middle_data,
        //                             MIDDLE_PWM_CHANNEL, MIDDLE_PH_PIN);
        // bool ring_ok   = HomeFinger(enc_ring,   home.ring_data,
        //                             RING_PWM_CHANNEL,   RING_PH_PIN);
        // bool pinkie_ok = HomeFinger(enc_pinkie, home.pinkie_data,
        //                             PINKIE_PWM_CHANNEL, PINKIE_PH_PIN);
        // bool palm_ok   = HomeFinger(enc_palm,   home.palm_data,
        //                             PALM_PWM_CHANNEL,   PALM_PH_PIN);

        bool all_homed = thumb_ok;
        
        // && index_ok && middle_ok &&
        //                  ring_ok && pinkie_ok && palm_ok;

        if (all_homed) {
          StopAllMotors();
          ResetAllPIDs();               // clear integrators before PID mode
          state = POSITION_CONTROL_STATE;
        }
        break;
      }

      case POSITION_CONTROL_STATE: {
        // 2) Get latest desired targets from ESP-NOW queues (non-blocking)
        uint32_t val;
        if (xQueueReceive(glove.thumb_queue,  &val, 0) == pdTRUE) targets.thumb_data  = val;
        // if (xQueueReceive(glove.index_queue,  &val, 0) == pdTRUE) targets.index_data  = val;
        // if (xQueueReceive(glove.middle_queue, &val, 0) == pdTRUE) targets.middle_data = val;
        // if (xQueueReceive(glove.ring_queue,   &val, 0) == pdTRUE) targets.ring_data   = val;
        // if (xQueueReceive(glove.pinkie_queue, &val, 0) == pdTRUE) targets.pinkie_data = val;
        // if (xQueueReceive(glove.palm_queue,   &val, 0) == pdTRUE) targets.palm_data   = val;

        // 3) Run PID for each finger
        PositionControlFinger(enc_thumb,  targets.thumb_data,
                              &pid_thumb,  THUMB_PWM_CHANNEL,  THUMB_PH_PIN,  dt);

        // PositionControlFinger(enc_index,  targets.index_data,
        //                       &pid_index,  INDEX_PWM_CHANNEL,  INDEX_PH_PIN,  dt);

        // PositionControlFinger(enc_middle, targets.middle_data,
        //                       &pid_middle, MIDDLE_PWM_CHANNEL, MIDDLE_PH_PIN, dt);

        // PositionControlFinger(enc_ring,   targets.ring_data,
        //                       &pid_ring,   RING_PWM_CHANNEL,   RING_PH_PIN,   dt);

        // PositionControlFinger(enc_pinkie, targets.pinkie_data,
        //                       &pid_pinkie, PINKIE_PWM_CHANNEL, PINKIE_PH_PIN, dt);

        // PositionControlFinger(enc_palm,   targets.palm_data,
        //                       &pid_palm,   PALM_PWM_CHANNEL,   PALM_PH_PIN,   dt);
        break;
      }

      case ERROR_STATE:
      default:
        StopAllMotors();
        // could add some error handling / LED blink here
        break;
    }

    vTaskDelayUntil(&last_wake, period);
  }
}

void setup() {
  Serial.begin(115200);
  BaseType_t retVal;

  InitComms();
  InitADC();
  InitMotors();

  retVal = xTaskCreate(ControllerTask, "Controller Task", 1024, NULL, 1, NULL);
  if (retVal != pdPASS) {
    Serial.println("Failed to create Bootup Task\n");
  }
  
  retVal = xTaskCreate(ReceiverTask, "Receiver Task", 1024, NULL, 2, NULL);
  if (retVal != pdPASS) {
    Serial.println("Failed to create Receiver Task\n");
  }
}