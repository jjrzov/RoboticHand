#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>
#include <Adafruit_ADS7830.h>
#include <esp_now.h>
#include <WiFi.h>

// External ADC
const uint8_t EXT_ADC_THUMB = 0;

// Homing
const float HOMING_KP = 0.5f;
const uint32_t HOMING_PWM_MIN = 150;
const uint32_t HOMING_PWM_MAX = 800;
const uint32_t HOMING_THRESHOLD = 10;

// Motor GPIOs
const int MOTOR_PWM_FREQ = 15000;     // 15kHz PWM Signal
const int MOTOR_PWM_RESOLUTION = 10;  // 10 bit PWM Resolution

const int THUMB_PWM_CHANNEL = 0;      // LED PWM Channel 0
const int THUMB_PWM_PIN = 5;  // GPIO 25
const int THUMB_PH_PIN = 2;   // GPIO 33

// Flex Sensor Range
const float F_MIN = 200.0f;   // flex at fully open
const float F_MAX = 600.0f;   // flex at fully closed

// Encoder Range
const float E_MIN = 0.0f;     // encoder at fully open
const float E_MAX = 255.0f;   // encoder at fully closed


struct Hand_Data {
  uint16_t thumb_data;
};

class Hand_Queues {
    public:
        QueueHandle_t thumb_queue;

        Hand_Queues() {
          thumb_queue = xQueueCreate(1, sizeof(uint16_t));
        }
};

typedef enum {
    HOMING_STATE,
    POSITION_CONTROL_STATE,
    ERROR_STATE
} HandState;

#endif