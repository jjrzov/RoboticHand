/**
 * @file Constants.h
 * @brief Hardware configuration, calibration values, and shared data types.
 *
 * This file centralizes all system-wide constants for:
 *  - External ADC channel mappings
 *  - Motor control parameters
 *  - Homing thresholds and gains
 *  - Flex sensor and encoder calibration
 *  - Shared data structures and state enums
 *
 * All values here are platform- and hardware-specific.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>
#include <Adafruit_ADS7830.h>
#include <esp_now.h>
#include <WiFi.h>

/** @name External ADC channel assignments
 *  @brief ADS7830 channel numbers for each finger encoder.
 *  @{
 */
const uint8_t EXT_ADC_THUMB = 0;
const uint8_t EXT_ADC_INDEX = 1;
const uint8_t EXT_ADC_MIDDLE = 4;
const uint8_t EXT_ADC_RING = 5;
const uint8_t EXT_ADC_PINKIE = 3;
const uint8_t EXT_ADC_PALM = 2;
/** @} */

/** @name Homing control parameters
 *  @brief Gains and thresholds used during finger homing.
 *  @{
 */
const float HOMING_KP = 0.5f;
const uint32_t HOMING_PWM_MIN = 150;
const uint32_t HOMING_PWM_MAX = 800;
const uint32_t HOMING_THRESHOLD = 20;
/** @} */

/** @name Motor PWM configuration
 *  @brief Global PWM parameters shared across all motors.
 *  @{
 */
const int MOTOR_PWM_FREQ = 15000;     // 15kHz PWM Signal
const int MOTOR_PWM_RESOLUTION = 10;  // 10 bit PWM Resolution
/** @} */


/** @name Motor pin and channel mappings
 *  @brief PWM channels, output pins, and phase pins for each motor.
 *  @{
 */
const int THUMB_PWM_CHANNEL = 0;      // LED PWM Channel 0
const int THUMB_PWM_PIN = 2;  // GPIO 5
const int THUMB_PH_PIN = 5;   // GPIO 2

const int INDEX_PWM_CHANNEL = 1;      // LED PWM Channel 1
const int INDEX_PWM_PIN = 4;  // GPIO 4
const int INDEX_PH_PIN = 16;   // GPIO 16

const int MIDDLE_PWM_CHANNEL = 2;      // LED PWM Channel 2
const int MIDDLE_PWM_PIN = 14;  // GPIO 14
const int MIDDLE_PH_PIN = 12;   // GPIO 12

const int RING_PWM_CHANNEL = 3;      // LED PWM Channel 3
const int RING_PWM_PIN = 25;  // GPIO 25
const int RING_PH_PIN = 26;   // GPIO 26

const int PINKIE_PWM_CHANNEL = 4;      // LED PWM Channel 4
const int PINKIE_PWM_PIN = 23;  // GPIO 23
const int PINKIE_PH_PIN = 19;   // GPIO 19

const int PALM_PWM_CHANNEL = 5;      // LED PWM Channel 5
const int PALM_PWM_PIN = 13;  // GPIO 13
const int PALM_PH_PIN = 27;   // GPIO 27
/** @} */


/** @name Flex sensor calibration
 *  @brief Expected flex sensor range corresponding to open/closed hand.
 *  @{
 */
const float F_MIN = 142.0f;   // flex at fully open
const float F_MAX = 1100.0f;   // flex at fully closed
/** @} */

/** @name Encoder calibration
 *  @brief Encoder range corresponding to open/closed hand.
 *  @{
 */
const float E_MIN = 15.0f;     // encoder at fully open
const float E_MAX = 240.0f;   // encoder at fully closed
/** @} */

/**
 * @brief Data packet containing per-finger values.
 *
 * Used for:
 *  - ESP-NOW communication
 *  - Queue-based inter-task data passing
 */
struct Hand_Data {
  uint16_t thumb_data;
  uint16_t index_data;
  uint16_t middle_data;
  uint16_t ring_data;
  uint16_t pinkie_data;
  uint16_t palm_data;
};

/**
 * @brief Container for per-finger FreeRTOS queues.
 *
 * Each queue holds the most recent target encoder value
 * for its corresponding motor.
 */
class Hand_Queues {
    public:
        QueueHandle_t thumb_queue;
        QueueHandle_t index_queue;
        QueueHandle_t middle_queue;
        QueueHandle_t ring_queue;
        QueueHandle_t pinkie_queue;
        QueueHandle_t palm_queue;

        /**
        * @brief Construct a new Hand_Queues object.
        *
        * Initializes all queues with a depth of one element.
        */
        Hand_Queues() {
          thumb_queue = xQueueCreate(1, sizeof(uint16_t));
          index_queue = xQueueCreate(1, sizeof(uint16_t));
          middle_queue = xQueueCreate(1, sizeof(uint16_t));
          ring_queue = xQueueCreate(1, sizeof(uint16_t));
          pinkie_queue = xQueueCreate(1, sizeof(uint16_t));
          palm_queue = xQueueCreate(1, sizeof(uint16_t));
        }
};


/**
 * @brief State machine states for hand controller.
 */
typedef enum {
    HOMING_STATE,
    POSITION_CONTROL_STATE
} HandState;

#endif