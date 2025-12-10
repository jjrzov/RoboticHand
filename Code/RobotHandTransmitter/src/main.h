/**
 * @file main.h
 * @brief Glove transmitter interface for sensor acquisition and ESP-NOW communication.
 *
 * This header defines the public interface for the glove-side firmware, including:
 *  - Data structures for packaging sensor readings
 *  - A Hand class encapsulating FreeRTOS queues for each finger and the palm
 *  - Hardware configuration constants for ADC inputs
 *  - ESP-NOW communication parameters
 *  - FreeRTOS task prototypes for sensing and transmission
 */

#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "WiFi.h"
#include "esp_now.h"

/**
 * @brief Packet containing averaged flex sensor readings for each finger and the palm.
 *
 * This structure is sent over ESP-NOW from the glove to the receiver board.
 */
struct packet {
  uint16_t thumb_reading;
  uint16_t index_reading;
  uint16_t middle_reading;
  uint16_t ring_reading;
  uint16_t pinkie_reading;
  uint16_t palm_reading;
};


/**
 * @brief Class holding FreeRTOS queues for all glove fingers and palm.
 *
 * Each queue stores recent ADC readings (uint16_t) for the corresponding flex sensor.
 */
class Hand {
    public:
        QueueHandle_t thumb_queue;
        QueueHandle_t index_queue;
        QueueHandle_t middle_queue;
        QueueHandle_t ring_queue;
        QueueHandle_t pinkie_queue;
        QueueHandle_t palm_queue;

        /**
        * @brief Construct a new Hand object and create all sensor queues.
        *
        * Each queue has a length of 10 elements of type uint16_t.
        */
        Hand() {
          thumb_queue = xQueueCreate(10, sizeof(uint16_t));
          index_queue = xQueueCreate(10, sizeof(uint16_t));
          middle_queue = xQueueCreate(10, sizeof(uint16_t));
          ring_queue = xQueueCreate(10, sizeof(uint16_t));
          pinkie_queue = xQueueCreate(10, sizeof(uint16_t));
          palm_queue = xQueueCreate(10, sizeof(uint16_t));
        }
};

/** @name Flex sensor ADC input pins
 *  @brief GPIO pins connected to each flex sensor (ADC channels).
 *  @{
 */
const uint32_t THUMB_PIN = GPIO_NUM_36;
const uint32_t INDEX_PIN = GPIO_NUM_39;
const uint32_t MIDDLE_PIN = GPIO_NUM_34;
const uint32_t RING_PIN = GPIO_NUM_35;
const uint32_t PINKIE_PIN = GPIO_NUM_32;
const uint32_t PALM_PIN = GPIO_NUM_33;
/** @} */

/**
 * @brief ADC configuration: resolution in bits.
 * Used with ::analogReadResolution.
 */
const uint8_t ADC_RESOLUTION = 12;  // 12 bit resolution

/**
 * @brief ADC attenuation configuration.
 * Used with ::analogSetAttenuation to set the input range.
 */
const adc_attenuation_t ADC_ATTENUATION = ADC_11db;

// WROOM 32D DevKit MAC Address: 0xE8, 0x6B, 0xEA, 0xD3, 0x9B, 0x98
// Firebeetle MAC Address: 0x24, 0xDC, 0xC3, 0x82, 0x7F, 0xB4
const uint8_t BROADCAST_ADDRESS[] = {0x24, 0xDC, 0xC3, 0x82, 0x7F, 0xB4};  // Receiver MAC Address


void ReadSensorsTask(void *p_params);
void TransmitTask(void *p_params);
BaseType_t SendToQueue(const uint32_t pin, QueueHandle_t queue);
uint16_t ReadQueue(QueueHandle_t queue);
void PrintReadings(packet *hand);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
bool InitComms();


#endif