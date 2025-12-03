#include <Arduino.h>
#include "WiFi.h"
#include "esp_now.h"


const uint8_t ADC_RESOLUTION = 12;  // 12 bit resolution
const adc_attenuation_t ADC_ATTENUATION = ADC_11db;

// WROOM 32D DevKit MAC Address: 0xE8, 0x6B, 0xEA, 0xD3, 0x9B, 0x98
// Firebeetle MAC Address: 0x24, 0xDC, 0xC3, 0x82, 0x7F, 0xB4
const uint8_t BROADCAST_ADDRESS[] = {0x24, 0xDC, 0xC3, 0x82, 0x7F, 0xB4};  // Receiver MAC Address
esp_now_peer_info_t peerInfo;

const uint32_t THUMB_PIN = GPIO_NUM_36;
const uint32_t INDEX_PIN = GPIO_NUM_39;
const uint32_t MIDDLE_PIN = GPIO_NUM_34;
const uint32_t RING_PIN = GPIO_NUM_35;
const uint32_t PINKIE_PIN = GPIO_NUM_32;
const uint32_t PALM_PIN = GPIO_NUM_33;

struct packet {
  uint32_t thumb_reading;
  uint32_t index_reading;
  uint32_t middle_reading;
  uint32_t ring_reading;
  uint32_t pinkie_reading;
  uint32_t palm_reading;
};

class Hand {
    public:
        QueueHandle_t thumb;
        QueueHandle_t index;
        QueueHandle_t middle;
        QueueHandle_t ring;
        QueueHandle_t pinkie;
        QueueHandle_t palm;

        Hand() {
          thumb = xQueueCreate(10, sizeof(uint32_t));
          index = xQueueCreate(10, sizeof(uint32_t));
          middle = xQueueCreate(10, sizeof(uint32_t));
          ring = xQueueCreate(10, sizeof(uint32_t));
          pinkie = xQueueCreate(10, sizeof(uint32_t));
          palm = xQueueCreate(10, sizeof(uint32_t));
        }
};

Hand glove = Hand();

void ReadSensorsTask(void *p_params) {
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_ATTENUATION);

  BaseType_t retVal;
  uint32_t thumb_reading;
  uint32_t index_reading;
  uint32_t middle_reading;
  uint32_t ring_reading;
  uint32_t pinkie_reading;
  uint32_t palm_reading;

  while(1) {
    // Read latest sensor data and add to queue for each finger
    thumb_reading = analogReadMilliVolts(THUMB_PIN);
    retVal = xQueueSend(glove.thumb, &thumb_reading, 0);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to thumb queue\n");
    }

    index_reading = analogReadMilliVolts(INDEX_PIN);
    retVal = xQueueSend(glove.index, &index_reading, 0);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to index queue\n");
    }

    middle_reading = analogReadMilliVolts(MIDDLE_PIN);
    retVal = xQueueSend(glove.middle, &middle_reading, 0);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to middle queue\n");
    }

    ring_reading = analogReadMilliVolts(RING_PIN);
    retVal = xQueueSend(glove.ring, &ring_reading, 0);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to ring queue\n");
    }

    pinkie_reading = analogReadMilliVolts(PINKIE_PIN);
    retVal = xQueueSend(glove.pinkie, &pinkie_reading, 0);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to pinkie queue\n");
    }

    palm_reading = analogReadMilliVolts(PALM_PIN);
    retVal = xQueueSend(glove.palm, &palm_reading, 0);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to palm queue\n");
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void TransmitTask(void *p_params) {
  packet message;

  uint32_t reading;
  uint32_t sum = 0, count = 0; 

  while(1) {
    // Take the average for each finger and transmit a packet of averaged readings
    uint32_t sum = 0, count = 0; 
    while(xQueueReceive(glove.thumb, &reading, 0) == pdTRUE) {
      sum += reading;
      count++;
    }

    if (count > 0) {
      message.thumb_reading = sum / count;
    } else {
      message.thumb_reading = 0;  // or previous value, or some sentinel
    }
    Serial.println(message.thumb_reading);

    sum = 0;
    count = 0;
    while(xQueueReceive(glove.index, &reading, 0) == pdTRUE) {
      sum += reading;
      count++;
    }

    if (count > 0) {
      message.index_reading = sum / count;
    } else {
      message.index_reading = 0;  // or previous value, or some sentinel
    }

    sum = 0;
    count = 0;
    while(xQueueReceive(glove.middle, &reading, 0) == pdTRUE) {
      sum += reading;
      count++;
    }

    if (count > 0) {
      message.middle_reading = sum / count;
    } else {
      message.middle_reading = 0;  // or previous value, or some sentinel
    }

    sum = 0;
    count = 0;
    while(xQueueReceive(glove.ring, &reading, 0) == pdTRUE) {
      sum += reading;
      count++;
    }

    if (count > 0) {
      message.ring_reading = sum / count;
    } else {
      message.ring_reading = 0;  // or previous value, or some sentinel
    }

    sum = 0;
    count = 0;
    while(xQueueReceive(glove.pinkie, &reading, 0) == pdTRUE) {
      sum += reading;
      count++;
    }

    if (count > 0) {
      message.pinkie_reading = sum / count;
    } else {
      message.pinkie_reading = 0;  // or previous value, or some sentinel
    }

    sum = 0;
    count = 0;
    while(xQueueReceive(glove.palm, &reading, 0) == pdTRUE) {
      sum += reading;
      count++;
    }

    if (count > 0) {
      message.palm_reading = sum / count;
    } else {
      message.palm_reading = 0;  // or previous value, or some sentinel
    }

    esp_err_t result = esp_now_send(BROADCAST_ADDRESS, (uint8_t *) &message, sizeof(packet));
   
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);  // freq of 20ms

  }
}

void loop() {
  vTaskDelay(10000);
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

bool InitComms() {
  // ESP_NOW Setup
  WiFi.mode(WIFI_MODE_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to get the status 
  // of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, BROADCAST_ADDRESS, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

    // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  BaseType_t retVal;

  InitComms();

  retVal = xTaskCreate(ReadSensorsTask, "Read Sensors Task", 1024, NULL, 1, NULL);
  if (retVal != pdPASS) {
    Serial.println("Failed to create Read Sensor Task\n");
  }
  
  retVal = xTaskCreate(TransmitTask, "Transmit Task", 1024, NULL, 2, NULL);
  if (retVal != pdPASS) {
    Serial.println("Failed to create Transmit Task\n");
  }
}