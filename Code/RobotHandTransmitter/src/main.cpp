/**
 * @file main.cpp
 * @brief ESP-NOW glove transmitter using FreeRTOS tasks to read flex sensors and send averaged values.
 */

#include "main.h"

/** @brief ESP-NOW peer configuration information. */
esp_now_peer_info_t peerInfo;

/** @brief Global Hand instance representing the glove and its sensor queues. */
Hand glove = Hand();

/**
 * @brief FreeRTOS task that continuously reads flex sensors and pushes values into per-finger queues.
 *
 * The task:
 *  - Configures ADC resolution and attenuation.
 *  - Periodically reads each sensor using ::SendToQueue.
 *  - Logs an error over Serial if queue send fails.
 *
 * @param p_params Pointer to task parameters (unused).
 */
void ReadSensorsTask(void *p_params) {
  // Set ADC settings
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_ATTENUATION);

  BaseType_t retVal;

  while(1) {
    // Read latest sensor data and add to queue for each finger
    retVal = SendToQueue(THUMB_PIN, glove.thumb_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to thumb queue\n");
    }

    retVal = SendToQueue(INDEX_PIN, glove.index_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to index queue\n");
    }

    retVal = SendToQueue(MIDDLE_PIN, glove.middle_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to middle queue\n");
    }

    retVal = SendToQueue(RING_PIN, glove.ring_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to ring queue\n");
    }

    retVal = SendToQueue(PINKIE_PIN, glove.pinkie_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to pinkie queue\n");
    }

    retVal = SendToQueue(PALM_PIN, glove.palm_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to palm queue\n");
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms freq
  }
}

/**
 * @brief FreeRTOS task that averages queue data for each finger and sends packets over ESP-NOW.
 *
 * The task:
 *  - Reads and averages queued sensor values with ::ReadQueue.
 *  - Packages readings into a @ref packet.
 *  - Prints readings via ::PrintReadings.
 *  - Sends the packet with ::esp_now_send and reports success or failure.
 *
 * @param p_params Pointer to task parameters (unused).
 */
void TransmitTask(void *p_params) {
  packet message;

  while(1) {
    // Take the average for each finger and transmit a packet of averaged readings
    message.thumb_reading = ReadQueue(glove.thumb_queue);
    message.index_reading = ReadQueue(glove.index_queue);
    message.middle_reading = ReadQueue(glove.middle_queue);
    message.ring_reading = ReadQueue(glove.ring_queue);
    message.pinkie_reading = ReadQueue(glove.pinkie_queue);
    message.palm_reading = ReadQueue(glove.palm_queue);

    PrintReadings(&message);

    // Send message over esp_now to receiver
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

/**
 * @brief Read a single ADC value and send it to a FreeRTOS queue.
 *
 * This function performs an ADC reading on the specified pin (in millivolts)
 * and attempts to enqueue the reading into the provided queue.
 *
 * @param pin   GPIO pin connected to the flex sensor (ADC input).
 * @param queue Handle to the FreeRTOS queue for this sensor.
 *
 * @return pdPASS on success, or an error code from ::xQueueSend on failure.
 */
BaseType_t SendToQueue(const uint32_t pin, QueueHandle_t queue) {
  BaseType_t retVal;
  uint32_t flex_reading;

  flex_reading = analogReadMilliVolts(pin);
  
  retVal = xQueueSend(queue, &flex_reading, 0);
  return retVal;
}

/**
 * @brief Empty a queue and return the average of all readings.
 *
 * This function repeatedly calls ::xQueueReceive with zero block time until
 * the queue is empty. It computes the average of all values removed.
 *
 * @param queue Handle to the FreeRTOS queue containing sensor readings.
 *
 * @return Average of all readings in the queue, or 0 if the queue was empty.
 */
uint16_t ReadQueue(QueueHandle_t queue) {
  uint16_t sum = 0, count = 0;
  uint16_t reading;

  // Read from queue until empty and take average
  while(xQueueReceive(queue, &reading, 0) == pdTRUE) {
    sum += reading;
    count++;
  }

  if (count > 0) {
    return (sum / count);
  } else {
    return 0; // Don't want to divide by zero if nothing in queue
  }  
}

/**
 * @brief Print formatted finger readings to the Serial monitor.
 *
 * @param hand Pointer to a @ref packet structure containing all finger readings.
 */
void PrintReadings(packet *hand) {
  Serial.print("Thumb: ");
  Serial.println(hand->thumb_reading);

  Serial.print("\tIndex: ");
  Serial.println(hand->index_reading);

  Serial.print("\t\t\tMiddle: ");
  Serial.println(hand->middle_reading);

  Serial.print("\t\t\t\tRing: ");
  Serial.println(hand->ring_reading);

  Serial.print("\t\t\t\t\tPinkie: ");
  Serial.println(hand->pinkie_reading);

  Serial.print("\t\t\t\t\t\tPalm: ");
  Serial.println(hand->palm_reading);
}

/**
 * @brief ESP-NOW send callback called after a packet is transmitted.
 *
 * This callback is registered via ::esp_now_register_send_cb and reports
 * whether the last transmission was successful.
 *
 * @param mac_addr MAC address of the peer to which the data was sent.
 * @param status   Transmission status, e.g. ::ESP_NOW_SEND_SUCCESS on success.
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/**
 * @brief Initialize ESP-NOW communications and register the receiver peer.
 *
 * This function:
 *  - Puts WiFi into station mode.
 *  - Initializes ESP-NOW.
 *  - Registers ::OnDataSent as the send callback.
 *  - Adds the receiver peer specified by ::BROADCAST_ADDRESS.
 *
 * @return true on success, false if initialization or peer registration fails.
 */
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


/**
 * @brief Arduino setup function.
 *
 * Initializes Serial, ESP-NOW communications, and creates all FreeRTOS tasks
 * used by the application.
 */
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

/**
 * @brief Arduino main loop.
 *
 * Not used.
 */
void loop() {
  vTaskDelay(10000);
}
/**
 * @file main.cpp
 * @brief ESP-NOW glove transmitter using FreeRTOS tasks to read flex sensors and send averaged values.
 */

#include "main.h"

/** @brief ESP-NOW peer configuration information. */
esp_now_peer_info_t peerInfo;

/** @brief Global Hand instance representing the glove and its sensor queues. */
Hand glove = Hand();

/**
 * @brief FreeRTOS task that continuously reads flex sensors and pushes values into per-finger queues.
 *
 * The task:
 *  - Configures ADC resolution and attenuation.
 *  - Periodically reads each sensor using ::SendToQueue.
 *  - Logs an error over Serial if queue send fails.
 *
 * @param p_params Pointer to task parameters (unused).
 */
void ReadSensorsTask(void *p_params) {
  // Set ADC settings
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_ATTENUATION);

  BaseType_t retVal;

  while(1) {
    // Read latest sensor data and add to queue for each finger
    retVal = SendToQueue(THUMB_PIN, glove.thumb_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to thumb queue\n");
    }

    retVal = SendToQueue(INDEX_PIN, glove.index_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to index queue\n");
    }

    retVal = SendToQueue(MIDDLE_PIN, glove.middle_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to middle queue\n");
    }

    retVal = SendToQueue(RING_PIN, glove.ring_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to ring queue\n");
    }

    retVal = SendToQueue(PINKIE_PIN, glove.pinkie_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to pinkie queue\n");
    }

    retVal = SendToQueue(PALM_PIN, glove.palm_queue);
    if (retVal != pdPASS) {
      Serial.println("Failed to add to palm queue\n");
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms freq
  }
}

/**
 * @brief FreeRTOS task that averages queue data for each finger and sends packets over ESP-NOW.
 *
 * The task:
 *  - Reads and averages queued sensor values with ::ReadQueue.
 *  - Packages readings into a @ref packet.
 *  - Prints readings via ::PrintReadings.
 *  - Sends the packet with ::esp_now_send and reports success or failure.
 *
 * @param p_params Pointer to task parameters (unused).
 */
void TransmitTask(void *p_params) {
  packet message;

  while(1) {
    // Take the average for each finger and transmit a packet of averaged readings
    message.thumb_reading = ReadQueue(glove.thumb_queue);
    message.index_reading = ReadQueue(glove.index_queue);
    message.middle_reading = ReadQueue(glove.middle_queue);
    message.ring_reading = ReadQueue(glove.ring_queue);
    message.pinkie_reading = ReadQueue(glove.pinkie_queue);
    message.palm_reading = ReadQueue(glove.palm_queue);

    PrintReadings(&message);

    // Send message over esp_now to receiver
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

/**
 * @brief Read a single ADC value and send it to a FreeRTOS queue.
 *
 * This function performs an ADC reading on the specified pin (in millivolts)
 * and attempts to enqueue the reading into the provided queue.
 *
 * @param pin   GPIO pin connected to the flex sensor (ADC input).
 * @param queue Handle to the FreeRTOS queue for this sensor.
 *
 * @return pdPASS on success, or an error code from ::xQueueSend on failure.
 */
BaseType_t SendToQueue(const uint32_t pin, QueueHandle_t queue) {
  BaseType_t retVal;
  uint32_t flex_reading;

  flex_reading = analogReadMilliVolts(pin);
  
  retVal = xQueueSend(queue, &flex_reading, 0);
  return retVal;
}

/**
 * @brief Empty a queue and return the average of all readings.
 *
 * This function repeatedly calls ::xQueueReceive with zero block time until
 * the queue is empty. It computes the average of all values removed.
 *
 * @param queue Handle to the FreeRTOS queue containing sensor readings.
 *
 * @return Average of all readings in the queue, or 0 if the queue was empty.
 */
uint16_t ReadQueue(QueueHandle_t queue) {
  uint16_t sum = 0, count = 0;
  uint16_t reading;

  // Read from queue until empty and take average
  while(xQueueReceive(queue, &reading, 0) == pdTRUE) {
    sum += reading;
    count++;
  }

  if (count > 0) {
    return (sum / count);
  } else {
    return 0; // Don't want to divide by zero if nothing in queue
  }  
}

/**
 * @brief Print formatted finger readings to the Serial monitor.
 *
 * @param hand Pointer to a @ref packet structure containing all finger readings.
 */
void PrintReadings(packet *hand) {
  Serial.print("Thumb: ");
  Serial.println(hand->thumb_reading);

  Serial.print("\tIndex: ");
  Serial.println(hand->index_reading);

  Serial.print("\t\t\tMiddle: ");
  Serial.println(hand->middle_reading);

  Serial.print("\t\t\t\tRing: ");
  Serial.println(hand->ring_reading);

  Serial.print("\t\t\t\t\tPinkie: ");
  Serial.println(hand->pinkie_reading);

  Serial.print("\t\t\t\t\t\tPalm: ");
  Serial.println(hand->palm_reading);
}

/**
 * @brief ESP-NOW send callback called after a packet is transmitted.
 *
 * This callback is registered via ::esp_now_register_send_cb and reports
 * whether the last transmission was successful.
 *
 * @param mac_addr MAC address of the peer to which the data was sent.
 * @param status   Transmission status, e.g. ::ESP_NOW_SEND_SUCCESS on success.
 */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

/**
 * @brief Initialize ESP-NOW communications and register the receiver peer.
 *
 * This function:
 *  - Puts WiFi into station mode.
 *  - Initializes ESP-NOW.
 *  - Registers ::OnDataSent as the send callback.
 *  - Adds the receiver peer specified by ::BROADCAST_ADDRESS.
 *
 * @return true on success, false if initialization or peer registration fails.
 */
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


/**
 * @brief Arduino setup function.
 *
 * Initializes Serial, ESP-NOW communications, and creates all FreeRTOS tasks
 * used by the application.
 */
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

/**
 * @brief Arduino main loop.
 *
 * Not used.
 */
void loop() {
  vTaskDelay(10000);
}