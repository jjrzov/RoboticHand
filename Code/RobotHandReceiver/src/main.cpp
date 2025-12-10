/**
 * @file main.cpp
 * @brief ESP-NOW robotic hand receiver and PID motor controller.
 *
 * This application:
 *  - Receives flex sensor data via ESP-NOW
 *  - Converts flex readings to encoder targets
 *  - Performs homing and position control using PID
 *  - Drives DC motors using PWM + phase control
 *
 * FreeRTOS tasks are used for communication and control.
 */

#include "Constants.h"
#include "pid.h"

/** @name PID controllers for each finger and palm
 *  @{
 */
PID_t pid_thumb;
PID_t pid_index;
PID_t pid_middle;
PID_t pid_ring;
PID_t pid_pinkie;
PID_t pid_palm;
/** @} */


/**
 * @brief Queue container for desired encoder targets for each finger.
 */
Hand_Queues glove = Hand_Queues();

/**
 * @brief Queue used to receive ESP-NOW packets from the glove transmitter.
 */
QueueHandle_t packet_queue = xQueueCreate(1, sizeof(Hand_Data));


/**
 * @brief External ADC instance for rotary encoder (potentiometer) readings.
 */
Adafruit_ADS7830 ad7830;

/**
 * @brief ESP-NOW peer information structure.
 */
esp_now_peer_info_t peerInfo;


/**
 * @brief Convert flex sensor reading to encoder target value.
 *
 * Performs clamping and linear interpolation between calibrated
 * flex sensor limits and encoder limits.
 *
 * @param flex_value Raw flex sensor reading.
 * @return Corresponding encoder target value.
 */
uint32_t FlexToEncoder(uint32_t flex_value) { 
  // Clamp input
  if (flex_value <= F_MIN) return (uint32_t)E_MIN;
  if (flex_value >= F_MAX) return (uint32_t)E_MAX;

  // Linear interpolation
  float ratio = (flex_value - F_MIN) / (F_MAX - F_MIN);
  float enc = E_MIN + ratio * (E_MAX - E_MIN);

  return (uint32_t)enc;
}

/**
 * @brief FreeRTOS task that processes received ESP-NOW packets.
 *
 * This task:
 *  - Waits for incoming Hand_Data packets
 *  - Converts flex values to encoder targets
 *  - Dispatches targets to individual finger queues
 *
 * @param p_params Task parameters (unused).
 */
void ReceiverTask(void *p_params) {
  Hand_Data recv_data, enc_data;
  
  while(1) {
    if (xQueueReceive(packet_queue, &recv_data, portMAX_DELAY) == pdTRUE) {
      // Convert from flex value to encoder value
      enc_data.thumb_data = FlexToEncoder(recv_data.thumb_data);
      enc_data.index_data = FlexToEncoder(recv_data.index_data);
      enc_data.middle_data = FlexToEncoder(recv_data.middle_data);
      enc_data.ring_data = FlexToEncoder(recv_data.ring_data);
      enc_data.pinkie_data = FlexToEncoder(recv_data.pinkie_data);
      enc_data.palm_data = FlexToEncoder(recv_data.palm_data);

      // Send data to other tasks with queue
      xQueueSend(glove.thumb_queue, &enc_data.thumb_data, 0);
      xQueueSend(glove.index_queue, &enc_data.index_data, 0);
      xQueueSend(glove.middle_queue, &enc_data.middle_data, 0);
      xQueueSend(glove.ring_queue, &enc_data.ring_data, 0);
      xQueueSend(glove.pinkie_queue, &enc_data.pinkie_data, 0);
      xQueueSend(glove.palm_queue, &enc_data.palm_data, 0);
    }
  }
}

/**
 * @brief ESP-NOW receive callback.
 *
 * Copies the received packet into a FreeRTOS queue for processing
 * by ::ReceiverTask.
 *
 * @param mac          MAC address of sender.
 * @param incomingData Pointer to received data buffer.
 * @param len          Length of received data in bytes.
 */
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {  
  Hand_Data data;
  memcpy(&data, incomingData, sizeof(data));
  xQueueSend(packet_queue, &data, 0);
  // Serial.println(data.index_data);
  return;
}


/**
 * @brief Initialize ESP-NOW communication subsystem.
 *
 * Sets WiFi to station mode, initializes ESP-NOW,
 * and registers the receive callback.
 *
 * @return true on success, false on failure.
 */
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

/**
 * @brief Initialize the external ADS7830 ADC.
 *
 * @return true if ADC initialization succeeds, false otherwise.
 */
bool InitADC() {
  // Initialize external ADC
  if (!ad7830.begin()) {
    Serial.println("Failed to initialize ADS7830");
    return false;
  }
  return true;
}

/**
 * @brief Initialize PWM channels and GPIOs for all motors.
 *
 * Configures PWM frequency, resolution, output pins,
 * and phase direction pins for each motor.
 */
void InitMotors() {
  // Initialize pins to control motors

  // Thumb
  ledcSetup(THUMB_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(THUMB_PWM_PIN, THUMB_PWM_CHANNEL);
  pinMode(THUMB_PH_PIN, OUTPUT);

  // Index
  ledcSetup(INDEX_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(INDEX_PWM_PIN, INDEX_PWM_CHANNEL);
  pinMode(INDEX_PH_PIN, OUTPUT);

  // Middle
  ledcSetup(MIDDLE_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MIDDLE_PWM_PIN, MIDDLE_PWM_CHANNEL);
  pinMode(MIDDLE_PH_PIN, OUTPUT);

  // Ring
  ledcSetup(RING_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(RING_PWM_PIN, RING_PWM_CHANNEL);
  pinMode(RING_PH_PIN, OUTPUT);

  // Pinkie
  ledcSetup(PINKIE_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(PINKIE_PWM_PIN, PINKIE_PWM_CHANNEL);
  pinMode(PINKIE_PH_PIN, OUTPUT);

  // Palm
  ledcSetup(PALM_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(PALM_PWM_PIN, PALM_PWM_CHANNEL);
  pinMode(PALM_PH_PIN, OUTPUT);
  return;
}

/**
 * @brief Read a rotary encoder value via the external ADC.
 *
 * @param ext_adc_num External ADC channel number.
 * @return Raw ADC reading corresponding to encoder position.
 */
uint32_t ReadEncoder(uint8_t ext_adc_num) {
  // Read encoder value from rotary potentiometer
  return ad7830.readADCsingle(ext_adc_num);
}

/**
 * @brief Stop all motors immediately.
 *
 * Sets all PWM duty cycles to zero.
 */
void StopAllMotors() {
  // Turn off all motors by setting pwm to zero
  ledcWrite(THUMB_PWM_CHANNEL, 0);
  ledcWrite(INDEX_PWM_CHANNEL, 0);
  ledcWrite(MIDDLE_PWM_CHANNEL, 0);
  ledcWrite(RING_PWM_CHANNEL, 0);
  ledcWrite(PINKIE_PWM_CHANNEL, 0);
  ledcWrite(PALM_PWM_CHANNEL, 0);
  return;
}

/**
 * @brief Main control FreeRTOS task implementing the hand state machine.
 *
 * States:
 *  - HOMING_STATE: Drive all fingers to known home positions
 *  - POSITION_CONTROL_STATE: Run PID control using received targets
 *
 * @param p_params Task parameters (unused).
 */
void ControllerTask(void *p_params) {
  HandState state = HOMING_STATE;

  // home encoder counts (fill these with your real home values)
  Hand_Data home = {20, 20, 20, 20, 20, 115};

  // current target positions (for POSITION_CONTROL)
  Hand_Data targets = home;  // start at home

  InitPIDs();
  ResetAllPIDs();

  const float dt = 0.02f; // 20 ms in seconds

  while(1) {
    Serial.print("State: ");
    Serial.println(state);
    // Read actual encoder values 
    uint32_t enc_thumb = ReadEncoder(EXT_ADC_THUMB);
    uint32_t enc_index = ReadEncoder(EXT_ADC_INDEX);
    uint32_t enc_middle = ReadEncoder(EXT_ADC_MIDDLE);
    uint32_t enc_ring = ReadEncoder(EXT_ADC_RING);
    uint32_t enc_pinkie = ReadEncoder(EXT_ADC_PINKIE);
    uint32_t enc_palm = ReadEncoder(EXT_ADC_PALM);

    switch (state) {
      case HOMING_STATE: {
        bool thumb_ok  = HomeFinger(enc_thumb,  home.thumb_data,
                                    THUMB_PWM_CHANNEL,  THUMB_PH_PIN);

        bool index_ok  = HomeFinger(enc_index,  home.index_data,
                                    INDEX_PWM_CHANNEL,  INDEX_PH_PIN);

        bool middle_ok  = HomeFinger(enc_middle,  home.middle_data,
                                    MIDDLE_PWM_CHANNEL,  MIDDLE_PH_PIN);

        bool ring_ok  = HomeFinger(enc_ring,  home.ring_data,
                                    RING_PWM_CHANNEL,  RING_PH_PIN);

        bool pinkie_ok  = HomeFinger(enc_pinkie,  home.pinkie_data,
                                    PINKIE_PWM_CHANNEL,  PINKIE_PH_PIN);

        bool palm_ok  = HomeFinger(enc_palm,  home.palm_data,
                                    PALM_PWM_CHANNEL,  PALM_PH_PIN);

        bool all_homed = thumb_ok && index_ok && middle_ok && ring_ok 
                              && pinkie_ok && palm_ok;

        if (all_homed) {
          StopAllMotors();
          state = POSITION_CONTROL_STATE;
        }
        break;
      }

      case POSITION_CONTROL_STATE: {
        // Get latest desired targets from ESP-NOW queues (non-blocking)
        uint32_t val;

        if (xQueueReceive(glove.thumb_queue,  &val, 0) == pdTRUE) {
          targets.thumb_data = val;
        }

        if (xQueueReceive(glove.index_queue,  &val, 0) == pdTRUE) {
          targets.index_data = val;
        }

        if (xQueueReceive(glove.middle_queue,  &val, 0) == pdTRUE) {
          targets.middle_data = val;
        }

        if (xQueueReceive(glove.ring_queue,  &val, 0) == pdTRUE) {
          targets.ring_data = val;
        }

        if (xQueueReceive(glove.pinkie_queue,  &val, 0) == pdTRUE) {
          targets.pinkie_data = val;
        }

        if (xQueueReceive(glove.palm_queue,  &val, 0) == pdTRUE) {
          targets.palm_data = val;
        }
        
        // Run PID for each finger
        PositionControlFinger(enc_thumb,  targets.thumb_data,
                              &pid_thumb,  THUMB_PWM_CHANNEL,  THUMB_PH_PIN,  dt);

        PositionControlFinger(enc_index,  targets.index_data,
                              &pid_index,  INDEX_PWM_CHANNEL,  INDEX_PH_PIN,  dt);

        PositionControlFinger(enc_middle,  targets.middle_data,
                              &pid_middle,  MIDDLE_PWM_CHANNEL,  MIDDLE_PH_PIN,  dt);

        PositionControlFinger(enc_ring,  targets.ring_data,
                              &pid_ring,    RING_PWM_CHANNEL,  RING_PH_PIN,  dt);

        PositionControlFinger(enc_pinkie,  targets.pinkie_data,
                              &pid_pinkie,  PINKIE_PWM_CHANNEL,  PINKIE_PH_PIN,  dt);

        PositionControlFinger(enc_palm,  targets.palm_data,
                              &pid_palm,  PALM_PWM_CHANNEL,  PALM_PH_PIN,  dt);

        break;
      }
      
      default:
        StopAllMotors();
        break;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);  // 20ms period
  }
}

/**
 * @brief Arduino setup function.
 *
 * Initializes communication, ADC, motors, and creates FreeRTOS tasks.
 */
void setup() {
  Serial.begin(115200);
  delay(2000);
  BaseType_t retVal;

  InitComms();
  InitADC();
  InitMotors();

  retVal = xTaskCreate(ControllerTask, "Controller Task", 2048, NULL, 1, NULL);
  if (retVal != pdPASS) {
    Serial.println("Failed to create Bootup Task\n");
  }
  
  retVal = xTaskCreate(ReceiverTask, "Receiver Task", 1024, NULL, 2, NULL);
  if (retVal != pdPASS) {
    Serial.println("Failed to create Receiver Task\n");
  }

}

/**
 * @brief Arduino main loop (unused).
 */
void loop() {
  vTaskDelay(1000000);
}