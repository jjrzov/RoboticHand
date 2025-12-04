#include "Constants.h"
#include "pid.h"

PID_t pid_thumb;

Hand_Queues glove = Hand_Queues();

// Receiver Queue
QueueHandle_t packet_queue = xQueueCreate(1, sizeof(Hand_Data));

// External ADC Instance
Adafruit_ADS7830 ad7830;

// ESP-NOW
esp_now_peer_info_t peerInfo;


uint32_t FlexToEncoder(uint32_t flex_value) {
  // Convert from flex readings to encoder values
  if (flex_value < 200) {
    return 10; // Extended finger (home)
  } else if (flex_value > 200 && flex_value < 600) {
    return 100;
  } else {
    return 190;
  }
}

void ReceiverTask(void *p_params) {
  Hand_Data recv_data, enc_data;
  
  while(1) {
    if (xQueueReceive(packet_queue, &recv_data, portMAX_DELAY) == pdTRUE) {
      // Convert from flex value to encoder value
      enc_data.thumb_data = FlexToEncoder(recv_data.thumb_data);

      // Send data to other tasks with queue
      xQueueSend(glove.thumb_queue, &enc_data.thumb_data, 0);
    }
  }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {  
  Hand_Data data;
  memcpy(&data, incomingData, sizeof(data));
  xQueueSend(packet_queue, &data, 0);
  // Serial.println(data.thumb_data);
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
}

uint32_t ReadEncoder(uint8_t ext_adc_num) {
  // Read encoder value from rotary potentiometer
  return ad7830.readADCsingle(ext_adc_num);
}

void StopAllMotors() {
  // Turn off all motors by setting pwm to zero
  ledcWrite(THUMB_PWM_CHANNEL, 0);
  return;
}

void ControllerTask(void *p_params) {
  HandState state = HOMING_STATE;

  // home encoder counts (fill these with your real home values)
  Hand_Data home = {15};

  // current target positions (for POSITION_CONTROL)
  Hand_Data targets = home;  // start at home

  InitPIDs();
  ResetAllPIDs();

  const float dt = 0.02f; // 20 ms in seconds

  while(1) {
    // Serial.print("State: ");
    // Serial.println(state);
    // Read actual encoder values 
    uint32_t enc_thumb  = ReadEncoder(EXT_ADC_THUMB);
    // Serial.print("Curr Enc: ");
    // Serial.println(enc_thumb);

    switch (state) {
      case HOMING_STATE: {
        bool thumb_ok  = HomeFinger(enc_thumb,  home.thumb_data,
                                    THUMB_PWM_CHANNEL,  THUMB_PH_PIN);

        bool all_homed = thumb_ok;

        if (all_homed) {
          StopAllMotors();
          // ResetAllPIDs();               // clear integrators before PID mode
          state = POSITION_CONTROL_STATE;
          // Serial.println("Finished homing");
        }
        break;
      }

      case POSITION_CONTROL_STATE: {
        // 2) Get latest desired targets from ESP-NOW queues (non-blocking)
        uint32_t val;
        if (xQueueReceive(glove.thumb_queue,  &val, 0) == pdTRUE) {
          targets.thumb_data = val;
          // Serial.print("Newly Received Data: ");
          // Serial.println(val);
        }
        
        // Serial.print("Thumb target: ");
        // Serial.println(targets.thumb_data);
        // 3) Run PID for each finger
        PositionControlFinger(enc_thumb,  targets.thumb_data,
                              &pid_thumb,  THUMB_PWM_CHANNEL,  THUMB_PH_PIN,  dt);

        break;
      }

      case ERROR_STATE: {
        StopAllMotors();

        break;
      }
      default:
        StopAllMotors();

        break;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);  // 20ms period
  }
}

void setup() {
  Serial.begin(115200);
  delay(5000);
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

void loop() {
  vTaskDelay(1000000);
}