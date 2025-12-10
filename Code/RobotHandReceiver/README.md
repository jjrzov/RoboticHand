# Robot Hand Receiver

This folder contians the platform.io project for the receiver in the project.
The receiver is tasked with receiving flex readings from the transmitter and
moving the motors to bend the finger(s) to match the user's movements.

<img width="572" height="154" alt="image" src="https://github.com/user-attachments/assets/22ad9c68-781f-4aae-846d-7565a1a44701" />

There are 2 tasks for this project, the Receiver Task and the Controller Task. The Receiver Task is blocking waiting for the ESP-NOW callback to run meaning that data from the transmitter has arrived. The callback is made to be extermely fast and quick to ensure safe non critical behavior:

```
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

  return;
}
```

Since the transmitter is unicasting specifically to the MAC address of the receiver ESP32, we do not need to worry about setting up the MAC address like we did in the receiver.

Once having received data the Receiver Task converts the raw flex readings into potentiometer-space targets:

```
float ratio = (flex_value - F_MIN) / (F_MAX - F_MIN);
float enc   = E_MIN + ratio * (E_MAX - E_MIN);
```
Each target is then sent to it's dedicated finger queue. Similar to the Transmitter Task, a Hand class is used to house all the queues.

The Controller Task implements a 2 state FSM:

<img width="454" height="204" alt="image" src="https://github.com/user-attachments/assets/b91002b2-3764-4f0b-944b-b6a7779c19a5" />

Though before entering a state, the Controller reads from the encoders on each loop. There are rotary potentiometers attached to each motor that serve as encoder values. To read the current position of each motor, the potentiometers are connected to an 8 channel external ADC chip that communicates over I2C. We use the official Adafruit library for interfacing with this chip:

```
/**
 * @brief External ADC instance for rotary encoder (potentiometer) readings.
*/
Adafruit_ADS7830 ad7830;
```

In the Homing state, each funger motor is driver towards a known home position

```
Hand_Data home = {20, 20, 20, 20, 20, 20};
```

Once all the fingers are homed the system transition to Position Control state. During position control, the task performs non blocking reads of the latest target value from the finger queues:

```
if (xQueueReceive(glove.thumb_queue, &val, 0) == pdTRUE)
  targets.thumb_data = val;

```

For both Homing and Position Control, motor movement is calculated through a PID. Each finger/joint has its own PID that takes in the current potentiometer value and the target potentiometer value. There are 2 separate functions for running the PID in the Homing state versus the Position Control state. The Homing state only utilizes a P controller while the Position Control has a PID controller. 

```
void PositionControlFinger(uint32_t enc, uint32_t target, PID_t *pid, int pwm_channel, int phase_pin, float dt);

bool HomeFinger(uint32_t enc, uint32_t home_target, int pwm_channel, int phase_pin);
```

This distinction was made because when homing we only want to be around a certain threshold of potentiometer values to be home but we want more precise control when trying to match the user's motion.

The magnitude of the PID and P controller becomes the speed of the motor.

PID Controller:

```
float error = (float)target - (float)enc;
float u = PID_Step(pid, error, dt);  // signed

SetPhase(u, phase_pin); // Find direction to move in

float mag = fabsf(u);
int pwm = (int)mag;
```

P Controller:
```
float u = HOMING_KP * (float)err;
SetPhase(u, phase_pin);

float mag_u = fabsf(u);
int pwm = (int)mag_u; 
```

while the direction comes from the sign of the output.

```
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
  digitalWrite(motor_phase_pin,(u >= 0.0f) ? HIGH : LOW);
}
```