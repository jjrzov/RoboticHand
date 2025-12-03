// #include "Hand.h"

// // class Hand {
// //     public:
// //         QueueHandle_t thumb;
// //         QueueHandle_t index;
// //         QueueHandle_t middle;
// //         QueueHandle_t ring;
// //         QueueHandle_t pinkie;
// //         QueueHandle_t palm;
// // };


// class Finger {
//     protected:
//         gpio_num_t pin;
//         QueueHandle_t value;

//     public: 
//         Finger(gpio_num_t GPIO_NUM) {
//             pin = GPIO_NUM;
//         }

//         int32_t GetLatestReading() {
//             return flex_reading;
//         }

//         void ReadFlex() {
//             flex_reading = analogReadMilliVolts(pin);
//         }
// };

// // class Hand {
// //     protected:
// //         Finger thumb;
// //         Finger index;
// //         Finger middle;
// //         Finger ring;
// //         Finger pinkie;
// //         Finger palm;

// //     public:
// //         Hand(gpio_num_t thumb_pin, gpio_num_t index_pin, gpio_num_t middle_pin,
// //                 gpio_num_t ring_pin, gpio_num_t pinkie_pin, gpio_num_t palm_pin)
// //             : thumb(thumb_pin), index(index_pin), middle(middle_pin),
// //                 ring(ring_pin), pinkie(pinkie_pin), palm(palm_pin)
// //         {
            
// //         }

// //         // Update all flex readings to latest value
// //         void Hand::ReadSensors() {
// //             thumb.ReadFlex();
// //             index.ReadFlex();
// //             middle.ReadFlex();
// //             ring.ReadFlex();
// //             pinkie.ReadFlex();
// //             palm.ReadFlex();
// //         }

// //         // Return sensor values
// //         uint32_t Hand::GetSensorVals() {
// //             return thumb.GetLatestReading(), 
// //                 index.GetLatestReading(),
// //                 middle.GetLatestReading(),
// //                 ring.GetLatestReading(),
// //                 pinkie.GetLatestReading(),
// //                 palm.GetLatestReading();
// //         }
// // };
