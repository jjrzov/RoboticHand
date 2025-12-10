# Overall Software Architecture

The code features 2 separate platform.io projects, one for the transmitter
and the other for the receiver. Both projects use an ESP-32 and FreeRTOS with
communication between the both ESPs happening through ESP-NOW. 