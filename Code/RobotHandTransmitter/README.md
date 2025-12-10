# Robot Hand Transmitter

This folder holds the platform.io project for the transmitter. The main task of
this project is to read the bend of the glove and transmit the data to the 
receiver.

In order to mimic the user's hand motions while wearing the glove, we used flex
sensors that act like potentiometers. Depending on how much the user is bending
the flex sensor, the resistance will increase or decrease. By creating a simple
voltage divider circuit with the flex sensor and a 100 ohm resistor, we can read
how the voltage is changing through ADC channels on our ESP-32.