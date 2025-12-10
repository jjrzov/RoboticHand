# Robot Hand Transmitter

This folder holds the platform.io project for the transmitter. The main task of
this project is to read the bend of the glove and transmit the data to the 
receiver.

In order to mimic the user's hand motions while wearing the glove, we used flex
sensors that act like potentiometers. Depending on how much the user is bending
the flex sensor, the resistance will increase or decrease. By creating a simple
voltage divider circuit with the flex sensor and a 100 ohm resistor, we can read
how the voltage is changing through ADC channels on our ESP-32.

<img width="578" height="149" alt="image" src="https://github.com/user-attachments/assets/309f3436-5216-49cf-a61c-420ee75bec58" />

Delving into the software, we can see the basic task architecure above. There are only 2 tasks for this project with the read sensors task running every 10ms and then sending that data over to the transmit task to then be sent to the receiver. 

Each finger has its own queue when communicating between tasks, but to make the code simpler a Hand class was created. This Hand class holds all the queues and is used as a way to access each queue from the tasks.

In the read sensors task, the voltage is read from each flex sensor and published to a queue. The transmit task is constantly reading from each queue until it is empty. An average is then taken while making sure to avoid dividng by zero. 

'''
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
'''  

Setup for ESP-NOW is all done in the setup function before the scheduler begins. To avoid broadcasting to any ESP-32s nearby, we broadcast specifically to the MAC address of the device we want

'''
  // Register peer
  memcpy(peerInfo.peer_addr, BROADCAST_ADDRESS, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return false;
  }
'''