#include <Arduino.h>

// Import Ethernet Connection File
#include "dac_connection.hpp"

// Initialize Object
DAC_CONNECTION dac;

int tvc_state = BRAKE;

void setup() {
  Serial.begin(115200); 
  delay(2000);

  if (dac.initialize()) {
    // Initialize the DAC Server
    Serial.println("DAC INITIALIZATION SUCCESS!");
  }
  else {
    Serial.println("DAC INITIALIZAITON FAILED.");
    return;
  }

  if (dac.connect()) {
    Serial.println("CONNECTED!");
  }
  else {
    Serial.println("CONNECTION FAILED.");
  }

  Serial.println("SKIP");
}

void loop() {
  if (dac.update()) {
    // If a new message has come in from dac, update our current state. 
    Serial.println(dac.getState());
  }
}