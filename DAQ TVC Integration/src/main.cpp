#include <Arduino.h>

// Import Ethernet Connection File
#include "dac_connection.hpp"

// Initialize Object
DAC_CONNECTION dac;

int tvc_state = BRAKE;

void setup() {
  Serial.begin(115200); 
  while (!Serial) {};
  Serial.println("STARTING...");

  if (dac.initialize()) {
    // Initialize the DAC Server
    Serial.println("DAC INITIALIZATION SUCCESS! with IP");
    Serial.println(dac.ip);
  }
  else {
    Serial.println("DAC INITIALIZAITON FAILED.");
    return;
  }

  while(!dac.connect());

  Serial.println("CONNECTED!");
}

void loop() {

  // CHECK IF LINK STATE IS STILL ACTIVE 
  dac.updateLinkState(); 

  // IF LINK STATE IS NO LONGER ACTIVE, WAIT FOR INITIALIZATION + CONNECTION
  if (dac.getStatus() == DISC) {
    Serial.println("ETHERNET CABLE DISCONNECTED, WAITING FOR RECONNECT ...");
    while (!dac.initialize());
    Serial.println("ETHERNET CABLE RECONNECTED!");
  }

  // Check if DAC CONNECTION IS STILL ACTIVE 
  dac.updateStatus();

  // If DAC CONNECTION IS NO LONGER ACTIVE, WAIT FOR RECONNECTION
  if (dac.getStatus() == DISCONNECTED) {
    Serial.println("DAC DISCONNECTED, WAITING FOR RECONNECT ... ");
    while(!dac.connect());
    Serial.println("DAC CONNECTED!");
  }

  // NOW THAT DAC CONNECTION HAS BEEN ESTABLISHED, CHECK FOR MESSAGES 
  if (dac.getStatus() == CONNECTED) {
    if (dac.update()) Serial.println(dac.getState());
  }

  // TVC CODE v
}