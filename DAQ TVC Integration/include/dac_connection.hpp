#ifndef DAC_CONNECTION_HPP
#define DAC_CONNECTION_HPP

#include <QNEthernet.h>

using namespace qindesign::network;

enum states { BRAKE, CALIBRATE, VECTOR, ABORT };
enum statuses { DISCONNECTED, CONNECTED };

const IPAddress LOCALIP(192, 168, 0, 102);
const uint8_t MAC[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
constexpr uint16_t PORT = 19690;

const int COMMAND_LENGTH = 5;

class DAC_CONNECTION {
private:
    int status;
    int state;
    
    int message_length;
    char message[COMMAND_LENGTH];
    EthernetServer server;
    EthernetClient client;

public:
    DAC_CONNECTION();  // Constructor
    ~DAC_CONNECTION(); // Destructor (No Dynamic Memory Allocation)

    bool initialize(); // Establish MAC address and IP of Teensy
    bool connect();    // Wait for and establish a connection
    bool update();     // Check for messages and update state

    int getState();    // Get the current state
    int getStatus();   // Get the current connection status
    void setState(int state); // Set a new state
};

#endif