#ifndef DAC_CONNECTION_HPP
#define DAC_CONNECTION_HPP

#include <QNEthernet.h>
#include <SD.h>

using namespace qindesign::network;

enum states { BRAKE, CALIBRATE, VECTOR, IDLE};
enum statuses { DISCONNECTED, CONNECTED };
enum link {DISC, CONN};

const IPAddress LOCALIP(192, 168, 1, 151);
const IPAddress SUBNET_MASK(255, 255, 0, 0);
const IPAddress GATEWAY(192, 168, 0, 1);
const uint8_t MAC[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
constexpr uint16_t PORT = 19690;

const int COMMAND_LENGTH = 6;

class DAC_CONNECTION {
private:
    int status;
    int state;
    int linkState;
    File dataFile;
    int fileOpened = 1;
    
    unsigned int message_length;
    char message[COMMAND_LENGTH];
    EthernetServer server;
    EthernetClient client;

    void setState(int state); // Set a new state
    void setStatus(int status); // Set a new status
    void setLinkState(int linkState); // Set a new Link State


public:
    IPAddress ip; 
    DAC_CONNECTION();  // Constructor
    ~DAC_CONNECTION(); // Destructor (No Dynamic Memory Allocation)

    bool initialize(); // Establish MAC address and IP of Teensy
    bool connect();    // Wait for and establish a connection
    bool update();     // Check for messages and update state
    void updateStatus(); // Update the connection status for the current client
    void updateLinkState(); // Updates the current Link State

    int getState();    // Get the current state
    int getStatus();   // Get the current connection status
    int getLinkState(); // Gets the current link state
};

#endif