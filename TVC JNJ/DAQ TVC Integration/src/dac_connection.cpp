#include "dac_connection.hpp"
#include <cstring>
#include <SD.h>

// Constructor
DAC_CONNECTION::DAC_CONNECTION()
    : status(DISCONNECTED), state(CALIBRATE), linkState(DISC),message_length(0), server(PORT) {}

// Destructor (No Dynamic Memory Allocation)
DAC_CONNECTION::~DAC_CONNECTION() {}

// Establish MAC address and IP of Teensy
bool DAC_CONNECTION::initialize() {
    Ethernet.begin(MAC, (uint32_t)LOCALIP);

    // Check whether Ethernet Shield is Attached
    if (!Ethernet.linkState()) {
        return false;
    }
    else {
        this -> setLinkState(CONN);
    }

    this -> ip = Ethernet.localIP();

    // Start Server
    server.begin();

    return true;
}

// Wait for and establish a connection
bool DAC_CONNECTION::connect() {
    
    this->client = this->server.available();

    if (this->client) {
        status = CONNECTED;
        return true;
    }
    
    return false;
}

// Check for messages and update state
bool DAC_CONNECTION::update() {

    if ((this -> getLinkState() == DISC) || (this -> getStatus() == DISCONNECTED)) return false;

    if (this -> client.available()){
        
        while (this -> client.available() && this -> message_length < COMMAND_LENGTH + 1) {
            char c = this -> client.read();
            if (c == '#') {
                this -> message[this -> message_length] = '\0'; // Proper String Termination
                this -> message_length = 0; // Resets for future messages 

                if (strcmp(this -> message, "VECTR") == 0) {this -> setState(VECTOR); this -> client.print("ACK#");}
                else if (strcmp(this -> message, "CALBR") == 0) {this -> setState(CALIBRATE); this -> client.print("ACK#");}
                else if (strcmp(this -> message, "BRAKE") == 0) {this -> setState(BRAKE); this -> client.print("ACK#");}
                else if (strcmp(this -> message, "READS") == 0) {
                    if(fileOpened == 1){
                        dataFile.println("ACK##");
                        dataFile.close(); // close the file if it is open
                        fileOpened = 0;
                    }
                    if (!dataFile) {
                        dataFile = SD.open("TVCdata.txt", FILE_READ);  // only open once
                    }
                    String tem = dataFile.readStringUntil('\n');
                    if(tem != "ACK##"){
                        this -> client.print(tem);
                    }
                    else{
                        dataFile.close(); // close the file if it is open
                        this -> client.print("ACK##");
                    }
                }
                else return false; // Invalid Command

                return true; 
            }
            else {
                // Unfinished Command, Continue Processing
                this -> message[this -> message_length] = c;
                ++this -> message_length;
            }
        }
    }
    
    return false; 
}

void DAC_CONNECTION::updateStatus() {
    if (this -> client && this -> client.connected()) this -> setStatus(CONNECTED);
    else this -> setStatus(DISCONNECTED);
}

void DAC_CONNECTION::updateLinkState() {
    if (!Ethernet.linkState()) {
        client.stop();
        this -> setLinkState(DISC);

        // If ethernet is disconnected, all connections invalid 
        this -> setStatus(DISCONNECTED);
    }
    else {
        this -> setLinkState(CONN);
    }
}

// Get the current state
int DAC_CONNECTION::getState() {
    return this->state;
}

// Get the current connection status
int DAC_CONNECTION::getStatus() {
    return this->status;
}

int DAC_CONNECTION::getLinkState() {
    return this -> linkState;
}

// Set a new state
void DAC_CONNECTION::setState(int state) {
    this->state = state;
}

void DAC_CONNECTION::setStatus(int status) {
    this -> status = status; 
}

void DAC_CONNECTION::setLinkState(int linkState) {
    this -> linkState = linkState;
}