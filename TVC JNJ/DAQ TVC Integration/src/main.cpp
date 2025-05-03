#ifndef __IMXRT1062__
#error "This sketch should be compiled for Teensy 4.x"
#endif

#include <Arduino.h>
#include <ACAN_T4.h>
#include "Moteus.h"
#include <arrays.h>
#include <SD.h>
#include <SPI.h>

// Import Ethernet Connection File
#include "dac_connection.hpp"
//——————————————————————————————————————————————————————————————————————————————
// vector list calculation
//——————————————————————————————————————————————————————————————————————————————
double actuator1_record[1000];
double actuator2_record[1000];

int vector_size = sizeof(y_int) / sizeof(y_int[0]); // Correct for C-style arrays
int increase_index = sqrt(vector_size);             // this is the number of indices between each intercent step
int middle_search = (increase_index - 1)/2;
const std::vector<double> z_sections = []
{
  const double length = vector_size, step = increase_index + 1;
  std::vector<double> vec(static_cast<int>(length / step) + 1);
  for (size_t i = 0; i < vec.size(); i++)
    vec[i] = i * step;
  return vec;
}();

static double conversion_factor = 0.1575;

//——————————————————————————————————————————————————————————————————————————————
// forward function declaration
//——————————————————————————————————————————————————————————————————————————————

void moteus1_calibration();
void moteus2_calibration();
double find_z_section(int y_int_upper, int y_int_lower);
bool abort_sense(double m1_position,double m2_position);
bool at_edge(double m1_position,double m2_position);
void abort_by_space();

//——————————————————————————————————————————————————————————————————————————————
// TV class 
//——————————————————————————————————————————————————————————————————————————————

class TV {
  public:
    
    double TV_x, TV_y, TV_z;
    double mag;
    double desire_y_int, desire_z_int;
    double act1_position, act2_position;

  //-----------------------------------------------------------------
  // constructor
  //-----------------------------------------------------------------
  TV(double x, double y, double z){
    //-----------------------------------------------------------------
    //normalization
    //-----------------------------------------------------------------
    mag = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    TV_x = x/mag;
    TV_y = y/mag;
    TV_z = z/mag;
    //-----------------------------------------------------------------
    // maps TV -> desire y and z intercept
    //-----------------------------------------------------------------
    desire_y_int = TV_y/TV_x;
    desire_z_int = TV_z/TV_x;
  }

  void actuatorsLength(TV tv){
    double x = desire_y_int;
    double y = desire_z_int;
    //——————————————————————————————————————————————————————————————————————————————
    //  The following equations are result of linear fit in matlab curve fitter
    //——————————————————————————————————————————————————————————————————————————————
    act1_position = 17.0070 + 4.2091 * x + 4.2166 * y - 1.7991 * x * x - 0.5835 * x * y - 1.8018 * y * y;
    act2_position = 17.0070 + (-4.2091) * x + 4.2166 * y + (-1.7991) * x * x + 0.5835 * x * y + (-1.8018) * y * y;
  }


};

//——————————————————————————————————————————————————————————————————————————————
// Initialize Object
//——————————————————————————————————————————————————————————————————————————————

DAC_CONNECTION dac;
Moteus moteus1;
Moteus moteus2;

//——————————————————————————————————————————————————————————————————————————————
// constants 
//——————————————————————————————————————————————————————————————————————————————
static double moteus2_lastPosition;
static double moteus1_lastPosition;
static double moteus1_lastCurrent;
static double moteus2_lastCurrent;



static double moteus1_lastVelo;
static double moteus2_lastVelo;

static double moteus1_torq;
static double moteus2_torq;
static double moteus1_d_current;
static double moteus2_d_current;
static double moteus1_temp;
static double moteus2_temp;
static double moteus1_vol;
static double moteus2_vol;

// static double commanded_position;     // testing, this variable is manually set
static int m1_commandCompleted = 0;   // m1 command completion check
static int m2_commandCompleted = 0;   // m2 command completion check
static int both_commandCompleted = 0;
// static uint32_t gSendNextCommand = 0; 
static uint32_t gNextSendMillis = 0;  // adds 20ms everyloop to ensure we send command every 20 seconds
uint16_t gLoopCount = 0;              // loop counter
static float max_current = 0;         // max current/resistance we observed for motor to experienced
static float limit_current_m1 = 10.0;     // max current/resistance we allow for motor to experienced for moteus 1
static float limit_current_m2 = 12.0;     // max current/resistance we allow for motor to experienced for moteus 2
static int main_loop_counter = 0;     // main loop counter, used to advance in TV list
static int states = 1; // states: 1 = calibration, state = 2 break, state 3 = vector, state 4 = IDLE

//——————————————————————————————————————————————————————————————————————————————
// critical constants
//——————————————————————————————————————————————————————————————————————————————
static int traj_length = vector_size;
static double min_act1 = 15.25;             // updated min length
static double max_act1 = 18.4;             
static double min_act2 = 15.25;
static double max_act2 = 18.4;
static double middle_act1 = 17.00;            // actuator 1 zero position
static double middle_act2 = 17.00;            // actuator 2 zero position
static double abort_current = 12.0 ;           // current which will cause abort  
static double kp_scale_tune = 0.8;
static double kd_scale_tune = 0.5;
static double accel_lim = 6.5;
static double velo_lim = 15; 
static double bubble_zone = 0.05;
static double deviation_zone = 0.1;           // zone which we determine whether we deviated from desire zone

File dataFile;
unsigned long loop_start_time;


//——————————————————————————————————————————————————————————————————————————————
// braking hotfire specific constants and commands
//——————————————————————————————————————————————————————————————————————————————

static boolean deviation = 0;
Moteus::PositionMode::Command act1_forward;
Moteus::PositionMode::Command act2_forward;
Moteus::PositionMode::Command act1_backward;
Moteus::PositionMode::Command act2_backward;

int tvc_state = BRAKE;

void setup() {
  // let the world know we have began ! 
  Serial.begin(115200); 
  // while (!Serial) {};
  Serial.println("STARTING...");

  // initialize ACAN_T4FD and CAN message
  ACAN_T4FD_Settings settings(1000000, DataBitRateFactor::x1);

  const uint32_t errorCode = ACAN_T4::can3.beginFD(settings);
  if (0 == errorCode) {
    Serial.println("can3 ok");
  } else {
    Serial.print("Error can3: 0x");
    Serial.println(errorCode, HEX);
  }

  if(!SD.begin(BUILTIN_SDCARD) ){
    Serial.println("SD card failed");
    while(true){};
  }
  else{
    Serial.println("SD card ok");
  }
  SD.remove("TVCdata.txt");
  dataFile = SD.open("TVCdata.txt", FILE_WRITE);
  if(dataFile) {
    dataFile.println("Time|Act1Cmd|Act1Real|Act1Velo|Act1Torq|Act1Qcurr|Act1Dcurr|Act1Temp|Act1Vol|Act2Cmd|Act2Real|Act2Velo|Act2Torq|Act2Qcurr|Act2Dcurr|Act2Temp|Act2Vol");
    dataFile.flush();
  } else {
    Serial.println("Error generating datafile");
  }
  
  //——————————————————————————————————————————————————————————————————————————————
  // initialize moteus controller
  //——————————————————————————————————————————————————————————————————————————————
    moteus1.Initialize();
    moteus1.options_.id = 1;

    moteus2.Initialize();
    moteus2.options_.id = 2;

    moteus1.SetStop();
    moteus2.SetStop();

    Serial.println("test");

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


  //——————————————————————————————————————————————————————————————————————————————
  // assign/store position/current values
  //——————————————————————————————————————————————————————————————————————————————
  moteus1_lastPosition = moteus1.last_result().values.position*conversion_factor;  // conversion between rev to inches 
  moteus2_lastPosition = moteus2.last_result().values.position*conversion_factor;  // conversion between rev to inches
  moteus1_lastCurrent  = moteus1.last_result().values.q_current;
  moteus2_lastCurrent  = moteus2.last_result().values.q_current; 
  moteus1_lastVelo = moteus1.last_result().values.velocity;
  moteus2_lastVelo = moteus2.last_result().values.velocity;
  moteus1_torq = moteus1.last_result().values.torque;
  moteus2_torq = moteus2.last_result().values.torque;
  moteus1_d_current = moteus1.last_result().values.d_current;
  moteus2_d_current = moteus2.last_result().values.d_current;
  moteus1_temp = moteus1.last_result().values.motor_temperature;
  moteus2_temp = moteus2.last_result().values.motor_temperature;
  moteus1_vol = moteus1.last_result().values.voltage;
  moteus2_vol = moteus2.last_result().values.voltage;

  // TV instance 
  TV tvcommand(TV_list_x[main_loop_counter],TV_list_y[main_loop_counter],TV_list_z[main_loop_counter]);
  
  // Compute new actuator lengths
  tvcommand.actuatorsLength(tvcommand);
  
  // CHECK IF LINK STATE IS STILL ACTIVE 
  // dac.updateLinkState(); 
  // // Check if DAC CONNECTION IS STILL ACTIVE 
  // dac.updateStatus();

  if(dac.getState() == IDLE && states == 1){dac.update(); return;}

  if((dac.getState() == CALIBRATE && (states == 1))&&(dac.getStatus() == CONNECTED)){

    Serial.println("second moteus calibration beginning in 2 seconds");
    delay(2000);

    moteus2_calibration();
    Serial.println("first Moteus calibration beginning in 2 seconds");
    delay(2000);
    Serial.println("------------------------");
    moteus1_calibration();  //moteus 1 calibration

    Serial.println("------------------------");
    moteus1.SetStop();
    moteus2.SetStop();
    
    // finished calibration, go to brake 
    states = 2;
  }

  // We intend to send control frames every 20ms for break and vector
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }
  gNextSendMillis += 20;

  if(dac.getState() == BRAKE || (states == 2)){

    // clear mainloop counter
    main_loop_counter = 0;
    // actuator commands for breaking mode
    act1_forward.position = NaN;
    act1_forward.velocity = 5;
    act1_forward.accel_limit = accel_lim;
  
    act1_backward.position = NaN;
    act1_backward.velocity = -5;
    act1_backward.accel_limit = accel_lim;
    
    act2_forward.position = NaN;
    act2_forward.velocity = 5;
    act2_forward.accel_limit = accel_lim;
  
    act2_backward.position = NaN;
    act2_backward.velocity = -5;
    act2_backward.accel_limit = accel_lim;

    //——————————————————————————————————————————————————————————————————————————————
    // Check deviation state
    //——————————————————————————————————————————————————————————————————————————————  
    if((abs(middle_act1-moteus1_lastPosition)>deviation_zone) || (abs(middle_act2-moteus2_lastPosition)>deviation_zone)){
      deviation = 1;
    }
    else if((abs(middle_act1-moteus1_lastPosition)<=deviation) && (abs(middle_act2-moteus2_lastPosition)<=deviation_zone)) {
      deviation = 0;
    }
    //——————————————————————————————————————————————————————————————————————————————
    // Excecute Brake Command
    //——————————————————————————————————————————————————————————————————————————————
    if(deviation == 1){
      if((abs(moteus1_lastPosition-middle_act1 )<= bubble_zone)){
          m1_commandCompleted = 1;
          moteus1.SetBrake();
        }
      else if((middle_act1 - moteus1_lastPosition )>0){
          m1_commandCompleted = 0;
          moteus1.SetPosition(act1_forward);
        }
      else if((middle_act1 - moteus1_lastPosition )<0){
          m1_commandCompleted = 0;
          moteus1.SetPosition(act1_backward);
        }
        
    
      // checking m2 command
      if((abs(moteus2_lastPosition-middle_act2)<= bubble_zone)){
          m2_commandCompleted = 1;
          moteus2.SetBrake();
        }
      else if((middle_act2 - moteus2_lastPosition)>0){
          m2_commandCompleted = 0;
          moteus2.SetPosition(act2_forward);
        }
      else if((middle_act2 - moteus2_lastPosition )<0){
          m2_commandCompleted = 0;
          moteus2.SetPosition(act2_backward);
        }
    
      // checking both command
      if((m1_commandCompleted == 1) && (m2_commandCompleted == 1)){
        
          m1_commandCompleted = 0;
          m2_commandCompleted = 0;
          deviation = 0;
          ++main_loop_counter;
      }
      else{
        both_commandCompleted = 0;
      }
    }
    
    if(deviation == 0){
        moteus1.SetBrake();
        moteus2.SetBrake();
    }
    dac.updateStatus();
    if (dac.getStatus() == DISC) {
      dac.connect();
    }
    if(dac.update() == true){
      if (dac.getState() == CALIBRATE){
      states = 1; 
      }
      else if (dac.getState() == VECTOR){
        states = 3;
        Serial.println("state is 3");
      }
    } // if there is upate in dac, exit
    else {states = 2;} // otherwise stay in break 
  }
  
  if(dac.getState() == VECTOR && (states == 3 )){

    loop_start_time = millis();

    Serial.println("vector phase");
    states = 3; // you are in vector state
    // listen to new message 
    if(dac.update() == true){
      Serial.println("is DAC really false ?");
      states = 1; 
      return;
      }
    //——————————————————————————————————————————————————————————————————————————————
    //  check if we are close to edge of ball screw
    //——————————————————————————————————————————————————————————————————————————————
    if(at_edge(moteus1_lastPosition,moteus2_lastPosition)){states = 2; return;}
    //——————————————————————————————————————————————————————————————————————————————
    //  Setup position command
    //——————————————————————————————————————————————————————————————————————————————

      Moteus::PositionMode::Command m1_position_cmd;
      Moteus::PositionMode::Command m2_position_cmd;

      Moteus::PositionMode::Command m1_hold_cmd;
      Moteus::PositionMode::Command m2_hold_cmd;
   
      
      //——————————————————————————————————————————————————————————————————————————————
      // moteus 1 command 
      //——————————————————————————————————————————————————————————————————————————————
        m1_position_cmd.position = tvcommand.act1_position/conversion_factor;

        m1_position_cmd.velocity = NaN;
        m1_position_cmd.velocity_limit = velo_lim;
        m1_position_cmd.accel_limit = accel_lim;
        m1_position_cmd.kp_scale = kp_scale_tune;
        m1_position_cmd.kd_scale = kd_scale_tune;

        //brake command

        m1_hold_cmd.position = tvcommand.act1_position/conversion_factor;
        m1_hold_cmd.stop_position = tvcommand.act1_position/conversion_factor;
      //——————————————————————————————————————————————————————————————————————————————
      // moteus 2 command 
      //——————————————————————————————————————————————————————————————————————————————

        m2_position_cmd.position = tvcommand.act2_position/conversion_factor;

        m2_position_cmd.velocity = NaN;
        m2_position_cmd.velocity_limit = velo_lim;
        m2_position_cmd.accel_limit = accel_lim;
        m2_position_cmd.kp_scale = kp_scale_tune;
        m2_position_cmd.kd_scale = kd_scale_tune;

        //brake command

        m2_hold_cmd.position = tvcommand.act2_position/conversion_factor;
        m2_hold_cmd.stop_position = tvcommand.act2_position/conversion_factor;

      //——————————————————————————————————————————————————————————————————————————————
      // Check whether or not command is completed
      //——————————————————————————————————————————————————————————————————————————————

        // checking m1 command
        if((abs(moteus1_lastPosition-tvcommand.act1_position )<= bubble_zone)){
          m1_commandCompleted = 1;
          moteus1.SetPosition(m1_hold_cmd);
          }
        else{
          m1_commandCompleted = 0;
          moteus1.SetPosition(m1_position_cmd);
          Serial.println("trying to command m1");
          }

        // checking m2 command
        if((abs(moteus2_lastPosition-tvcommand.act2_position)<= bubble_zone)){
          m2_commandCompleted = 1;
          moteus2.SetPosition(m2_hold_cmd);
          }
        else{
          m2_commandCompleted = 0;
          moteus2.SetPosition(m2_position_cmd);
          Serial.println("trying to command m2");
          }

        // checking both command
        if((m1_commandCompleted == 1) && (m2_commandCompleted == 1)){
          
            m1_commandCompleted = 0;
            m2_commandCompleted = 0;
            ++main_loop_counter; // advance in vectoring
            Serial.println("advanced");
        }
        else{
          both_commandCompleted = 0;
        }

        if(main_loop_counter == 500){
          Serial.println("trajectory ended");
          //main_loop_counter = 0; // uncomment if you want to rerun
          states = 2;
          return;
        } 

        //——————————————————————————————————————————————————————————————————————————————
// print results 
//——————————————————————————————————————————————————————————————————————————————
  if (gLoopCount % 100 != 0) { return; }
  // Only print our status every 5th cycle, so every 1s.
  Serial.print(F("time "));
  Serial.println(gNextSendMillis);

  Serial.print("moteus 1 position is ");
  Serial.println(moteus1_lastPosition);

  Serial.print("moteus 2 position is ");
  Serial.println(moteus2_lastPosition);
  Serial.println();

  actuator1_record[main_loop_counter] = tvcommand.act1_position;
  actuator2_record[main_loop_counter] = tvcommand.act2_position;

  if(dataFile){
    // dataFile.print("ACK#");
    dataFile.print(loop_start_time);               // Time
    dataFile.print(",");
    dataFile.print(tvcommand.act1_position);       // Actuator 1 commanded position
    dataFile.print(",");
    dataFile.print(moteus1_lastPosition);         // Actuator 1 real position
    dataFile.print(",");
    dataFile.print(moteus1_lastVelo);            // Actuator 1 velocity
    dataFile.print(",");
    dataFile.print(moteus1_torq);                // Actuator 1 torque
    dataFile.print(",");
    dataFile.print(moteus1_lastCurrent);         // Actuator 1 current
    dataFile.print(",");
    dataFile.print(moteus1_d_current);           // Actuator 1 d_current
    dataFile.print(",");
    dataFile.print(moteus1_temp);                // Actuator 1 temperature
    dataFile.print(",");
    dataFile.print(moteus1_vol);                 // Actuator 1 voltage
    dataFile.print(",");
    dataFile.print(tvcommand.act2_position);       // Actuator 2 commanded position
    dataFile.print(",");
    dataFile.print(moteus2_lastPosition);         // Actuator 2 real position
    dataFile.print(",");
    dataFile.print(moteus2_lastVelo);            // Actuator 2 velocity
    dataFile.print(",");
    dataFile.print(moteus2_torq);                // Actuator 2 torque
    dataFile.print(",");
    dataFile.print(moteus2_lastCurrent);         // Actuator 2 current
    dataFile.print(",");
    dataFile.print(moteus2_d_current);           // Actuator 2 d_current
    dataFile.print(",");
    dataFile.print(moteus2_temp);  
    dataFile.print(",");
    dataFile.print(moteus2_vol);                 // Actuator 2 voltage
    // dataFile.print("#");              // Actuator 2 temperature
    dataFile.println();
    dataFile.flush();
  }

//-----------------------------------------------------------------
// print current TV command status 
//-----------------------------------------------------------------

  if(both_commandCompleted == 1){
    Serial.println("-------------moteus 1 and 2 command completed-------------");
  }
  else if(both_commandCompleted == 0){
    Serial.println("---------------------------------------");
    Serial.print("m1 commanding to ");
    Serial.print(tvcommand.act1_position);
    Serial.print(" m2 commanding to ");
    Serial.println(tvcommand.act2_position);
    Serial.println("---------------------------------------");
    Serial.println(main_loop_counter);
    Serial.println("---------------------------------------");


  }


  }

    // CHECK IF LINK STATE IS STILL ACTIVE 
    //dac.updateLinkState(); 

    // IF LINK STATE IS NO LONGER ACTIVE, WAIT FOR INITIALIZATION + CONNECTION
    if (dac.getStatus() == DISC) {

      if((states == 2) || (states == 3)){return;} // if we are disc. and in vector or break, continue

      else{
      Serial.println("ETHERNET CABLE DISCONNECTED, WAITING FOR RECONNECT ...");
      while (!dac.initialize());
      Serial.println("ETHERNET CABLE RECONNECTED!");
      }
    }
  
    // Check if DAC CONNECTION IS STILL ACTIVE 
    dac.updateStatus();
  
    // If DAC CONNECTION IS NO LONGER ACTIVE, WAIT FOR RECONNECTION
    if (dac.getStatus() == DISCONNECTED) {

      if((states == 2) || (states == 3)){return;} // if we are disconnected, and in vector or break, continue
      
      else{
      Serial.println("DAC DISCONNECTED, WAITING FOR RECONNECT ... ");
      while(!dac.connect()); 
      Serial.println("DAC CONNECTED!");
      }
    }

}


void moteus1_calibration() {

  Moteus::PositionMode::Command cmd;
  Moteus::PositionMode::Command zero_cmd;
  Moteus::PositionMode::Command extend_cmd;

  mm::OutputExact::Command calibration_cmd;

  max_current = 0;

  // cmd.accel_limit = 1.5;
  cmd.position = NaN;
  cmd.velocity = 2;
  cmd.kp_scale = 1.2;

  // extend_cmd.accel_limit = 1.5;
  extend_cmd.position = NaN;
  extend_cmd.velocity = 3;
  extend_cmd.kp_scale = 1.6;

  cmd.velocity = -cmd.velocity;

  while (!(max_current > abs(limit_current_m1))) {

    if(abort_sense(moteus1.last_result().values.q_current,moteus2.last_result().values.q_current)){
        Serial.println("program terminated");
        while(true){}
      }

    while (gNextSendMillis >= millis()) {}

    gNextSendMillis += 20;
    moteus1.SetPosition(cmd);
    // moteus2.SetBrake();
    // moteus2.SetPosition(extend_cmd);  // extends to assist the other one 

    if (max_current < abs(moteus1.last_result().values.q_current)) {
      max_current = abs(moteus1.last_result().values.q_current);  // store maximum current recorded
    }
  }

  moteus1.SetStop();

  delay(1000);
  Serial.println("------------------------");
  Serial.print("excessive q_current detected: ");
  Serial.println(abs(max_current));
  Serial.print("current position is: ");
  Serial.println(moteus1.last_result().values.position);
  Serial.println("------------------------");
  Serial.println("Calibration Completed");
  Serial.println("------------------------");

  Serial.println("Changing current position. . . ");
  delay(2000);

  Serial.println("------------------------");
  Serial.print("Current position is : ");
  Serial.println(moteus1.last_result().values.position);
  Serial.println("------------------------");

  //-----------------------------------------------------------------
  // change currentp position to be bottom of LKup table
  //-----------------------------------------------------------------

  calibration_cmd.position = min_act1/conversion_factor;  // this is the smallest actuator length value

  moteus1.SetOutputExact(calibration_cmd);
  
  Serial.println("moteus 1 current psotion is now : ");

  zero_cmd.accel_limit = 0.1;
  zero_cmd.position = calibration_cmd.position;
  zero_cmd.velocity = NaN;

  moteus1.SetPosition(zero_cmd);
  Serial.println(moteus1.last_result().values.position);

//-----------------------------------------------------------------
// new section, going back to it's middle section
//-----------------------------------------------------------------
Serial.println("moteus 1 going back to middle in 1 sec");

while (!((abs(moteus1.last_result().values.position-((middle_act1)/conversion_factor))<=0.04/conversion_factor))){
  if(abort_sense(moteus1.last_result().values.q_current,moteus2.last_result().values.q_current)){
        Serial.println("program terminated");
        while(true){}
      }
  else{
  //Serial.println("check");
  moteus1.SetPosition(extend_cmd);
  moteus2.SetBrake();
      }
  
  }
  moteus1.SetBrake();
  
}

void moteus2_calibration() {

  Moteus::PositionMode::Command cmd;
  Moteus::PositionMode::Command zero_cmd;
  Moteus::PositionMode::Command extend_cmd;

  mm::OutputExact::Command calibration_cmd;

  max_current = 0;

  // cmd.accel_limit = 1.5;
  cmd.position = NaN;
  cmd.velocity = 2;
  cmd.kp_scale = 1.2;

  // extend_cmd.accel_limit = 1.5;
  extend_cmd.position = NaN;
  extend_cmd.velocity = 2;
  extend_cmd.kp_scale = 1.6;
  

  cmd.velocity = -cmd.velocity;

  while (!(max_current > abs(limit_current_m2))) {

    if(abort_sense(moteus1.last_result().values.q_current , moteus2.last_result().values.q_current)){
        Serial.println("program terminated");
        while(true){}
      }

    while (gNextSendMillis >= millis()) {}

    gNextSendMillis += 20;
    moteus2.SetPosition(cmd);
    // moteus1.SetBrake();
    // moteus1.SetPosition(extend_cmd); // extends to assist calibration

    if(gNextSendMillis >= 100){

     if (max_current < abs(moteus2.last_result().values.q_current)) {
      max_current = abs(moteus2.last_result().values.q_current);  // store maximum current recorded
      }

    }
  }

  moteus2.SetStop();
  

  delay(1000);
  Serial.println("------------------------");
  Serial.print("excessive q_current detected: ");
  Serial.println(abs(max_current));
  Serial.print("current position is: ");
  Serial.println(moteus2.last_result().values.position);
  Serial.println("------------------------");
  Serial.println("Calibration Completed");
  Serial.println("------------------------");

  Serial.println("Changing current position. . . ");
  delay(2000);

  Serial.println("------------------------");
  Serial.print("Current position is : ");
  Serial.println(moteus2.last_result().values.position);
  Serial.println("------------------------");

  //-----------------------------------------------------------------
  // change currentp position to be bottom of LKup table
  //-----------------------------------------------------------------

  calibration_cmd.position = min_act2/conversion_factor;  // this is the smallest actuator length value

  moteus2.SetOutputExact(calibration_cmd);
  Serial.println("moteus 2 current position is now : ");

  zero_cmd.accel_limit = 0.1;
  zero_cmd.position = calibration_cmd.position;
  zero_cmd.velocity = NaN;

  moteus2.SetPosition(zero_cmd);
  Serial.println(moteus2.last_result().values.position);


//-----------------------------------------------------------------
// new section, going back to it's middle section
//-----------------------------------------------------------------
Serial.println("moteus 2 going back to middle in 1 sec");

while (!((abs(moteus2.last_result().values.position-((middle_act2)/conversion_factor))<=0.04/conversion_factor))){
  if(abort_sense(moteus1.last_result().values.q_current , moteus2.last_result().values.q_current)){
        Serial.println("program terminated");
        while(true){}
      }
  else{
  moteus2.SetPosition(extend_cmd);
  moteus1.SetBrake();
      }
  
  }
  moteus2.SetBrake();
}

bool abort_sense(double m1_current_sensed, double m2_current_sensed){

  if((m1_current_sensed >= abort_current) || (m2_commandCompleted >= abort_current)){

    Serial.println("abort initiated");

    moteus1.SetStop();
    moteus2.SetStop();
    return true;
  }
  else{

    return false;
  }
}

bool at_edge(double m1_position,double m2_position){

  if((abs(m1_position - min_act1) <= 0.05)||(abs(m1_position - max_act1) <= 0.05)){
    Serial.println("m1 edge detected");
    return true;
  }
  else if((abs(m2_position - min_act2) <= 0.05)||(abs(m2_position - max_act2) <= 0.05)){
    Serial.println("m2 edge detected");
    return true;
  }
  else {
    return false;
  }

}