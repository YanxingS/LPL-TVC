#ifndef __IMXRT1062__
#error "This sketch should be compiled for Teensy 4.x"
#endif

#include <Arduino.h>
#include <ACAN_T4.h>
#include "Moteus.h"
#include <arrays.h>

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
    act1_position = 17.1956 + 4.2100 * x + 4.2303 * y - 1.9297 * x * x - 0.1368 * x * y - 1.9375 * y * y;
    act2_position = 17.1956 + (-4.2100) * x + 4.2303 * y + (-1.9297) * x * x + 0.1368 * x * y + (-1.9375) * y * y;
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
// static double commanded_position;     // testing, this variable is manually set
static int m1_commandCompleted = 0;   // m1 command completion check
static int m2_commandCompleted = 0;   // m2 command completion check
static int both_commandCompleted = 0;
// static uint32_t gSendNextCommand = 0; 
static uint32_t gNextSendMillis = 0;  // adds 20ms everyloop to ensure we send command every 20 seconds
uint16_t gLoopCount = 0;              // loop counter
static float max_current = 0;         // max current/resistance we observed for motor to experienced
static float limit_current_m1 = 12.0;     // max current/resistance we allow for motor to experienced for moteus 1
static float limit_current_m2 = 12.0;     // max current/resistance we allow for motor to experienced for moteus 2
static int main_loop_counter = 0;     // main loop counter, used to advance in TV list

//——————————————————————————————————————————————————————————————————————————————
// critical constants
//——————————————————————————————————————————————————————————————————————————————
static int traj_length = vector_size;
static double min_act1 = 15.3125;             // updated min length
static double max_act1 = 19.1622;             
static double min_act2 = 15.7419;
static double max_act2 = 19.1622;
static double middle_act1 = 17.20;            // actuator 1 zero position
static double middle_act2 = 17.40;            // actuator 2 zero position
static double abort_current = 6.0 ;           // current which will cause abort  
static double kp_scale_tune = 0.8;
static double kd_scale_tune = 0.5;
static double accel_lim = 6.5;
static double velo_lim = 9; 
static double bubble_zone = 0.05;
static double deviation_zone = 0.1;           // zone which we determine whether we deviated from desire zone

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
  while (!Serial) {};
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
  
  //——————————————————————————————————————————————————————————————————————————————
  // initialize moteus controller
  //——————————————————————————————————————————————————————————————————————————————
    moteus1.Initialize();
    moteus1.options_.id = 1;

    moteus2.Initialize();
    moteus2.options_.id = 2;

    moteus1.SetStop();
    moteus2.SetStop();

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

  // TV instance 
  TV tvcommand(TV_list_x[main_loop_counter],TV_list_y[main_loop_counter],TV_list_z[main_loop_counter]);
  
  // Compute new actuator lengths
  tvcommand.actuatorsLength(tvcommand);

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
  
  if(dac.getState() == CALIBRATE){

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
  }

  // We intend to send control frames every 20ms.
  const auto time = millis();
  if (gNextSendMillis >= time) { return; }
  gNextSendMillis += 20;

  if(dac.getState() == BRAKE || (main_loop_counter = traj_length)){

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
  }
  
  if(dac.getState() == VECTOR){

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
          }

        // checking m2 command
        if((abs(moteus2_lastPosition-tvcommand.act2_position)<= bubble_zone)){
          m2_commandCompleted = 1;
          moteus2.SetPosition(m2_hold_cmd);
          }
        else{
          m2_commandCompleted = 0;
          moteus2.SetPosition(m2_position_cmd);
          }

        // checking both command
        if((m1_commandCompleted == 1) && (m2_commandCompleted == 1)){
          
            m1_commandCompleted = 0;
            m2_commandCompleted = 0;
            ++main_loop_counter;
        }
        else{
          both_commandCompleted = 0;
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
  extend_cmd.velocity = 2;
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
      }
  
  }
  moteus1.SetStop();
  
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
      }
  
  }
  moteus2.SetStop();
}