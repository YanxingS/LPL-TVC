#ifndef __IMXRT1062__
#error "This sketch should be compiled for Teensy 4.x"
#endif


#include <ACAN_T4.h>
#include "Moteus.h"
#include <arrays.h>
#include <SD.h>
#include <SPI.h>


//——————————————————————————————————————————————————————————————————————————————
//  The following defines our thrust vector list, actuator lengths, and lookup table from arrays.h
//——————————————————————————————————————————————————————————————————————————————

double actuator1_record[1000];
double actuator2_record[1000];

int vector_size = sizeof(TV_list_y) / sizeof(TV_list_y[0]); // Correct for C-style arrays
int increase_index = sqrt(vector_size);             // this is the number of indices between each intercent step
int middle_search = (increase_index - 1)/2;

static double conversion_factor = 0.1575;

//-----------------------------------------------------------------
// Declaration of functions
//-----------------------------------------------------------------
void moteus1_calibration();
void moteus2_calibration();
bool abort_sense(double m1_position,double m2_position);
bool at_edge(double m1_position,double m2_position);
void abort_by_space();

//-----------------------------------------------------------------
// TV class
//-----------------------------------------------------------------

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
//  The following defines two moteus controller object and one TV class object
//——————————————————————————————————————————————————————————————————————————————
Moteus moteus1;
Moteus moteus2;

// Moteus::PositionMode::Command m1_position_cmd;
// Moteus::PositionMode::Command m2_position_cmd;

//-----------------------------------------------------------------
// define some constants
//-----------------------------------------------------------------
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
// static double abs_speed = 4;
//-----------------------------------------------------------------
// critical limit constant that we modify
//-----------------------------------------------------------------

static int traj_length = vector_size;
static double min_act1 = 15.3125;             // updated min length
static double max_act1 = 19.1622;             
static double min_act2 = 15.7419;
static double max_act2 = 19.1622;
static double middle_act1 = 17.00;            // actuator 1 zero position
static double middle_act2 = 17.40;            // actuator 2 zero position
static double abort_current = 6.0 ;           // current which will cause abort  
static double kp_scale_tune = 0.8;
static double kd_scale_tune = 0.5;
static double accel_lim = 6.5;
static double velo_lim = 9; 
static double bubble_zone = 0.05;
static double deviation_zone = 0.1;           // zone which we determine whether we deviated from desire zone
static double req_time = 10000;
static int start_flag = 0;
static uint32_t start_time;
//-----------------------------------------------------------------
// braking hotfire specific constants and commands
//-----------------------------------------------------------------
static boolean deviation = 0;
Moteus::PositionMode::Command act1_forward;
Moteus::PositionMode::Command act2_forward;
Moteus::PositionMode::Command act1_backward;
Moteus::PositionMode::Command act2_backward;
//-----------------------------------------------------------------
// setup function
//-----------------------------------------------------------------

void setup() {
  // Let the world know we have begun!

  //-----------------------------------------------------------------
  // pre-defined setup procedure
  //------------------------------------------33-----------------------
  delay(5000);
  Serial.begin(115200);
  // while (!Serial) {}
  Serial.println(F("started"));

  ACAN_T4FD_Settings settings(1000000, DataBitRateFactor::x1);

  const uint32_t errorCode = ACAN_T4::can3.beginFD(settings);
  if (0 == errorCode) {
    Serial.println("can3 ok");
  } else {
    Serial.print("Error can3: 0x");
    Serial.println(errorCode, HEX);
  }
  //-----------------------------------------------------------------
  // moteus 1 and 2 initilization phase
  //-----------------------------------------------------------------
  moteus1.Initialize();
  moteus1.options_.id = 1;

  moteus2.Initialize();
  moteus2.options_.id = 2;

  moteus1.SetStop();
  moteus2.SetStop();
  Serial.println(F("all stopped"));
  
  //-----------------------------------------------------------------
  // add calibration start command here
  //-----------------------------------------------------------------
  
  Serial.println("calibration beginning in 2 seconds");
  delay(2000);

  
  moteus2_calibration();
  Serial.println("first Moteus calibration beginning in 2 seconds");
  Serial.println("------------------------");
  moteus1_calibration();  //moteus 1 calibration


  Serial.println("Both Moteus calibrated");
  moteus1.SetStop();
  moteus2.SetStop();

  Serial.println("---------------------------------------");
  Serial.println("Sequencing starting in 2 seconds");
  Serial.println("---------------------------------------");

  // Serial.println("freezed");

  // Serial.println(moteus1.last_result().values.position);
  // Serial.println(moteus2.last_result().values.position);
  // while (true){}
  

  delay(2000);

}


void loop() {

  abort_by_space();

//——————————————————————————————————————————————————————————————————————————————
// assign critical values 
//——————————————————————————————————————————————————————————————————————————————
  moteus1_lastPosition = moteus1.last_result().values.position*conversion_factor;  // conversion between rev to inches 
  moteus2_lastPosition = moteus2.last_result().values.position*conversion_factor;  // conversion between rev to inches
  moteus1_lastCurrent  = moteus1.last_result().values.q_current;
  moteus2_lastCurrent  = moteus2.last_result().values.q_current;

//——————————————————————————————————————————————————————————————————————————————
// Check for abort condition
//——————————————————————————————————————————————————————————————————————————————

// if(abort_sense(moteus1_lastCurrent,moteus2_lastCurrent)){
//   Serial.println("program terminated");
//   while(true){}
// }

//——————————————————————————————————————————————————————————————————————————————
// Check whether or not command is completed
//—————————————————————————————————————————————————————————————————————————————— 
  TV tvcommand(TV_list_x[main_loop_counter],TV_list_y[main_loop_counter],TV_list_z[main_loop_counter]);
  
  tvcommand.actuatorsLength(tvcommand);
  
  // We intend to send control frames every 20ms.

  const auto time = millis();
  
  if (gNextSendMillis >= time) { return; }
  
  gNextSendMillis += 20;

//——————————————————————————————————————————————————————————————————————————————
//sequence time
//——————————————————————————————————————————————————————————————————————————————
if(start_flag == 0){
  start_time = millis();
  start_flag = 1;
}

uint32_t elapsed_time = millis() - start_time;



//——————————————————————————————————————————————————————————————————————————————
// Check for end of trajectory list, if ended, go to neutral and brake
//——————————————————————————————————————————————————————————————————————————————

  if(main_loop_counter == traj_length || elapsed_time >= req_time){
    
  
    
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
      if(deviation == 0){
        Serial.println("-------------actuators Braking-------------");
      }

      if(deviation == 1){
        Serial.print(("-------------Deviation detected, returning "));
        Serial.print("act1 to ");
        Serial.print(middle_act1);
        Serial.print(" act2 to ");
        Serial.print(middle_act2);
        Serial.println("-------------");
      }
  }

  
  if(main_loop_counter == traj_length || req_time <= elapsed_time){return;} // if we went through all the traj, go back to top

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


//——————————————————————————————————————————————————————————————————————————————
// print results 
//——————————————————————————————————————————————————————————————————————————————
  if (gLoopCount % 100 != 0) { return; }
  // Only print our status every 5th cycle, so every 1s.
  Serial.print(F("time "));
  Serial.println(gNextSendMillis);

  Serial.print(F("elapsed time"));
  Serial.println(elapsed_time);

  Serial.print("moteus 1 position is ");
  Serial.println(moteus1_lastPosition);

  Serial.print("moteus 2 position is ");
  Serial.println(moteus2_lastPosition);
  Serial.println();

  actuator1_record[main_loop_counter] = tvcommand.act1_position;
  actuator2_record[main_loop_counter] = tvcommand.act2_position;

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

//-----------------------------------------------------------------
// moteus 1 calibration function, sets bottom position as 8
//-----------------------------------------------------------------
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
//-----------------------------------------------------------------
// Declaration of a function that finds corresponding z-intercept section given a upper and lower index
//-----------------------------------------------------------------

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

  if((abs(m1_position - min_act1) <= 0.01)||(abs(m1_position - max_act1) <= 0.01)){
    Serial.println("m1 edge detected");
    return true;
  }
  else if((abs(m2_position - min_act2) <= 0.01)||(abs(m2_position - max_act2) <= 0.01)){
    Serial.println("m2 edge detected");
    return true;
  }
  else {
    return false;
  }

}

void abort_by_space(){
  if(Serial.available() > 0){
    char input = Serial.read();
    if(input == ' '){
      Serial.println("Entered Space, Program Terminated");
      moteus1.SetStop();
      moteus2.SetStop();
      while(true){}
    }
  }
}
