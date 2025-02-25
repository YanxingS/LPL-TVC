#ifndef __IMXRT1062__
#error "This sketch should be compiled for Teensy 4.x"
#endif


#include <ACAN_T4.h>
#include "Moteus.h"
#include <arrays.h>


//——————————————————————————————————————————————————————————————————————————————
//  The following defines our thrust vector list, actuator lengths, and lookup table from arrays.h
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

//-----------------------------------------------------------------
// Declaration of functions
//-----------------------------------------------------------------
void moteus1_calibration();
void moteus2_calibration();
double find_z_section(int y_int_upper, int y_int_lower);
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
static double abort_current = 6.0 ;           // current which will cause abort  
static double kp_scale_tune = 0.8;
static double kd_scale_tune = 0.5;
static double accel_lim = 6.5;
static double velo_lim = 9; 
static double bubble_zone = 0.05;

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
  Serial.println("Braking starting in 2 seconds");
  Serial.println("---------------------------------------");

  // Serial.println("freezed");

  // Serial.println(moteus1.last_result().values.position);
  // Serial.println(moteus2.last_result().values.position);
  // while (true){}
  

  delay(2000);

}


void loop() {

//——————————————————————————————————————————————————————————————————————————————
// abort by space checker 
//——————————————————————————————————————————————————————————————————————————————
  abort_by_space();

//——————————————————————————————————————————————————————————————————————————————
// assign critical values 
//——————————————————————————————————————————————————————————————————————————————
  moteus1_lastPosition = moteus1.last_result().values.position*conversion_factor;  // conversion between rev to inches 
  moteus2_lastPosition = moteus2.last_result().values.position*conversion_factor;  // conversion between rev to inches
  moteus1_lastCurrent  = moteus1.last_result().values.q_current;
  moteus2_lastCurrent  = moteus2.last_result().values.q_current;

  const auto time = millis();
  if (gNextSendMillis >= time) { return; }
  
  gNextSendMillis += 20;

//——————————————————————————————————————————————————————————————————————————————
// Excecute Brake Command
//——————————————————————————————————————————————————————————————————————————————

moteus1.SetBrake();
moteus2.SetBrake();

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

  Serial.println("-------------actuators Braking-------------");

 
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

while (!((abs(moteus1.last_result().values.position-((17.25)/conversion_factor))<=0.04/conversion_factor))){
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

while (!((abs(moteus2.last_result().values.position-((17.25)/conversion_factor))<=0.04/conversion_factor))){
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

double find_z_section(int y_int_upper, int y_int_lower){
    
    double middle_value = (double)((y_int_upper+y_int_lower)/2);
    int low_index = 0;
    int high_index = 1;
    
    while(middle_value > z_sections[low_index]){
        
        if((middle_value>z_sections[low_index])&&(middle_value<z_sections[high_index])){
            break;
        }
        else
        low_index  += 1;
        high_index += 1;
    }
    
    return z_sections[low_index];
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