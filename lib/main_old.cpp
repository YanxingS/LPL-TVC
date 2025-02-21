#ifndef __IMXRT1062__
#error "This sketch should be compiled for Teensy 4.x"
#endif


#include <ACAN_T4.h>
#include "Moteus.h"
#include <arrays.h>


//——————————————————————————————————————————————————————————————————————————————
//  The following defines our thrust vector list
//——————————————————————————————————————————————————————————————————————————————


//——————————————————————————————————————————————————————————————————————————————
//  The following defines y-z -> l1 l2 lookup table
//——————————————————————————————————————————————————————————————————————————————

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


//-----------------------------------------------------------------
// Declaration of functions
//-----------------------------------------------------------------
void moteus1_calibration();
void moteus2_calibration();
double find_z_section(int y_int_upper, int y_int_lower);
bool abort_sense(double m1_position,double m2_position);
bool at_edge(double m1_position,double m2_position);

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

  //——————————————————————————————————————————————————————————————————————————————
  //  Define searching algo
  //——————————————————————————————————————————————————————————————————————————————
    int loop_count_y = 0;
    int loop_count_z = 0;
    int y_int_lower_index = 0;
    int y_int_upper_index = increase_index;
    int z_int_lower_index = 0;
    int z_int_upper_index = 0;
    
// -----------------------------------------------------------------------------------------------
    
    // this section loop searches for the section of y-int and records its lower and upper index value
    while (desire_y_int >= y_int[y_int_lower_index]){
        
//        printf("testtttt %d \n ", z_int_lower_index);
       
        //-----------------------------------------------------------------------------------------------
        // when we are below middle section
        //-----------------------------------------------------------------------------------------------
        if(loop_count_y < middle_search){
            
            if((desire_y_int>=y_int[y_int_lower_index]) && (desire_y_int<y_int[y_int_upper_index])){
                
                //            double test_var = abs(desire_y_int-y_int[y_int_upper_index]);
                z_int_lower_index = (int)(find_z_section(y_int_upper_index, y_int_lower_index));
                z_int_upper_index = z_int_lower_index + 1;
                //            printf("abs(desire_y_int-y_int[y_int_upper_index]) = %f \n", test_var);
//                printf("we are below 312\n");
                
                break;
                
                
            }
        }
        //-----------------------------------------------------------------------------------------------
        // when we are above middle section
        //-----------------------------------------------------------------------------------------------
        if(loop_count_y > middle_search){
            
            if((desire_y_int>=y_int[y_int_lower_index]) && (desire_y_int<y_int[y_int_upper_index])){
                
                y_int_lower_index += increase_index;
                y_int_upper_index += increase_index;
                z_int_lower_index = (int)(find_z_section(y_int_upper_index, y_int_lower_index));
                z_int_upper_index = z_int_lower_index + 1;
                //            printf("abs(desire_y_int-y_int[y_int_upper_index]) = %f \n", test_var);
//                printf("we are above 321\n");
                
                break;
                
            }
            
        }
        //-----------------------------------------------------------------------------------------------
        // special case : when we are at middle section
        //-----------------------------------------------------------------------------------------------
        if (loop_count_y == middle_search){
            double small_deviation = 0.001;
            if(((y_int[y_int_upper_index]-small_deviation)<desire_y_int)&& (y_int[y_int_upper_index]+small_deviation)>desire_y_int){
                y_int_lower_index += increase_index;
                y_int_upper_index += increase_index;
                z_int_lower_index = (int)(find_z_section(y_int_upper_index, y_int_lower_index));
                z_int_upper_index = z_int_lower_index + 1;
                //            printf("abs(desire_y_int-y_int[y_int_upper_index]) = %f \n", test_var);
//                printf("we are above 312\n");
                
                break;
            }
            
            if(((y_int[y_int_lower_index]-small_deviation)<desire_y_int)&& (y_int[y_int_lower_index]+small_deviation)>desire_y_int){
                
                z_int_lower_index = (int)(find_z_section(y_int_upper_index, y_int_lower_index));
                z_int_upper_index = z_int_lower_index + 1;
//                printf("we are below 312\n");
                
                break;
            }
        }
        
        y_int_lower_index += increase_index;
        y_int_upper_index += increase_index;
        ++loop_count_y;
//        printf("from %d to %d and loop count is %d \n",y_int_lower_index,y_int_upper_index,loop_count_y);
        
    }
//----------------------------------128-------------------------------------------------------------
    
    // this section uses the recorded lower and uppder index to search for z-int
    
    while (desire_z_int < z_int[z_int_lower_index]){
        
        if(desire_z_int > z_int[z_int_lower_index+1]){
//            printf("not suppose to run this part\n");
            break;
        }
        
        ++z_int_lower_index;
        ++z_int_upper_index;
        ++loop_count_z;
    }

    act1_position = (act1_length[z_int_upper_index] - act1_length[z_int_lower_index]) / 2 + act1_length[z_int_lower_index];
    act2_position = (act2_length[z_int_upper_index] - act2_length[z_int_lower_index]) / 2 + act2_length[z_int_lower_index];

  }

};

//——————————————————————————————————————————————————————————————————————————————
//  The following defines two moteus controller object and one TV class object
//——————————————————————————————————————————————————————————————————————————————
Moteus moteus1;
Moteus moteus2;

Moteus::PositionMode::Command m1_position_cmd;
Moteus::PositionMode::Command m2_position_cmd;

//-----------------------------------------------------------------
// define some constants
//-----------------------------------------------------------------
static double moteus2_lastPosition;
static double moteus1_lastPosition;
static double moteus1_lastCurrent;
static double moteus2_lastCurrent;
// static double commanded_position;     // testing, this variable is manually set
static int m1_commandCompleted = 0;
static int m2_commandCompleted = 0;
static int both_commandCompleted = 0;
static uint32_t gSendNextCommand = 0; 
static uint32_t gNextSendMillis = 0;  // adds 20ms everyloop to ensure we send command every 20 seconds
uint16_t gLoopCount = 0;              // loop counter
static float max_current = 0;         // max current/resistance we observed for motor to experienced
static float limit_current_m1 = 12.0;     // max current/resistance we allow for motor to experienced for moteus 1
static float limit_current_m2 = 12.0;     // max current/resistance we allow for motor to experienced for moteus 2
static int main_loop_counter = 0;     // main loop counter, used to advance in TV list
static double abs_speed = 4;
//-----------------------------------------------------------------
// critical limit constant 
//-----------------------------------------------------------------

static int traj_length = 2401;
static double min_act1 = 15.3125;             // updated min length
static double max_act1 = 19.1622;
static double min_act2 = 15.7419;
static double max_act2 = 19.1622;

static double abort_current = 6.0 ;           // current which will cause abort  

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

//——————————————————————————————————————————————————————————————————————————————
// Check for end of trajectory list
//——————————————————————————————————————————————————————————————————————————————

if(main_loop_counter == traj_length){
  Serial.println("Trajectory ended");
  moteus1.SetStop();
  moteus2.SetStop();
}
//——————————————————————————————————————————————————————————————————————————————
// assign critical values 
//——————————————————————————————————————————————————————————————————————————————
  moteus1_lastPosition = moteus1.last_result().values.position*0.157;  // conversion between rev to inches 
  moteus2_lastPosition = moteus2.last_result().values.position*0.157;  // conversion between rev to inches
  moteus1_lastCurrent  = moteus1.last_result().values.q_current;
  moteus2_lastCurrent  = moteus2.last_result().values.q_current;

//——————————————————————————————————————————————————————————————————————————————
// Check for abort condition
//——————————————————————————————————————————————————————————————————————————————

// if(abort_sense(moteus1_lastCurrent,moteus2_lastCurrent)){
//   Serial.println("program terminated");
//   while(true){}
// }

  TV tvcommand(TV_list_x[main_loop_counter],TV_list_y[main_loop_counter],TV_list_z[main_loop_counter]);
  
  tvcommand.actuatorsLength(tvcommand);
  
  // We intend to send control frames every 20ms.

  const auto time = millis();
  if (gNextSendMillis >= time) { return; }
  
  gNextSendMillis += 20;
  gSendNextCommand = gNextSendMillis;
  gLoopCount++;


  //——————————————————————————————————————————————————————————————————————————————
  //  Setup position command
  //——————————————————————————————————————————————————————————————————————————————

  Moteus::PositionMode::Command m1_position_cmd;
  Moteus::PositionMode::Command m2_position_cmd;

//——————————————————————————————————————————————————————————————————————————————
// moteus 1 command 
//——————————————————————————————————————————————————————————————————————————————

  m1_position_cmd.position = NaN;

  if(moteus1_lastPosition<tvcommand.act1_position){
    m1_position_cmd.velocity = abs_speed;
  }
  else{
    m1_position_cmd.velocity = -abs_speed;
    if (gLoopCount % 100 == 0) { 
    Serial.println("m1 spinning back !");
    }
  }

  m1_position_cmd.accel_limit = 0.1;

//——————————————————————————————————————————————————————————————————————————————
// moteus 2 command 
//——————————————————————————————————————————————————————————————————————————————

  m2_position_cmd.position = NaN;

  if(moteus2_lastPosition < tvcommand.act2_position){
    m2_position_cmd.velocity = abs_speed;
  }
  else{
    m2_position_cmd.velocity = -abs_speed;
    if (gLoopCount % 100 == 0) { 
    Serial.println("m2 spinning back !");
    }
  }

  m2_position_cmd.accel_limit = 0.1;


//——————————————————————————————————————————————————————————————————————————————
// Check whether or not command is completed
//——————————————————————————————————————————————————————————————————————————————
Serial.println(main_loop_counter);

// checking m1 command
if((abs(moteus1_lastPosition-tvcommand.act1_position)<=0.03)){
  m1_commandCompleted = 1;
  moteus1.SetStop();
  }
else{
  m1_commandCompleted = 0;
  moteus1.SetPosition(m1_position_cmd);
  }

// checking m2 command
if((abs(moteus2_lastPosition-tvcommand.act2_position)<=0.03)){
  m2_commandCompleted = 1;
  moteus2.SetStop();
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
// check command completion and advance in TV sequence
//——————————————————————————————————————————————————————————————————————————————

  // if(both_commandCompleted == 1){
  //   both_commandCompleted = 0;
  //   m1_commandCompleted = 0;
  //   m2_commandCompleted = 0;
  //   ++main_loop_counter;
  // }



//——————————————————————————————————————————————————————————————————————————————
// print results 
//——————————————————————————————————————————————————————————————————————————————
  if (gLoopCount % 100 != 0) { return; }

  // Only print our status every 5th cycle, so every 1s.
  Serial.print(F("time "));
  Serial.println(gNextSendMillis);

  // auto print_moteus = [](const Moteus::Query::Result& query) {
  //   Serial.print(static_cast<int>(query.mode));
  //   Serial.print(F(" "));
  //   Serial.print(query.position);
  //   Serial.print(F("  velocity "));
  //   Serial.print(query.velocity);
  // };

  // print_moteus(moteus1.last_result().values);
  Serial.print("moteus 1 position is ");
  Serial.println(moteus1_lastPosition);

  Serial.print("moteus 2 position is ");
  Serial.println(moteus2_lastPosition);
  Serial.println();

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

  cmd.accel_limit = 0.1;
  cmd.position = NaN;
  cmd.velocity = 2;

  extend_cmd.accel_limit = 0.1;
  extend_cmd.position = NaN;
  extend_cmd.velocity = 2;

  cmd.velocity = -cmd.velocity;

  while (!(max_current > abs(limit_current_m1))) {

    if(abort_sense(moteus1.last_result().values.q_current,moteus2.last_result().values.q_current)){
        Serial.println("program terminated");
        while(true){}
      }

    while (gNextSendMillis >= millis()) {}

    gNextSendMillis += 20;
    moteus1.SetPosition(cmd);
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

  calibration_cmd.position = min_act1/0.157;  // this is the smallest actuator length value

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

while (!((abs(moteus1.last_result().values.position-((17.8975+0.04)/0.157))<=0.04))){
  if(abort_sense(moteus1.last_result().values.q_current,moteus2.last_result().values.q_current)){
        Serial.println("program terminated");
        while(true){}
      }
  else{
  Serial.println("check");
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

  cmd.accel_limit = 0.1;
  cmd.position = NaN;
  cmd.velocity = 2;

  extend_cmd.accel_limit = 0.1;
  extend_cmd.position = NaN;
  extend_cmd.velocity = 2;
  

  cmd.velocity = -cmd.velocity;

  while (!(max_current > abs(limit_current_m2))) {

    if(abort_sense(moteus1.last_result().values.q_current , moteus2.last_result().values.q_current)){
        Serial.println("program terminated");
        while(true){}
      }

    while (gNextSendMillis >= millis()) {}

    gNextSendMillis += 20;
    moteus2.SetPosition(cmd);
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

  calibration_cmd.position = min_act2/0.157;  // this is the smallest actuator length value

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

while (!((abs(moteus2.last_result().values.position-((17.8975+0.04)/0.157))<=0.04))){
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