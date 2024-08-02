#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <Encoder.h> 

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <PID_v1.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t subscriber;
rcl_publisher_t fl_publisher; // front left wheel ticks publisher
rcl_publisher_t fr_publisher; // front right wheel ticks publisher
rcl_publisher_t bl_publisher; // back left wheel ticks publisher
rcl_publisher_t br_publisher; // back right wheel ticks publisher

std_msgs__msg__Int32 fl_count;
std_msgs__msg__Int32 fr_count;
std_msgs__msg__Int32 bl_count;
std_msgs__msg__Int32 br_count;

geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

// Package Params
bool isHolonomic = true;

// Initialize PID paramaters
double Setpoint_fl, Input_fl, Output_fl;
double Setpoint_fr, Input_fr, Output_fr;
double Setpoint_bl, Input_bl, Output_bl;
double Setpoint_br, Input_br, Output_br;

double aggKp=450, aggKi=250, aggKd=4;
double consKp=280, consKi=120, consKd=1;

PID myPID_fl(&Input_fl, &Output_fl, &Setpoint_fl, aggKp, aggKi, aggKd, DIRECT);
PID myPID_fr(&Input_fr, &Output_fr, &Setpoint_fr, aggKp, aggKi, aggKd, DIRECT);
PID myPID_bl(&Input_bl, &Output_bl, &Setpoint_bl, aggKp, aggKi, aggKd, DIRECT);
PID myPID_br(&Input_br, &Output_br, &Setpoint_br, aggKp, aggKi, aggKd, DIRECT);


Encoder encoder_fleft(12,13); // encoder pins for front left motor A/B  these are assigned reverse because the left motors need to be reverse
Encoder encoder_bleft(8,9);  // encoder pins for back left motor A/B  these are assigned reverse because the left motors need to be reverse
Encoder encoder_fright(11,10);  // encoder pins for front right motor A/B
Encoder encoder_bright(15,14);  // encoder pins for back right motor A/B

const uint8_t LF_PWM = 2; // 
const uint8_t LF_FORW = 4; // the forward and backward pins are assigned reverse as the left motors rotate in reverse direction to right motors  
const uint8_t LF_BACK = 3; // the forward and backward pins are assigned reverse as the left motors rotate in reverse direction to right motors

const uint8_t LB_PWM = 7; //   
const uint8_t LB_FORW = 6; // the forward and backward pins are assigned reverse as the left motors rotate in reverse direction to right motors
const uint8_t LB_BACK = 5; //  the forward and backward pins are assigned reverse as the left motors rotate in reverse direction to right motors

const uint8_t RF_PWM = 21; // 
const uint8_t RF_FORW = 20; // 
const uint8_t RF_BACK = 19; //  

const uint8_t RB_PWM = 16; // 
const uint8_t RB_FORW = 18; // 
const uint8_t RB_BACK = 17; // 

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data++;
//   }
// }

bool wtf;
int ticks_since_target = 0;

int a = 0;


void subscription_callback(const void *msgin) 
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float x = msg->linear.x;
  float y = msg->linear.y;
  float z = msg->angular.z;

  // setting max velocities
  if(x > 0.7){x = 0.7;}
  if(y > 0.7){y = 0.7;}
  if(z > 0.5){z = 0.5;}
  float w = 0.2;

  float R = 0.04;
  float L1 = 0.1625; // 16.5 // 0.105
  float L2 = 0.140; // 14 // 0.0825
  float L = (L1+L2);   
  if (!isHolonomic){
    if(!(x==0 && z==0)){
    wtf=false;
      Setpoint_fr = x + (z * w / 2.0)/0.1;
      Setpoint_fl = x - (z * w / 2.0)/0.1;
      Setpoint_br = x + (z * w / 2.0)/0.1;
      Setpoint_bl = x - (z * w / 2.0)/0.1;
    }
    else{
      wtf=true;
      Setpoint_fl = 0;
      Setpoint_fr = 0;
      Setpoint_bl = 0;
      Setpoint_br = 0;
      
    }
  }
  else{
    if(!(x==0 && y==0 && z==0)){
    wtf=false;
      Setpoint_fl = (1/R)*(x - y - L * z);
      Setpoint_fr = (1/R)*(x + y + L * z);
      Setpoint_bl = (1/R)*(x + y - L * z);
      Setpoint_br = (1/R)*(x - y + L * z);
      
    }
    else{
      wtf=true;
      Setpoint_fl = 0;
      Setpoint_fr = 0;
      Setpoint_bl = 0;
      Setpoint_br = 0;
      
    }
  }
}


// Motor Control function
void Move_motor(int speed_pwm,const uint8_t pwm,const uint8_t forw,const uint8_t back)
{
  if(speed_pwm >= 0)
  {
    digitalWrite(forw, HIGH);
    digitalWrite(back, LOW);
    analogWrite(pwm, abs(speed_pwm));
  }
  else if(speed_pwm < 0)
  {
    digitalWrite(forw, LOW);
    digitalWrite(back, HIGH);
    analogWrite(pwm, abs(speed_pwm));
  }
}

void setpins()
{
  pinMode(LF_FORW,OUTPUT);
  pinMode(LF_BACK,OUTPUT);
  pinMode(RF_FORW,OUTPUT);
  pinMode(RF_BACK,OUTPUT);
  pinMode(LF_PWM,OUTPUT);
  pinMode(RF_PWM,OUTPUT);
  pinMode(LB_FORW,OUTPUT);
  pinMode(LB_BACK,OUTPUT);
  pinMode(RB_FORW,OUTPUT);
  pinMode(RB_BACK,OUTPUT);
  pinMode(LB_PWM,OUTPUT);
  pinMode(RB_PWM,OUTPUT);

  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
}

//void reset Integral error when we stop
void reset_pid_Ki()
{
  myPID_fl.SetMode(MANUAL);
  myPID_fr.SetMode(MANUAL);
  myPID_bl.SetMode(MANUAL);
  myPID_br.SetMode(MANUAL);
  Output_fl=0;
  Output_fr=0;
  Output_bl=0;
  Output_br=0;
  //myPID_fl.SetTunings(aggKp, 0, aggKd);
  //myPID_fr.SetTunings(aggKp, 0, aggKd);
  //myPID_bl.SetTunings(aggKp, 0, aggKd);
  //myPID_br.SetTunings(aggKp, 0, aggKd);
  myPID_fl.SetMode(AUTOMATIC);
  myPID_fr.SetMode(AUTOMATIC);
  myPID_bl.SetMode(AUTOMATIC);
  myPID_br.SetMode(AUTOMATIC);
  //myPID_fl.SetTunings(aggKp, aggKi, aggKd);
  //myPID_fr.SetTunings(aggKp, aggKi, aggKd);
  //myPID_bl.SetTunings(aggKp, aggKi, aggKd);
  //myPID_br.SetTunings(aggKp, aggKi, aggKd);
}

// stop movement
void stop()
{
  digitalWrite(LF_FORW, 0);
  digitalWrite(LF_BACK, 0);
  digitalWrite(RF_FORW, 0);
  digitalWrite(RF_BACK, 0);
  analogWrite(LF_PWM, 0);
  analogWrite(RF_PWM, 0);
  digitalWrite(LB_FORW, 0);
  digitalWrite(LB_BACK, 0);
  digitalWrite(RB_FORW, 0);
  digitalWrite(RB_BACK, 0);
  analogWrite(LB_PWM, 0);
  analogWrite(RB_PWM, 0);
}

void setup() 
{
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);  

  // Pid setup  
  myPID_fl.SetOutputLimits(-255, 255);
  myPID_fr.SetOutputLimits(-255, 255);
  myPID_bl.SetOutputLimits(-255, 255);
  myPID_br.SetOutputLimits(-255, 255);

  myPID_fl.SetMode(AUTOMATIC);
  myPID_fr.SetMode(AUTOMATIC);
  myPID_bl.SetMode(AUTOMATIC);
  myPID_br.SetMode(AUTOMATIC);

  myPID_fl.SetSampleTime(20);
  myPID_fr.SetSampleTime(20);
  myPID_bl.SetSampleTime(20);
  myPID_br.SetSampleTime(20);
  
  setpins();

  stop();
  // delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create publisher front left
  RCCHECK(rclc_publisher_init_default(
    &fl_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "fl_count"));

  // create publisher front right
  RCCHECK(rclc_publisher_init_default(
    &fr_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "fr_count"));

  // create publisher back left
  RCCHECK(rclc_publisher_init_default(
    &bl_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "bl_count"));

  // create publisher back right
  RCCHECK(rclc_publisher_init_default(
    &br_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "br_count"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

// Initialize starting loop paramaters for calculating velocity and time

unsigned long prev = 0;
int old_ct1=0;
int old_ct2=0;
int old_ct3=0;
int old_ct4=0;
float ticks_per_meter = 10000.1;

// maximum time that the system should wait before stopping the motor
// if there is no velocity command
int max_cmd_hold_time = 3000;

void loop() 
{

  // count encoder ticks
  int ct1 = encoder_fleft.read();
  int ct2 = encoder_fright.read();
  int ct3 = encoder_bleft.read();
  int ct4 = encoder_bright.read();

  // for some reason if i omit this it does not work properly
  if (ct1!=-1){
    fl_count.data = ct1;}
  if (ct2!=-1){
    fr_count.data = ct2;}
  if (ct3!=-1){
    bl_count.data = ct3;}
  if (ct4!=-1){
    br_count.data = ct4;}

  // publish encoder ticks
  rcl_publish(&fl_publisher, &fl_count, NULL);
  rcl_publish(&fr_publisher, &fr_count, NULL);
  rcl_publish(&bl_publisher, &bl_count, NULL);
  rcl_publish(&br_publisher, &br_count, NULL);

  // calculate time and current velocity
  
  unsigned long now = millis();
  Input_fl = (float(ct1 - old_ct1) / ticks_per_meter) / ((now - prev) / 1000.0);
  Input_fr = (float(ct2 - old_ct2) / ticks_per_meter) / ((now - prev) / 1000.0);
  Input_bl = (float(ct3 - old_ct3) / ticks_per_meter) / ((now - prev) / 1000.0);
  Input_br = (float(ct4 - old_ct4) / ticks_per_meter) / ((now - prev) / 1000.0);
 
  // Use aggresive pid paramaters if gap > 0.1 else conservative
  bool gap1 = abs(Setpoint_fl - Input_fl) < 0.12;
  bool gap2 = abs(Setpoint_fr - Input_fr) < 0.12;
  bool gap3 = abs(Setpoint_bl - Input_bl) < 0.12;
  bool gap4 = abs(Setpoint_br - Input_br) < 0.12;
  
  if(gap1 && gap2 && gap3 && gap4){
    myPID_fl.SetTunings(consKp, consKi, consKd);
    myPID_fr.SetTunings(consKp, consKi, consKd);
    myPID_bl.SetTunings(consKp, consKi, consKd);
    myPID_br.SetTunings(consKp, consKi, consKd);
  }
  else{
    myPID_fl.SetTunings(aggKp, aggKi, aggKd);
    myPID_fr.SetTunings(aggKp, aggKi, aggKd);
    myPID_bl.SetTunings(aggKp, aggKi, aggKd);
    myPID_br.SetTunings(aggKp, aggKi, aggKd);
  }
  if(wtf){
   reset_pid_Ki();
  }
  // Compute  Pid
  myPID_fl.Compute();
  myPID_fr.Compute();
  myPID_bl.Compute();
  myPID_br.Compute();

  // Move the motors with the output of the pid
  Move_motor(Output_fl,LF_PWM,LF_FORW,LF_BACK);
  Move_motor(Output_fr,RF_PWM,RF_FORW,RF_BACK);
  Move_motor(Output_bl,LB_PWM,LB_FORW,LB_BACK);
  Move_motor(Output_br,RB_PWM,RB_FORW,RB_BACK);

  //delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(25)));
  now = millis();

  // take the old encoder ticks and time for calculating velocity
  old_ct1 = encoder_fleft.read();
  old_ct2 = encoder_fright.read();
  old_ct3 = encoder_bleft.read();
  old_ct4 = encoder_bright.read();
  prev = now;

  delay(25);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// void loop() {
//   delay(100);
// }