#include <PID_v1.h>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "Arduino.h"

#define TICKS_PER_METER 1130

ros::NodeHandle nh;

int inApin[2] = { 9, 10 };  // INA: Clockwise input
int inBpin[2] = { 6, 11 }; // INB: Counter-clockwise input

unsigned long last_cmd = 0;

double vtarget_l = 0;
double cmd_l = 0;
volatile double vel_l = 0;
volatile int lticks;
volatile unsigned long last_time_l;
std_msgs::Int32 lticksMsg;
std_msgs::Float64 lVelMsg;

PID pid_l(&vel_l, &cmd_l, &vtarget_l, 0.5, 0.5, 0, DIRECT);

void lVelCmd(const std_msgs::Float32& msg) {
  last_cmd = millis();
  vtarget_l = msg.data;
}

void lTick() {
  int dir = digitalRead(5)==HIGH?1:-1;
  lticks += dir;

  unsigned long dT = micros() - last_time_l;
  last_time_l = micros();
  vel_l = 1000000 / (float)dT / TICKS_PER_METER * (float)dir;
}

double vtarget_r = 0;
double cmd_r = 0;
volatile double vel_r = 0;
volatile int rticks;
volatile unsigned long last_time_r;
std_msgs::Int32 rticksMsg;
std_msgs::Float64 rVelMsg;

PID pid_r(&vel_r, &cmd_r, &vtarget_r, 0.5, 0.5, 0, DIRECT);

void rVelCmd(const std_msgs::Float32& msg) {
  last_cmd = millis();
  vtarget_r = msg.data;
}

void rTick() {
  int dir = digitalRead(4)==HIGH?-1:1;
  rticks += dir;

  unsigned long dT = micros() - last_time_r;
  last_time_r = micros();
  vel_r = 1000000 / (float)dT / TICKS_PER_METER * (float)dir;
}

ros::Subscriber<std_msgs::Float32> subL("lwheel_vtarget", &lVelCmd);
ros::Subscriber<std_msgs::Float32> subR("rwheel_vtarget", &rVelCmd);
ros::Publisher lTickPub("lticks", &lticksMsg);
ros::Publisher lVelPub("lwheel_vel", &lVelMsg);
ros::Publisher rTickPub("rticks", &rticksMsg);
ros::Publisher rVelPub("rwheel_vel", &rVelMsg);

void initMotor(int number) {
  pinMode(inApin[number], OUTPUT);
  pinMode(inBpin[number], OUTPUT);

  //brake initially
  digitalWrite(inApin[number], LOW);
  digitalWrite(inBpin[number], LOW);

  pinMode(13, OUTPUT);
}

void setMotorPower(int number, double power) {
  int pwmPower = power * 255;
  if (abs(power) < 0.001) {
    digitalWrite(inApin[number], LOW);
    digitalWrite(inBpin[number], LOW);

  } else if (power > 0) {
    pwmPower = 255 - pwmPower;
    analogWrite(inApin[number], pwmPower);
    digitalWrite(inBpin[number], HIGH);
  } else {
    analogWrite(inApin[number], -pwmPower);
    digitalWrite(inBpin[number], LOW);
  }
}

//The setup function is called once  at startup of the sketch
void setup() {
  //Serial.begin(115200);
  initMotor(0);
  initMotor(1);
  setMotorPower(0, 0);
  setMotorPower(1, 0);

  lticks = 0;
  pinMode(3, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), &lTick, RISING);
  
  rticks = 0;
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), &rTick, RISING);

  nh.initNode();

  while(!nh.connected()) {nh.spinOnce();}

  float maxPower = 0.5;
  int sampleTime = 10;

  float KP = 0.5;
  float KI = 0.5;
  float KD = 0.0;

  nh.getParam("~maxpower", &maxPower);
  nh.getParam("~sampletime", &sampleTime);
  nh.getParam("~Kp_l", &KP);
  nh.getParam("~Ki_l", &KI);
  nh.getParam("~Kd_l", &KD);

  pid_l.SetTunings(KP, KI, KD);
  pid_l.SetSampleTime(sampleTime);
  pid_l.SetOutputLimits(-maxPower, maxPower);
  pid_l.SetMode(AUTOMATIC);
  
  KP = 0.5;
  KI = 0.5;
  KD = 0.0;

  nh.getParam("~Kp_r", &KP);
  nh.getParam("~Ki_r", &KI);
  nh.getParam("~Kd_r", &KD);

  pid_r.SetTunings(KP, KI, KD);
  pid_r.SetSampleTime(sampleTime);
  pid_r.SetOutputLimits(-maxPower, maxPower);
  pid_r.SetMode(AUTOMATIC);
  
  nh.advertise(lTickPub);
  nh.advertise(rTickPub);
  nh.advertise(lVelPub);
  nh.advertise(rVelPub);
  nh.subscribe(subL);
  nh.subscribe(subR);

  digitalWrite(13, LOW);
}

unsigned long nextSpin = 0;
#define SPIN_RATE 20

// The loop function is called in an endless loop
void loop() {
  if((millis() - last_cmd) > 1000) {
    cmd_l = 0;
    cmd_r = 0;
  }
  
  unsigned long dT = micros() - last_time_l;
  if(dT > 100000L) {
    vel_l = 0;
  }

  dT = micros() - last_time_r;
  if(dT > 100000L) {
    vel_r = 0;
  }
  
  if(pid_l.Compute()) {
    if(abs(vtarget_l) < 0.01) {
      setMotorPower(1, 0);
    } else {
      setMotorPower(1, cmd_l);
    }
  }
  
  if(pid_r.Compute()) {
    if(abs(vtarget_r) < 0.01) {
      setMotorPower(0, 0);
    } else {
      setMotorPower(0, cmd_r);
    }
  }

  if(millis() > nextSpin) {
    nextSpin = millis() + 1000/SPIN_RATE;
    
    lticksMsg.data = lticks;
    lTickPub.publish(&lticksMsg);
  
    lVelMsg.data = vel_l;
    lVelPub.publish(&lVelMsg);
  
    rticksMsg.data = rticks;
    rTickPub.publish(&rticksMsg);
    
    rVelMsg.data = vel_r;
    rVelPub.publish(&rVelMsg);
    
    nh.spinOnce();
  }
}
