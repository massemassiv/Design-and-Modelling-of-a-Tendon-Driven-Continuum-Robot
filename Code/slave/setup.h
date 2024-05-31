#ifndef SETUP_H
#define SETUP_H
#include <Arduino.h>
#include <math.h>
//#include "DRV8825.h"
#include "A4988.h"
//https://github.com/laurb9/StepperDriver

#define SERIAL_BAUD 115200  // Baudrate
// If DEBUG is set to true, the arduino will send back all the received messages
#define DEBUG true

#define buttonPin 7
//STEPPERS
//*******************************************************************************
#define RPM 40
#define MODE0 10
#define MODE1 11
#define MODE2 12
#define MOTOR_STEPS 200
#define SLEEP 13

#define DIR1 8
#define STEP1 9
A4988 stepper1(MOTOR_STEPS, DIR1, STEP1,MODE0, MODE1, MODE2);
//DRV8825 stepper1(MOTOR_STEPS, DIR1, STEP1, SLEEP, MODE0, MODE1, MODE2);

#define DIR2 2
#define STEP2 3

A4988 stepper2(MOTOR_STEPS, DIR2, STEP2, MODE0, MODE1, MODE2);

#define DIR3 4
#define STEP3 5
A4988 stepper3(MOTOR_STEPS,DIR3,STEP3,MODE0,MODE1,MODE2);
//*******************************************************************************

//Gearbox
//*******************************************************************************
const float DEG_PER_STEP=1.8;
const int N_GROOVE=1; //[-] Antal spåringångar på snäckväxel
const int Z_IN=36; //[-] Antal tänder på ingående kugg
const int Z_UT=13; //[-] Antal tänder på utgående kugg
float U1=double(N_GROOVE)*double(Z_IN);//static_cast<float>(N_GROOVE)/static_cast<float>(Z_IN);
float U2=double(Z_IN)/double(Z_UT);//static_cast<float>(Z_IN)/static_cast<float>(Z_UT);
float U_TOT =U1*U2;

const float DIAM_SPOLE=0.007; //[m]
float DEG_TO_DIST=(double( M_PI)/180.)*double(U_TOT); //Rotation i grader till mm
float DIST_TO_DEG=(180)/(M_PI*DIAM_SPOLE*U_TOT);//(2*double(M_PI)/DIAM_SPOLE)*(180./double(M_PI))/U_TOT;
//*******************************************************************************

//*******************************************************************************
float distance[]={0,0,0};
float s_degrees[]={0,0,0}; 
const float SERVO_CONV=1;//e-4;//1e-3;
float servo_angle;
void get_distance(float *dist);
void move_steppers(float *deg);
void get_degrees(float *deg,float *dist);
//*******************************************************************************
int button_state;

/*
Funktioner 
*/
#define dirPin 2
#define stepPin 3
#define potPin A0
#define stepsPerRevolution 200
float dist2deg(float dist){
  float deg=static_cast<float>(dist*DIST_TO_DEG);
  return deg;
};
void get_distance(float *dist)
{
  float dist1=random(-20,20);
  float dist2=random(-20,20);
  float dist3=random(-20,20);
  dist[0] = static_cast<float>(dist1);
  dist[1] = static_cast<float>(dist2);
  dist[2] = static_cast<float>(dist3);

};
void get_degrees(float *deg,float *dist)
{
  deg[0] = dist2deg(dist[0]);
  deg[1] = dist2deg(dist[1]); 
  deg[2] = dist2deg(dist[2]);
};
void move_steppers(float *deg)
{
  stepper1.rotate(-deg[0]);
  delay(10);

  stepper2.rotate(-deg[1]);
  delay(10);

  stepper3.rotate(-deg[2]);
  delay(10);

};

void setup_motors(){
  


  int MICROSTEP = 1;
  stepper1.setMicrostep(MICROSTEP);//Microstepping mode: 1, 2, 4, 8, 16 or 32 (where supported by driver)
  stepper1.begin(RPM,MICROSTEP);
  stepper1.enable();
  
  stepper2.setMicrostep(MICROSTEP);//Microstepping mode: 1, 2, 4, 8, 16 or 32 (where supported by driver)
  stepper2.begin(RPM,MICROSTEP);
  stepper2.enable();

  stepper3.setMicrostep(MICROSTEP);//Microstepping mode: 1, 2, 4, 8, 16 or 32 (where supported by driver)
  stepper3.begin(RPM,MICROSTEP);
  stepper3.enable();

}
unsigned int current_motor_idx=0;
float fake_sensor_data[]={1,2,3,4,5,6,7,8,9,10};
bool send_sensor_data=false;

void update_sensor_data();
void wait_for_received();
#endif