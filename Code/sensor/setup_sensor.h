/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  29 December 2022 -  initial release
*/

#include <Wire.h>
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw = 0;
float LoopTimer;
float pi = 3.14159;

float AccX_bias, AccY_bias, AccZ_bias; // För bias beräkningar
float Roll_bias, Pitch_bias, Yaw_bias;
int calibration_idx; 

//uint32_t LoopTimer; // Timer
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2; // För kalman filter
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2; // inital värden
float Kalman1DOutput[]={0,0};
#define sensor_SCL A5
#define sensor_SDA A4
#define int_pin 1

float position_vec[]={0,0,0};
float angle_vec[]={0,0,0};

void write_sensor_data();
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.005 * 0.005 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}


void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  // Division med 65.5 -> Grader/s
  RateRoll=(float)GyroX/65.5;      
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  
  AccX=(float)AccXLSB/4096 + 0.02;
  AccY=(float)AccYLSB/4096 + 0.03;
  AccZ=(float)AccZLSB/4096 + 0.02;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

  AngleYaw = AngleYaw + RateYaw*0.05;
}


void setup_sensor() {
  
  pinMode(int_pin, OUTPUT);
  digitalWrite(int_pin, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  //Serial.print("Börjar kalibrering");
  int calibration_iter=40000;
  for (calibration_idx=0;
         calibration_idx<calibration_iter; 
         calibration_idx ++) {
    gyro_signals();
    Roll_bias+=RateRoll; // Addera ett mätvärde varje millisekund
    Pitch_bias+=RatePitch;
    Yaw_bias+=RateYaw;

    //AccX_bias += AccX;
    //AccY_bias += AccY;
    //AccZ_bias += AccZ;
    //delay(1);
  }
  Roll_bias /=calibration_iter; // Dela med 2000 för medelvärde
  Pitch_bias/=calibration_iter;
  Yaw_bias  /=calibration_iter;  
  //AccX_bias /=2000;
  //AccY_bias /=2000;
  //AccZ_bias /=2000;
  //Serial.print("Slutar kalibrering");
}
void read_sensor();
void read_sensor(){
  gyro_signals();
  
  /*
  AccX -= AccX_bias;
  AccY -= AccY_bias;
  AccZ -= AccZ_bias-1;
  */

  RateRoll -= Roll_bias;
  RatePitch -= Pitch_bias;
  RateYaw -= Yaw_bias;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  

  //Vill kanske egentligen ha en vektor med pointers så det går snabbare
  const float meters2_to_mms2=1;
  position_vec[0]=AccX*meters2_to_mms2*9.82;
  position_vec[1]=AccY*meters2_to_mms2*9.82;
  position_vec[2]=AccZ*meters2_to_mms2*9.82;

  //angle_vec[0]=KalmanAngleRoll*meters2_to_mms2;
  //angle_vec[1]=KalmanAnglePitch*meters2_to_mms2;
  angle_vec[2]=AngleYaw*meters2_to_mms2;

  // testa att inte skicka Kalman Vinklarna
  angle_vec[0]=AngleRoll*meters2_to_mms2;
  angle_vec[1]=AnglePitch*meters2_to_mms2;



  }


void fake_loop() {

  gyro_signals();
  
  /*
  AccX -= AccX_bias;
  AccY -= AccY_bias;
  AccZ -= AccZ_bias-1;
  */

  RateRoll -= Roll_bias;
  RatePitch -= Pitch_bias;
  RateYaw -= Yaw_bias;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  
  
  Serial.print("Acceleration X [g]= ");
  Serial.print(AccX*9.82);
  Serial.print(" Acceleration Y [g]= ");
  Serial.print(AccY*9.82);
  Serial.print(" Acceleration Z [g]= ");
  Serial.print(AccZ*9.82);
  Serial.println();
  

  Serial.print("Angle X: ");
  //Serial.print(AngleRoll);
  //Serial.print(" ");
  Serial.print(KalmanAngleRoll);
  //Serial.print(RateRoll);
  
  Serial.print("\t Angle Y: ");
  //Serial.print(AnglePitch);
  //Serial.print(RatePitch);
  Serial.print(KalmanAnglePitch);

  Serial.print("\t Angle Z: ");
  Serial.println(AngleYaw);
  //Serial.println(RateYaw);

  delay(50);
}