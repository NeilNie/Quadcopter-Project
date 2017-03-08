#include <Wire.h>

double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal; //for calibration
int cal_int;
byte highByte, lowByte;

void setup() {

  Serial.begin(9600);
  Wire.begin();

  Wire.beginTransmission(150); //start communication (address 1101001)
  Wire.write(0x20); //write to register 20;
  Wire.write(0x0F); //turn on the gyro and enable all axis
  Wire.endTransmission();

  Wire.beginTransmission(105); //start communication (address 1101001)
  Wire.write(0x23); //write to register 23
  Wire.write(0x80); //Set up BDU bit.
  Wire.endTransmission();

  delay(250);

  Serial.print("Start calibration...");
  for (cal_int = 0; cal_int < 2001; cal_int ++){
    get_gyro_data();
    gyro_pitch_cal += gyro_pitch;
    gyro_roll_cal += gyro_roll;
    gyro_yaw_cal += gyro_yaw;
    if (cal_int % 100 == 0)
      Serial.print(".");
    delay(4);
  }

  Serial.println("Done, " + cal_int);
  gyro_yaw_cal /= 2000;
  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
}

void loop() {

  delay(250);
  get_gyro_data();
  print_gyro_data();
}

void get_gyro_data(){
  Wire.beginTransmission(105);
  Wire.write(168); //start @ register 28 and auto increment
  Wire.endTransmission();
  Wire.requestFrom(105, 6); //request 6 bytes from gyro
  while (Wire.available() < 6); //wait until recieves

  lowByte = Wire.read(); //low part of angular data;
  highByte = Wire.read(); //high part of angular data
  gyro_roll = ((highByte << 8) | lowByte); //multiply highbyte by 256 and add low byte
  if(cal_int == 2000)
    gyro_roll -= gyro_roll_cal;

  lowByte = Wire.read(); //low part of angular data;
  highByte = Wire.read(); //high part of angular data
  gyro_pitch = ((highByte << 8) | lowByte); //multiply highbyte by 256 and add low byte
  gyro_pitch *= -1;   
  if(cal_int == 2000)
    gyro_pitch -= gyro_pitch_cal;

  lowByte = Wire.read(); //low part of angular data;
  highByte = Wire.read(); //high part of angular data
  gyro_yaw = ((highByte << 8) | lowByte); //multiply highbyte by 256 and add low byte
  gyro_yaw *= -1;
  if(cal_int == 2000)
    gyro_yaw -= gyro_yaw_cal;
}

void print_gyro_data(){

  Serial.print("Pitch:");
  if(gyro_pitch >= 0)Serial.print("+");
  Serial.print(gyro_pitch/57.14286, 0);               //Convert to degree per second
  if(gyro_pitch/57.14286 - 2 > 0) Serial.print(" NoU");
  else if(gyro_pitch/57.14286 + 2 < 0) Serial.print(" NoD");
  else Serial.print(" ---");
  
  Serial.print("  Roll:");
  if(gyro_roll >= 0)Serial.print("+");
  Serial.print(gyro_roll/57.14286,0);                //Convert to degree per second
  if(gyro_roll/57.14286 - 2 > 0) Serial.print(" RwD");
  else if(gyro_roll/57.14286 + 2 < 0) Serial.print(" RwU");
  else Serial.print(" ---");
  
  Serial.print("  Yaw:");
  if(gyro_yaw >= 0)Serial.print("+");
  Serial.print(gyro_yaw/57.14286,0);                 //Convert to degree per second
  if(gyro_yaw/57.14286 - 2 > 0)Serial.println(" NoR");
  else if(gyro_yaw/57.14286 + 2 < 0)Serial.println(" NoL");
  else Serial.println(" ---");
}

