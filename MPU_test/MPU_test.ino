#include <Wire.h>
#include <MPU6050.h>

MPU 6050 mpu;

void setup() {

  Serial.begin(9600);
  Serial.print(Initializing MPU);

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Can't find MPU, check connection");
    delay(200);
  }

  mpu.calibrateGyro();

  mpu.setThreshold(3);
  // put your setup code here, to run once:

}

void loop() {

  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();
 
  Serial.print(" Xraw = ");
  Serial.print(rawGyro.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawGyro.YAxis);
  Serial.print(" Zraw = ");
  Serial.println(rawGyro.ZAxis);
 
  Serial.print(" Xnorm = ");
  Serial.print(normGyro.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normGyro.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normGyro.ZAxis);
 
  delay(10);
  // put your main code here, to run repeatedly:

}
