#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.3)
float pid_d_gain_roll = 15;                //Gain setting for the roll D-controller (15)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int PID_MAX = 400;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1;
int receiver_input_channel_2;
int receiver_input_channel_3;
int receiver_input_channel_4;

int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;
bool buttonDown;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  Wire.begin();                                                //Start the I2C as master.

  DDRD |= B11110000;                                           //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                                           //Configure digital poort 12 and 13 as output.
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.

  //Use the led on the Arduino for startup indication
  digitalWrite(12, HIGH);                                      //Turn on the warning led.
  delay(3000);                                                 //Wait 2 second befor continuing.

  Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(0x20);                                            //We want to write to register 1 (20 hex)
  Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis)
  Wire.endTransmission();                                      //End the transmission with the gyro

  Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(0x23);                                            //We want to write to register 4 (23 hex)
  Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale)
  Wire.endTransmission();                                      //End the transmission with the gyro

  delay(250);                                                  //Give the gyro time to start.

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {             //Take 2000 readings for calibration.
    if (cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));  //Change the led status to indicate calibration.
    gyro_signalen();                                           //Read the gyro output.
    gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
    gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
    gyro_yaw_cal += gyro_yaw;                                  //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
  gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
  gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.

  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.                                                 //Set start back to 0.

  start = 2;
  //Reset the pid controllers for a bumpless start.
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;

  //When everything is done, turn off the led.
  digitalWrite(12, LOW);                                       //Turn off the warning led.
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  if (buttonDown) {
    receiver_input_channel_1 = 1200;
    receiver_input_channel_2 = 1200;
    receiver_input_channel_3 = 1200;
    receiver_input_channel_4 = 1200;
  }

  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  gyro_signalen();
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

  //------------------------------------------------------------------------------------------------------------------------
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_1 > 1508)
    pid_roll_setpoint = (receiver_input_channel_1 - 1508) / 3.0;
  else if (receiver_input_channel_1 < 1492)
    pid_roll_setpoint = (receiver_input_channel_1 - 1492) / 3.0;

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_2 > 1508)
    pid_pitch_setpoint = (receiver_input_channel_2 - 1508) / 3.0;
  else if (receiver_input_channel_2 < 1492)
    pid_pitch_setpoint = (receiver_input_channel_2 - 1492) / 3.0;

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050) { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)
      pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492)
      pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }

  //------------------------------------------------------------------------------------------------------------------------
  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();

  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.
  //assume full battery
  battery_voltage = 1240;
  //------------------------------------------------------------
  if (start == 2) {                                                         //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800) {                  //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-4 pulse for voltage drop.
    }

    if (esc_1 < 1200) esc_1 = 1200;                                         //Keep the motors running.
    if (esc_2 < 1200) esc_2 = 1200;                                         //Keep the motors running.
    if (esc_3 < 1200) esc_3 = 1200;                                         //Keep the motors running.
    if (esc_4 < 1200) esc_4 = 1200;                                         //Keep the motors running.

    if (esc_1 > 2000) esc_1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000) esc_2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000) esc_3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000) esc_4 = 2000;                                          //Limit the esc-4 pulse to 2000us.
  }

  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while (micros() - loop_timer < 4000);                                     //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.

  while (PORTD >= 16) {                                                     //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if (timer_channel_1 <= esc_loop_timer) PORTD &= B11101111;               //Set digital output 4 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer) PORTD &= B11011111;               //Set digital output 5 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer) PORTD &= B10111111;               //Set digital output 6 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer) PORTD &= B01111111;               //Set digital output 7 to low if the time is expired.
  }
}

//*************************************************************************************************************************************
//Subroutine for reading the gyro
//*************************************************************************************************************************************
void gyro_signalen() {
  Wire.beginTransmission(105);                                 //Start communication with the gyro (adress 1101001)
  Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(105, 6);                                    //Request 6 bytes from the gyro
  while (Wire.available() < 6);                                //Wait until the 6 bytes are received
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_roll = ((highByte << 8) | lowByte);                     //Multiply highByte by 256 (shift left by 8) and ad lowByte
  if (cal_int == 2000)
    gyro_roll -= gyro_roll_cal;              //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_pitch = ((highByte << 8) | lowByte);                    //Multiply highByte by 256 (shift left by 8) and ad lowByte
  gyro_pitch *= -1;                                            //Invert axis
  if (cal_int == 2000)
    gyro_pitch -= gyro_pitch_cal;            //Only compensate after the calibration
  lowByte = Wire.read();                                       //First received byte is the low part of the angular data
  highByte = Wire.read();                                      //Second received byte is the high part of the angular data
  gyro_yaw = ((highByte << 8) | lowByte);                      //Multiply highByte by 256 (shift left by 8) and ad lowByte
  gyro_yaw *= -1;                                              //Invert axis
  if (cal_int == 2000)
    gyro_yaw -= gyro_yaw_cal;                //Only compensate after the calibration
}

//*************************************************************************************************************************************
//Subroutine for calculating pid outputs
//*************************************************************************************************************************************
//www.youtube.com/watch?v=JBvnB0279-Q

void calculate_pid() {

  //---------------------------------------------------Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > PID_MAX)
    pid_i_mem_roll = PID_MAX;
  else if (pid_i_mem_roll < PID_MAX * -1)
    pid_i_mem_roll = PID_MAX * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > PID_MAX)
    pid_output_roll = PID_MAX;
  else if (pid_output_roll < PID_MAX * -1)
    pid_output_roll = PID_MAX * -1;

  pid_last_roll_d_error = pid_error_temp;

  //---------------------------------------------------Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > PID_MAX)
    pid_i_mem_pitch = PID_MAX;
  else if (pid_i_mem_pitch < PID_MAX * -1)
    pid_i_mem_pitch = PID_MAX * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > PID_MAX)
    pid_output_pitch = PID_MAX;
  else if (pid_output_pitch < PID_MAX * -1)
    pid_output_pitch = PID_MAX * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //---------------------------------------------------Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > PID_MAX)
    pid_i_mem_yaw = PID_MAX;
  else if (pid_i_mem_yaw < PID_MAX * -1)
    pid_i_mem_yaw = PID_MAX * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > PID_MAX)
    pid_output_yaw = PID_MAX;
  else if (pid_output_yaw < PID_MAX * -1)
    pid_output_yaw = PID_MAX * -1;

  pid_last_yaw_d_error = pid_error_temp;
}


