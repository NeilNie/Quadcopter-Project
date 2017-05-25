/*
   This program is written by Neil Nie. Different parts of the software is reference from J. Brokking,
   and some code is taken directly from his repository. I deeply appreciate him for written a detailed
   tutorial on building quadcopter. This program can be adapted to many different hardware configerations.

   MIT License (c) Yongyang Nie 2017
   I, Yongyang Nie hereby grant you the full rights to freely copy, edit and distribute the program.
   However, I am not responsible for any damages and inguiries caused by this program. Quadcopters are
   dangerous. Use this software responsively.
*/
//Include LCD and I2C library
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

//PID gain and limit settings
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float p_gain_roll = 1.3;             
float i_gain_roll = 0.02;             
float d_gain_roll = 13.0;           

float p_gain_pitch = p_gain_roll; 
float i_gain_pitch = i_gain_roll;
float d_gain_pitch = d_gain_roll;  

float p_gain_yaw = 4.0;               //4.0
float i_gain_yaw = 0.02;              //0.02
float d_gain_yaw = 0.0;              
int PID_MAX = 400;

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring some global variables
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
int throttle, battery_voltage;
int gyro_roll, gyro_pitch, gyro_yaw;
long acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_roll_acc, angle_pitch_acc;
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int loop_counter;
int esc_1, esc_2, esc_3, esc_4;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
int start;
const float Pi = 3.14159;

//Initialize the LCD library
LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {

  // set PCIE0 to enable PCMSK0 scan
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Quadcopter  ");
  lcd.setCursor(0, 1);
  lcd.print("   Neil Nie");
  delay(1000);
  lcd.clear();

  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                                                        //Configure digital poort 12 and 13 as output.

  Wire.begin();                                                        //Start I2C as master
  TWBR = 12;
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);

  calibrate_gyro();

  battery_voltage = (analogRead(0) + 65) * 1.2317;
  lcd.clear();
  delay(50);
  start = 0;
  loop_timer = micros();
}

void loop() {

  read_gyro_data();

  gyro_roll -= gyro_x_cal;
  gyro_pitch -= gyro_y_cal;
  gyro_yaw -= gyro_z_cal;

  //65.5 = 1 deg/sec
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);

  //---------------------------------------------------------------------------
  calculate_angle();
  //---------------------------------------------------------------------------

  if (start == 0 && receiver_input_channel_2 < 1100 && receiver_input_channel_1 < 1100) start = 1;
  if (start == 1 && receiver_input_channel_2 < 1100 && receiver_input_channel_1 > 1450) start = 2; resetPID();
  if (start == 2 && receiver_input_channel_2 < 1100 && receiver_input_channel_1 > 1800) start = 0;

  //---------------------------------------------------------------------------
  //channel 1 --> roll
  //channel 2 --> throttle
  //channel 3 --> pitch
  //channel 4 --> yaw

  pid_roll_setpoint = 0;
  if (receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if (receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  pid_pitch_setpoint = 0;
  if (receiver_input_channel_3 > 1508)pid_pitch_setpoint = receiver_input_channel_3 - 1508;
  else if (receiver_input_channel_3 < 1492)pid_pitch_setpoint = receiver_input_channel_3 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= -3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  pid_yaw_setpoint = 0;
  if (receiver_input_channel_2 > 1050) { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }

  //---------------------------------------------------------------------------------------

  calculate_pid();

  //---------------------------------------------------------------------------------------

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if (battery_voltage < 1030 && battery_voltage > 600) digitalWrite(2, HIGH);

  throttle = receiver_input_channel_2;

  if (start == 2) {

    if (throttle > 1800) throttle = 1800;

    esc_1 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_2 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_3 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
    esc_4 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)

    //voltage drop calculation
    if (battery_voltage < 1240 && battery_voltage > 800) {
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500);
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500);
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500);
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500);
    }

    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;

    if (esc_1 > 1800) esc_1 = 1800;
    if (esc_2 > 1800) esc_2 = 1800;
    if (esc_3 > 1800) esc_3 = 1800;
    if (esc_4 > 1800) esc_4 = 1800;

    //---------------------------------------------------------------------------------------

  }
  else {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  //All the information for controlling the motor's is available
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while (micros() - loop_timer < 4000);                                     //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                    
  timer_channel_2 = esc_2 + loop_timer;                                   
  timer_channel_3 = esc_3 + loop_timer;                               
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse

  while (PORTD >= 16) {                                                    
    esc_loop_timer = micros();                                              //Read the current time.
    if (timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;               
    if (timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;             
    if (timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;        
    if (timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;   
  }
}

void calibrate_gyro() {

  lcd.setCursor(0, 0);
  lcd.print("Calibrating");
  lcd.setCursor(0, 1);
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
    if (cal_int % 125 == 0) {
      lcd.print(".");
      digitalWrite(13, HIGH);                           //Print a dot on the LCD every 125 readings
    }
    read_gyro_data();
    gyro_x_cal += gyro_roll;
    gyro_y_cal += gyro_pitch;
    gyro_z_cal += gyro_yaw;

    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);   //Delay 3us to simulate the 25
    digitalWrite(13, LOW);
  }

  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
}

void calculate_angle() {

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (pi / 180). Transferring angels (math that I don't understand)
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000061 * (Pi / 180));
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000061 * (Pi / 180));

  //Accelerometer angle calculations, Calculate the total accelerometer vector.
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  if (abs(acc_y) < acc_total_vector) angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;       //Calculate the pitch angle.
  if (abs(acc_x) < acc_total_vector) angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;

  /*//Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
    angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
    angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  */
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;
}

//-----------------------------------------------------------------------------------------
//This routine is called every time input 8, 9, 10 or 11 changed state.
//Measure the time, using micros(), between raising and falling edges of the ESC.
ISR(PCINT0_vect) {
  //Channel 1--------------------------
  if (last_channel_1 == 0 && PINB & B00000001 ) {
    last_channel_1 = 1;
    timer_1 = micros();
  }
  else if (last_channel_1 == 1 && !(PINB & B00000001)) {
    last_channel_1 = 0;
    receiver_input_channel_1 = micros() - timer_1;
  }
  
  //Channel 2--------------------------
  if (last_channel_2 == 0 && PINB & B00000010 ) {
    last_channel_2 = 1;
    timer_2 = micros();
  }
  else if (last_channel_2 == 1 && !(PINB & B00000010)) {
    last_channel_2 = 0;
    receiver_input_channel_2 = micros() - timer_2;
  }
  
  //Channel 3--------------------------
  if (last_channel_3 == 0 && PINB & B00000100 ) {
    last_channel_3 = 1;
    timer_3 = micros();
  }
  else if (last_channel_3 == 1 && !(PINB & B00000100)) {
    last_channel_3 = 0;
    receiver_input_channel_3 = micros() - timer_3;
  }
  
  //Channel 4--------------------------
  if (last_channel_4 == 0 && PINB & B00001000 ) {
    last_channel_4 = 1;
    timer_4 = micros();
  }
  else if (last_channel_4 == 1 && !(PINB & B00001000)) {
    last_channel_4 = 0;
    receiver_input_channel_4 = micros() - timer_4;
  }
}

void read_gyro_data() {

  //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  Wire.read() << 8 | Wire.read();
  gyro_roll = Wire.read() << 8 | Wire.read();
  gyro_pitch = Wire.read() << 8 | Wire.read();
  gyro_yaw = Wire.read() << 8 | Wire.read();

  gyro_pitch = gyro_pitch * -1;
  gyro_yaw = gyro_yaw * -1;
}

void write_LCD() {                                                     //Subroutine for writing the LCD
  //To get a 250Hz program loop (4us) it's only possible to write one character per loop
  //Writing multiple characters is taking to much time
  if (lcd_loop_counter == 14)lcd_loop_counter = 0;                     //Reset the counter after 14 characters
  lcd_loop_counter ++;                                                 //Increase the counter
  if (lcd_loop_counter == 1) {
    angle_pitch_buffer = angle_pitch * 10;                      //Buffer the pitch angle because it will change
    lcd.setCursor(6, 0);                                               //Set the LCD cursor to position to position 0,0
  }
  if (lcd_loop_counter == 2) {
    if (pid_output_pitch < 0)lcd.print("-");                         //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if (lcd_loop_counter == 3)lcd.print(abs(angle_pitch_buffer) / 1000); //Print first number
  if (lcd_loop_counter == 4)lcd.print((abs(angle_pitch_buffer) / 100) % 10); //Print second number
  if (lcd_loop_counter == 5)lcd.print((abs(angle_pitch_buffer) / 10) % 10); //Print third number
  if (lcd_loop_counter == 6)lcd.print(".");                            //Print decimal point
  if (lcd_loop_counter == 7)lcd.print(abs(angle_pitch_buffer) % 10);   //Print decimal number

  if (lcd_loop_counter == 8) {
    angle_roll_buffer = angle_roll * 10;
    lcd.setCursor(6, 1);
  }
  if (lcd_loop_counter == 9) {
    if (pid_output_roll < 0)lcd.print("-");                          //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if (lcd_loop_counter == 10)lcd.print(abs(angle_roll_buffer) / 1000);
  if (lcd_loop_counter == 11)lcd.print((abs(angle_roll_buffer) / 100) % 10);
  if (lcd_loop_counter == 12)lcd.print((abs(angle_roll_buffer) / 10) % 10);
  if (lcd_loop_counter == 13)lcd.print(".");
  if (lcd_loop_counter == 14)lcd.print(abs(angle_roll_buffer) % 10);
}

void setup_mpu_6050_registers() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                                    //End the transmission wit

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                          //Start reading @ register 0x1B
  Wire.endTransmission();                                                    //End the transmission
  Wire.requestFrom(0x68, 1);                                         //Request 1 bytes from the gyro
  while (Wire.available() < 1);                                              //Wait until the 6 bytes are received
  if (Wire.read() != 0x08) {                                                 //Check if the value is 0x08
    digitalWrite(12, HIGH);                                                  //Turn on the warning led
    while (1)delay(10);                                                      //Stay in this loop for ever
  }

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
//Taken from brokking.net
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > PID_MAX)pid_i_mem_roll = PID_MAX;
  else if (pid_i_mem_roll < PID_MAX * -1)pid_i_mem_roll = PID_MAX * -1;

  pid_output_roll = p_gain_roll * pid_error_temp + pid_i_mem_roll + d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > PID_MAX)pid_output_roll = PID_MAX;
  else if (pid_output_roll < PID_MAX * -1)pid_output_roll = PID_MAX * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > PID_MAX)pid_i_mem_pitch = PID_MAX;
  else if (pid_i_mem_pitch < PID_MAX * -1)pid_i_mem_pitch = PID_MAX * -1;

  pid_output_pitch = p_gain_pitch * pid_error_temp + pid_i_mem_pitch + d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > PID_MAX)pid_output_pitch = PID_MAX;
  else if (pid_output_pitch < PID_MAX * -1)pid_output_pitch = PID_MAX * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > PID_MAX)pid_i_mem_yaw = PID_MAX;
  else if (pid_i_mem_yaw < PID_MAX * -1)pid_i_mem_yaw = PID_MAX * -1;

  pid_output_yaw = p_gain_yaw * pid_error_temp + pid_i_mem_yaw + d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > PID_MAX)pid_output_yaw = PID_MAX;
  else if (pid_output_yaw < PID_MAX * -1)pid_output_yaw = PID_MAX * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void resetPID() {

  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;
  pid_pitch_setpoint = 0;
  pid_roll_setpoint = 0;
  pid_yaw_setpoint = 0;
}

