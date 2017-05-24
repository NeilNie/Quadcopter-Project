
//Include LCD and I2C library
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.40;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.010;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 13.00;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 2.00;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.00;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

//Declaring some global variables
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
int throttle, battery_voltage;
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
int start;

//Initialize the LCD library
LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {

  Serial.begin(9600);

  TWBR = 12;

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

  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);

  lcd.setCursor(0, 0);
  lcd.print("Calibrating");
  lcd.setCursor(0, 1);
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {
    if (cal_int % 125 == 0) {
      lcd.print(".");
      digitalWrite(13, HIGH);                           //Print a dot on the LCD every 125 readings
    }
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);   //Delay 3us to simulate the 25
    digitalWrite(13, LOW);
  }

  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;

  battery_voltage = (analogRead(0) + 65) * 1.2317;
  lcd.clear();
  resetPID();
  delay(50);
  start = 0;
  loop_timer = micros();
}

void loop() {

  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_z / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_y / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_z / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

  if (start == 0 && receiver_input_channel_2 < 1100 && receiver_input_channel_1 < 1100)
    start = 1;
  if (start == 1 && receiver_input_channel_2 < 1100 && receiver_input_channel_1 > 1450) {
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  if (start == 2 && receiver_input_channel_2 < 1100 && receiver_input_channel_1 > 1800) start = 0;

  //channel 1 --> roll
  //channel 2 --> throttle
  //channel 3 --> pitch
  //channel 4 --> yaw

  pid_pitch_setpoint = 0.0;
  pid_roll_setpoint = 0.0;
  pid_yaw_setpoint = 0.0;

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of dividing by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  /*
    pid_roll_setpoint = 0;
    if(receiver_input_channel_1 > 1470) pid_roll_setpoint = (receiver_input_channel_1 - 1470)/3.0;
    else if(receiver_input_channel_1 < 1460) pid_roll_setpoint = (receiver_input_channel_1 - 1460)/3.0;

    pid_pitch_setpoint = 0;
    if(receiver_input_channel_3 > 1470) pid_pitch_setpoint = (receiver_input_channel_3 - 1470)/3.0;
    else if(receiver_input_channel_3 < 1460)pid_pitch_setpoint = (receiver_input_channel_3 - 1460)/3.0;

    pid_yaw_setpoint = 0;
    if(receiver_input_channel_2 > 1100){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1470) pid_yaw_setpoint = (receiver_input_channel_4 - 1470)/3.0;
    else if(receiver_input_channel_4 < 1460) pid_yaw_setpoint = (receiver_input_channel_4 - 1460)/3.0;
    }
  */

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

    esc_1 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw; //esc 3 (rear-left - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw; //esc 4 (front-left - CW)
    esc_3 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //esc 1 (front-right - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; //esc 2 (rear-right - CW)

    //voltage drop calculation
    if (battery_voltage < 1240 && battery_voltage > 800) {
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500);
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500);
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500);
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500);
    }

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                      //Keep the motors running.

    if (esc_1 > 1700) esc_1 = 1700;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 1700) esc_2 = 1700;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 1700) esc_3 = 1700;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 1700) esc_4 = 1700;

    //---------------------------------------------------------------------------------------
    //write_LCD();
  }
  else {
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;
  }

  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while (micros() - loop_timer < 4000);                                     //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse

  while (PORTD >= 16) {                                                     //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if (timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;               //Set digital output 4 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;               //Set digital output 5 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;               //Set digital output 6 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;               //Set digital output 7 to low if the time is expired.
  }
}

void calculate_angle() {

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_y * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_x * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll += angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector.
  if (abs(acc_y) < acc_total_vector)                             //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;       //Calculate the pitch angle.
  if (abs(acc_x) < acc_total_vector)                                   //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;       //Calculate the roll angle.

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for rol

  angle_pitch_output = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll_output = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
}

//This routine is called every time input 8, 9, 10 or 11 changed state.
//Measure the time, using micros(), between each port changes from 0 to 1 and 1 to 0.
//Basically, the raising and falling edges of the ESC.
ISR(PCINT0_vect) {
  //Channel 1=========================================
  if (last_channel_1 == 0 && PINB & B00000001 ) {
    last_channel_1 = 1;
    timer_1 = micros();
  }
  else if (last_channel_1 == 1 && !(PINB & B00000001)) {
    last_channel_1 = 0;
    receiver_input_channel_1 = micros() - timer_1;
  }
  //Channel 2=========================================
  if (last_channel_2 == 0 && PINB & B00000010 ) {
    last_channel_2 = 1;
    timer_2 = micros();
  }
  else if (last_channel_2 == 1 && !(PINB & B00000010)) {
    last_channel_2 = 0;
    receiver_input_channel_2 = micros() - timer_2;
  }
  //Channel 3=========================================
  if (last_channel_3 == 0 && PINB & B00000100 ) {
    last_channel_3 = 1;
    timer_3 = micros();
  }
  else if (last_channel_3 == 1 && !(PINB & B00000100)) {
    last_channel_3 = 0;
    receiver_input_channel_3 = micros() - timer_3;
  }
  //Channel 4=========================================
  if (last_channel_4 == 0 && PINB & B00001000 ) {
    last_channel_4 = 1;
    timer_4 = micros();
  }
  else if (last_channel_4 == 1 && !(PINB & B00001000)) {
    last_channel_4 = 0;
    receiver_input_channel_4 = micros() - timer_4;
  }
}

void read_mpu_6050_data() {
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
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void write_LCD() {                                                     //Subroutine for writing the LCD
  //To get a 250Hz program loop (4us) it's only possible to write one character per loop
  //Writing multiple characters is taking to much time
  if (lcd_loop_counter == 14)lcd_loop_counter = 0;                     //Reset the counter after 14 characters
  lcd_loop_counter ++;                                                 //Increase the counter
  if (lcd_loop_counter == 1) {
    angle_pitch_buffer = pid_output_pitch * 10;                      //Buffer the pitch angle because it will change
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
    angle_roll_buffer = pid_output_roll * 10;
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void resetPID() {
  //reset all PID controllers
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

