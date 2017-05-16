///////////////////////////////////////////////////////////////////////////////////////
/*
  Website: http://www.brokking.net/imu.html
  Youtube: https://youtu.be/4BoIE8YQwM8
  Latest
*//////////////////////////////////////////////////////////////////////////////////////

//Include LCD and I2C library
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int PID_MAX_ROLL = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int PID_MAX_PITCH = PID_MAX_ROLL;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int PID_MAX_YAW = 400;                     //Maximum output of the PID-controller (+/-)

//Declaring some global variables
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
int throttle, battery_voltage;
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
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
int cal_int;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

//Initialize the LCD library
LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {

  lcd.begin();                                                         //Initialize the LCD
  lcd.backlight();                                                     //Activate backlight
  lcd.clear();                                                         //Clear the LCD                                                      //Initialize the LCD                                                       //Clear the LCD

  lcd.setCursor(0, 0);                                                 //Set the LCD cursor to position to position 0,0
  lcd.print("     *FC*  ");                                         //Print text to screen
  lcd.setCursor(0, 1);                                                 //Set the LCD cursor to position to position 0,1
  lcd.print(" Neil Nie V1.1");                                              //Print text to screen

  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                                                        //Configure digital poort 12 and 13 as output.

  Wire.begin();                                                        //Start I2C as master
  Serial.begin(9600);                                               //Use only for debugging
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  pinMode(2, INPUT);
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  delay(1000);                                                         //Delay 1.5 second to display the text
  lcd.clear();                                                         //Clear the LCD

  lcd.setCursor(0, 0);                                                 //Set the LCD cursor to position to position 0,0
  lcd.print("Calibrating");                                       //Print text to screen
  lcd.setCursor(0, 1);
  //Set the LCD cursor to position to position 0,1
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
    if (cal_int % 125 == 0){
      lcd.print("."); 
      digitalWrite(13, HIGH);                           //Print a dot on the LCD every 125 readings
    }
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);   //Delay 3us to simulate the 250Hz program loop
    digitalWrite(13, LOW);
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  lcd.clear();                                                         //Clear the LCD
  lcd.setCursor(0, 0);                                                 //Set the LCD cursor to position to position 0,0
  lcd.print("P:");                                                 //Print text to screen
  lcd.setCursor(0, 1);                                                 //Set the LCD cursor to position to position 0,1
  lcd.print("R:");                                                 //Print text to screen

  digitalWrite(13, LOW);                                               //All done, turn the LED off

  loop_timer = micros();                                               //Reset the loop timer

  receiver_input_channel_1 = 1200;
  receiver_input_channel_2 = 1200;
  receiver_input_channel_3 = 1200;
  receiver_input_channel_4 = 1200;

  //reset all PID controllers
  pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;

  delay(50);
}

void loop() {

  //if (digitalRead(2) == HIGH) {

    receiver_input_channel_1 = 1200;
    receiver_input_channel_2 = 1200;
    receiver_input_channel_3 = 1200;
    receiver_input_channel_4 = 1200;

    //Read the raw acc and gyro data from the MPU-60
    read_mpu_6050_data();

    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_y / 65.5) * 0.3);   //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_x / 65.5) * 0.3);//Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_z / 65.5) * 0.3);      //Gyro pid input is deg/sec.

    //Gyro angle calculations 
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
    angle_roll += angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
    angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
    angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

    if (set_gyro_angles) {                                               //If the IMU is already started
      angle_pitch = angle_pitch * 0.95 + angle_pitch_acc * 0.05;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
      angle_roll = angle_roll * 0.95 + angle_roll_acc * 0.05;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    }
    else {                                                               //At first start
      angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
      angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
      set_gyro_angles = true;                                            //Set the IMU started flag
    }

    //To dampen the pitch and roll angles a complementary filter is used
    angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
    angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

    /*//The PID set point in degrees per second is determined by the roll receiver input.
      //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
      pid_roll_setpoint = 0;
      //We need a little dead band of 16us for better results.
      if (receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
      else if (receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

      pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
      pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

      //The PID set point in degrees per second is determined by the pitch receiver input.
      //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
      pid_pitch_setpoint = 0;
      //We need a little dead band of 16us for better results.
      if (receiver_input_channel_2 > 1508) pid_pitch_setpoint = receiver_input_channel_2 - 1508;
      else if (receiver_input_channel_2 < 1492) pid_pitch_setpoint = receiver_input_channel_2 - 1492;

      pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
      pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

      //The PID set point in degrees per second is determined by the yaw receiver input.
      //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
      pid_yaw_setpoint = 0;
      //We need a little dead band of 16us for better results.
      if (receiver_input_channel_3 > 1050) { //Do not yaw when turning off the motors.
      if (receiver_input_channel_4 > 1508) pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
      else if (receiver_input_channel_4 < 1492) pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
      }*/

    //---------------------------------------------------------------------------------------
    calculate_pid();
    //---------------------------------------------------------------------------------------

    //////////////////////////////////////////////////////////////////////////////////////////////
    throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.
    if (throttle > 1700) throttle = 1700;

    esc_1 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    /*battery_voltage = 1239;
      if (battery_voltage < 1240 && battery_voltage > 800) {                  //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-4 pulse for voltage drop.
      }*/

    if (esc_1 < 1200) esc_1 = 1200;                                         //Keep the motors running.
    if (esc_2 < 1200) esc_2 = 1200;                                         //Keep the motors running.
    if (esc_3 < 1200) esc_3 = 1200;                                         //Keep the motors running.
    if (esc_4 < 1200) esc_4 = 1200;                                        //Keep the motors running.

    if (esc_1 > 1600)esc_1 = 1600;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 1600)esc_2 = 1600;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 1600)esc_3 = 1600;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 1600)esc_4 = 1600;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //Creating the pulses for the ESC's is explained in this video:
    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    //Because of the angle calculation the loop time is getting very important. If the loop time is
    //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
    //that the loop time is still 4000us and no longer! More information can be found on
    //the Q&A page:
    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

    //---------------------------------------------------------------------------------------
    write_LCD();                                                         //Write the roll and pitch values to the LCD display
    //---------------------------------------------------------------------------------------


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

  /*} else {
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);   //Delay 3us to simulate the 250Hz program loop
  }*/
}


//This routine is called every time input 8, 9, 10 or 11 changed state
ISR(PCINT0_vect) {
  //Channel 1=========================================
  if (last_channel_1 == 0 && PINB & B00000001 ) {       //Input 8 changed from 0 to 1
    last_channel_1 = 1;                                 //Remember current input state
    timer_1 = micros();                                 //Set timer_1 to micros()
  }
  else if (last_channel_1 == 1 && !(PINB & B00000001)) { //Input 8 changed from 1 to 0
    last_channel_1 = 0;                                 //Remember current input state
    receiver_input_channel_1 = micros() - timer_1;      //Channel 1 is micros() - timer_1
  }
  //Channel 2=========================================
  if (last_channel_2 == 0 && PINB & B00000010 ) {       //Input 9 changed from 0 to 1
    last_channel_2 = 1;                                 //Remember current input state
    timer_2 = micros();                                 //Set timer_2 to micros()
  }
  else if (last_channel_2 == 1 && !(PINB & B00000010)) { //Input 9 changed from 1 to 0
    last_channel_2 = 0;                                 //Remember current input state
    receiver_input_channel_2 = micros() - timer_2;      //Channel 2 is micros() - timer_2
  }
  //Channel 3=========================================
  if (last_channel_3 == 0 && PINB & B00000100 ) {       //Input 10 changed from 0 to 1
    last_channel_3 = 1;                                 //Remember current input state
    timer_3 = micros();                                 //Set timer_3 to micros()
  }
  else if (last_channel_3 == 1 && !(PINB & B00000100)) { //Input 10 changed from 1 to 0
    last_channel_3 = 0;                                 //Remember current input state
    receiver_input_channel_3 = micros() - timer_3 - 500;      //Channel 3 is micros() - timer_3
  }
  //Channel 4=========================================
  if (last_channel_4 == 0 && PINB & B00001000 ) {       //Input 11 changed from 0 to 1
    last_channel_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if (last_channel_4 == 1 && !(PINB & B00001000)) { //Input 11 changed from 1 to 0
    last_channel_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }
}

void read_mpu_6050_data() {
  //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
  Wire.read() << 8 | Wire.read();                  
  gyro_x = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
}

void write_LCD() {                                                     //Subroutine for writing the LCD

  //To get a 250Hz program loop (4us) it's only possible to write one character per loop
  //Writing multiple characters is taking to much time
  if (lcd_loop_counter == 14)lcd_loop_counter = 0;                     //Reset the counter after 14 characters
  lcd_loop_counter ++;                                                 //Increase the counter
  if (lcd_loop_counter == 1) {
    angle_pitch_buffer = esc_1 * 10;                      //Buffer the pitch angle because it will change
    lcd.setCursor(3, 0);                                               //Set the LCD cursor to position to position 0,0
  }

  if (lcd_loop_counter == 2)lcd.print(abs(angle_pitch_buffer) / 1000); //Print first number
  if (lcd_loop_counter == 3)lcd.print((abs(angle_pitch_buffer) / 100) % 10); //Print second number
  if (lcd_loop_counter == 4)lcd.print((abs(angle_pitch_buffer) / 10) % 10); //Print third number
  if (lcd_loop_counter == 5)lcd.print(".");                            //Print decimal point
  if (lcd_loop_counter == 6)lcd.print(abs(angle_pitch_buffer) % 10);   //Print decimal number

  if (lcd_loop_counter == 7) {
    angle_roll_buffer = esc_3 * 10;
    lcd.setCursor(3, 1);
  }

  if (lcd_loop_counter == 8)lcd.print(abs(angle_roll_buffer) / 1000); //Print first number
  if (lcd_loop_counter == 9)lcd.print((abs(angle_roll_buffer) / 100) % 10); //Print second number
  if (lcd_loop_counter == 10)lcd.print((abs(angle_roll_buffer) / 10) % 10); //Print third number
  if (lcd_loop_counter == 11)lcd.print(".");                           //Print decimal point
  if (lcd_loop_counter == 12)lcd.print(abs(angle_roll_buffer) % 10);   //Print decimal number
}

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > PID_MAX_ROLL) pid_i_mem_roll = PID_MAX_ROLL;
  else if (pid_i_mem_roll < PID_MAX_ROLL * -1) pid_i_mem_roll = PID_MAX_ROLL * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > PID_MAX_ROLL) pid_output_roll = PID_MAX_ROLL;
  else if (pid_output_roll < PID_MAX_ROLL * -1) pid_output_roll = PID_MAX_ROLL * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

  if (pid_i_mem_pitch > PID_MAX_PITCH)pid_i_mem_pitch = PID_MAX_PITCH;
  else if (pid_i_mem_pitch < PID_MAX_PITCH * -1)pid_i_mem_pitch = PID_MAX_PITCH * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > PID_MAX_PITCH)pid_output_pitch = PID_MAX_PITCH;
  else if (pid_output_pitch < PID_MAX_PITCH * -1) pid_output_pitch = PID_MAX_PITCH * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > PID_MAX_YAW) pid_i_mem_yaw = PID_MAX_YAW;
  else if (pid_i_mem_yaw < PID_MAX_YAW * -1) pid_i_mem_yaw = PID_MAX_YAW * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > PID_MAX_YAW)pid_output_yaw = PID_MAX_YAW;
  else if (pid_output_yaw < PID_MAX_YAW * -1) pid_output_yaw = PID_MAX_YAW * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

