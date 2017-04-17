#include <Wire.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
//int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
//unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
//unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

int gyro_x, gyro_y, gyro_z;
long receiver_input_channel_2;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
double pid_error_temp;
int loop_count = 0;
float pid_i_mem_pitch, pid_pitch_setpoint = 0, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;

//////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
//////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int PID_MAX_ROLL = 400;                    //Maximum output of the PID-controller (+/-)
//-------------------------------------------------------------------------------------------
float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = PID_MAX_ROLL;          //Maximum output of the PID-controller (+/-)
//-------------------------------------------------------------------------------------------
float pid_p_gain_yaw = 4.0;                //Gain setting for the yaw P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the yaw I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the yaw D-controller.
int PID_MAX_YAW = 400;                     //Maximum output of the PID-controller (+/-)

////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
////////////////////////////////////////////////////////////////////////////////////////
byte highByte, lowByte;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage, gyro_address; //some stuff
int cal_int, start;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
//-------------------------------------------------------------------------------------------
long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
boolean gyro_angles_set;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  //----------------------------------------------calibration--------------------------------
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                          //Take 2000 readings for calibration.
    if (cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));               //Change the led status to indicate calibration.
    gyro_signalen();                                                        //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.

  //-------------------------------------------------------------------------------------------
  // put your setup code here, to run once:
}

//Main program loop
void loop() {

  //----------------------------------------------***********-------------------------------------------------
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if (set_gyro_angles) {                                               //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else {                                                               //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_x / 65.5) * 0.3);

  calculate_pid();

  //----------------------------------------------***********-------------------------------------------------

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (digitalRead(2) == HIGH) {
    receiver_input_channel_2 = 1250;
    Serial.println("High");
  }
  else
    receiver_input_channel_2 = 1000;

  while (zero_timer + 4000 > micros());                      //Start the pulse after 4000 micro seconds.
  loop_timer = micros();                                     //Reset the zero timer.
  PORTD |= B11110000; //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = receiver_input_channel_2 + zero_timer;   //Calculate the time when digital port 4 is set low.
  timer_channel_2 = receiver_input_channel_2 + zero_timer;   //Calculate the time when digital port 5 is set low.
  timer_channel_3 = receiver_input_channel_2 + zero_timer;   //Calculate the time when digital port 6 is set low.
  timer_channel_4 = receiver_input_channel_2 + zero_timer;   //Calculate the time when digital port 7 is set low.

  while (PORTD >= 16) {                                      //Execute the loop until digital port 8 til 11 is low.
    esc_loop_timer = micros();                               //Check the current time.
    if (timer_channel_1 <= esc_loop_timer) PORTD &= B11101111; //When the delay time is expired, digital port 4 is set low.
    if (timer_channel_2 <= esc_loop_timer) PORTD &= B11011111; //When the delay time is expired, digital port 5 is set low.
    if (timer_channel_3 <= esc_loop_timer) PORTD &= B10111111; //When the delay time is expired, digital port 6 is set low.
    if (timer_channel_4 <= esc_loop_timer) PORTD &= B01111111; //When the delay time is expired, digital port 7 is set low.
  }
  read_mpu_6050_data();
}

void set_gyro_registers() {

  //Setup the MPU-6050
  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
  Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                    //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
  Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                                    //End the transmission with the gyro

  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
  Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();                                                    //End the transmission with the gyro

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
  Wire.write(0x1B);                                                          //Start reading @ register 0x1B
  Wire.endTransmission();                                                    //End the transmission
  Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
  while (Wire.available() < 1);                                              //Wait until the 6 bytes are received
  if (Wire.read() != 0x08) {                                                 //Check if the value is 0x08
    digitalWrite(12, HIGH);                                                  //Turn on the warning led
    while (1)delay(10);                                                      //Stay in this loop for ever
  }

  Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                    //End the transmission with the gyro
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen() {
  //Read the MPU-6050

  Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(gyro_address, 14);                                     //Request 14 bytes from the gyro.

  while (Wire.available() < 14);                                          //Wait until the 14 bytes are received.
  acc_axis[1] = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_x variable.
  acc_axis[2] = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_y variable.
  acc_axis[3] = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the temperature variable.
  gyro_axis[1] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_axis[2] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.
  gyro_axis[3] = Wire.read() << 8 | Wire.read();                          //Read high and low part of the angular data.


  if (cal_int == 2000) {
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid() {

  //Roll calculations--------------------------------------------------------------
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > PID_MAX_ROLL)
    pid_i_mem_roll = PID_MAX_ROLL;
  else if (pid_i_mem_roll < PID_MAX_ROLL * -1)
    pid_i_mem_roll = PID_MAX_ROLL * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > PID_MAX_ROLL)
    pid_output_roll = PID_MAX_ROLL;
  else if (pid_output_roll < PID_MAX_ROLL * -1)
    pid_output_roll = PID_MAX_ROLL * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations--------------------------------------------------------------
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)
    pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)
    pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)
    pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)
    pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;
}
