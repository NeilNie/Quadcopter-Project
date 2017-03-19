#include <Wire.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_pitch = 1.3;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.04;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 18.0;  //Gain setting for the pitch D-controller.

const float PID_MAX = 400;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
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

//Setup routine
void setup() {

  Serial.begin(57500);
  pinMode(2, INPUT);

  DDRD |= B01100000;                                 //Configure digital poort 4, 5, 6 and 7 as output
  PORTD |= B01100000;                              //Set digital poort 4, 5, 6 and 7 high.
  delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
  PORTD &= B10011111;                              //Set digital poort 4, 5, 6 and 7 low.
  delay(3);
  Serial.println("Calibrating");
  start = 0;
  digitalWrite(12, LOW);                             //Turn off the led.
  zero_timer = micros();                             //Set the zero_timer for the first loop.

  Wire.begin();                                                        //Start I2C as master
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
    if (cal_int % 125 == 0)Serial.print(".");                             //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset                                            //Print text to screen
  Serial.println("Done");

  loop_timer = micros();
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
  if (digitalRead(2) == HIGH){
    receiver_input_channel_2 = 1250;
    Serial.println("High");
  }
  else
    receiver_input_channel_2 = 1000;

  while (zero_timer + 4000 > micros());                      //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                     //Reset the zero timer.
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

void calculate_pid() {

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
}

void read_mpu_6050_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable

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


