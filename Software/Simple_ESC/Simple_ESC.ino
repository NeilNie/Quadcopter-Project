//Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;
int buttonState = 0;
//Setup routine
void setup() {
  pinMode(2, INPUT);
  Serial.begin(9600);
  DDRD |= B11110000;                                 //Configure digital poort 4, 5, 6 and 7 as output

  PORTD |= B11110000;                              //Set digital poort 4, 5, 6 and 7 high.
  delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
  PORTD &= B00001111;                              //Set digital poort 4, 5, 6 and 7 low.
  delay(3);
  Serial.println("Starting");
  start = 0;
  digitalWrite(12, LOW);                             //Turn off the led.
  zero_timer = micros();                             //Set the zero_timer for the first loop.
}

//Main program loop
void loop() {

  buttonState = digitalRead(2);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH){
    receiver_input_channel_2 = 1100;
    Serial.println("H");
  }
  else
    receiver_input_channel_2 = 1000;
receiver_input_channel_2 = 1200;
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
}

