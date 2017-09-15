//Declaring Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

//Setup routine
void setup() {
  Serial.begin(9600);
  DDRD |= B11110000;                                 //Configure digital poort 4, 5, 6 and 7 as output

  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change

  //  / Wait until the receiver is active and the throtle is set to the lower position.
  //  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
  //    start ++;                                        //While waiting increment start whith every loop.
  //    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
  //    PORTD |= B11110000;                              //Set digital poort 4, 5, 6 and 7 high.
  //    delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
  //    PORTD &= B00001111;                              //Set digital poort 4, 5, 6 and 7 low.
  //    delay(3);                                        //Wait 3 milliseconds before the next loop.
  //    if (start == 125) {                              //Every 125 loops (500ms).
  //      digitalWrite(12, !digitalRead(12));            //Change the led status.
  //      start = 0;                                     //Start again at 0.
  //    }
  //  }
  PORTD |= B11110000;                              //Set digital poort 4, 5, 6 and 7 high.
  delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
  PORTD &= B00001111;                              //Set digital poort 4, 5, 6 and 7 low.
  delay(3);
  Serial.println("Starting");
digitalWrite(12, LOW);                             //Turn off the led.
  zero_timer = micros();                             //Set the zero_timer for the first loop.
}

//Main program loop
void loop() {

  while (zero_timer + 4000 > micros());                      //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                     //Reset the zero timer.

  PORTD |= B11110000; //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 4 is set low.
  timer_channel_2 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 5 is set low.
  timer_channel_3 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 6 is set low.
  timer_channel_4 = receiver_input_channel_3 + zero_timer;   //Calculate the time when digital port 7 is set low.

  while (PORTD >= 16) {                                      //Execute the loop until digital port 8 til 11 is low.
    esc_loop_timer = micros();                               //Check the current time.
    if (timer_channel_1 <= esc_loop_timer) PORTD &= B11101111; //When the delay time is expired, digital port 4 is set low.
    if (timer_channel_2 <= esc_loop_timer) PORTD &= B11011111; //When the delay time is expired, digital port 5 is set low.
    if (timer_channel_3 <= esc_loop_timer) PORTD &= B10111111; //When the delay time is expired, digital port 6 is set low.
    if (timer_channel_4 <= esc_loop_timer) PORTD &= B01111111; //When the delay time is expired, digital port 7 is set low.
  }
  print_signals();
}

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

//Subroutine for displaying the receiver signals
void print_signals() {
  Serial.print("Roll:");
  if (receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
  else if (receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_1);

  Serial.print("  Nick:");
  if (receiver_input_channel_2 - 1520 > 0)Serial.print("^^^");
  else if (receiver_input_channel_2 - 1480 < 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Gas:");
  if (receiver_input_channel_3 - 1480 < 0)Serial.print("vvv");
  else if (receiver_input_channel_3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Yaw:");
  if (receiver_input_channel_4 - 1480 < 0)Serial.print("<<<");
  else if (receiver_input_channel_4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input_channel_4);
}
