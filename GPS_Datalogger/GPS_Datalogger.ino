/*
Connecting diagram
MOSI           - pin 11
MISO           - pin 12
CLK            - pin 13
CS             - pin 4
+5V            - +5V
GND            - GND
LED + resistor - pin 6
Switch         - pin 7
 	 
*/

#include <SD.h>

const int chipSelect = 4;
int start_log, start, file_number;
byte c;
File dataFile;
char file_name[10] = "logxx.ubx";

void setup()
{
  Serial.begin(57600);       //Set the correct log baud rate
  pinMode(6, OUTPUT);        //Led
  pinMode(7, INPUT_PULLUP);  //Switch
  
  if (!SD.begin(chipSelect)) {
    //Card error or card is missing.
    while(1){
      digitalWrite(6, !digitalRead(6));
      delay(100);
    }
  }
}

void loop(){
  //Prevent contact bouncing / slow switching.
  if(PIND & B10000000 && start_log < 1000)start_log ++;
  else if(start_log > 0)start_log --;
  
  if(start_log > 700 && start == 0){
    for(start = 0; start < 7; start ++){
      digitalWrite(6, !digitalRead(6));
      delay(250);
    }
    file_name[3] = '0';
    file_name[4] = '0';
    //Open a new ubx file after the last file on the card.
    for(file_number = 0; file_number < 100 && !dataFile; file_number ++){
      file_name[3] = (file_number / 10) + 48;
      file_name[4] = (file_number % 10) + 48;
      if(!SD.exists(file_name))dataFile = SD.open(file_name, FILE_WRITE);
    }
    
    if (dataFile){
      digitalWrite(6, HIGH);
      start = 1;
      while(Serial.available())c = Serial.read(); //Clear serial  buffer.
    }
    else{ //File is not open for writing.
      while(1){
        digitalWrite(6, !digitalRead(6));
        delay(100);
      }
    }
  }
  
  if(start){
    if (Serial.available()){
      c = Serial.read();
      dataFile.print((char)c);
    }
    else if(start_log < 200){
      dataFile.close();
      if(!dataFile)digitalWrite(6, LOW);
      start = 0;
    }
  }
}

