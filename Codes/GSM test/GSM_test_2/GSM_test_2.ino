#include "SIM900.h"
#include <SoftwareSerial.h>
//If not used, is better to exclude the HTTP library,
//for RAM saving.
//If your sketch reboots itself proprably you have finished,
//your memory available.
//#include "inetGSM.h"

//If you want to use the Arduino functions to manage SMS, uncomment the lines below.
#include "sms.h"
SMSGSM sms;

//To change pins for Software Serial, use the two lines in GSM.cpp.

//GSM Shield for Arduino
//www.open-electronics.org
//this code is based on the example of Arduino Labs.

//Simple sketch to send and receive SMS.

int tempc = 0;  // variable for holding Celcius temp (floating for decimal points precision)
int tempPin = 0;  // Declaring the Analog input to be 0 (A0) of Arduino board.
int numdata;
boolean started=false;
char smsbuffer[160];
char n[20];

void setup() 
{
  //Serial connection.
  Serial.begin(9600);
  Serial.println("GSM Shield testing.");
  //Start configuration of shield with baudrate.
  //For http uses is raccomanded to use 4800 or slower.
  if (gsm.begin(2400)){
    Serial.println("\nstatus=READY");
    started=true;  
  }
  else Serial.println("\nstatus=IDLE");
  
  if(started){
    //Enable this two lines if you want to send an SMS.
    //if (sms.SendSMS("+8801723807161", "Arduino SMS"))
     // Serial.println("\nSMS sent OK");
  }

};

void loop() 
{
  tempc=( 4 * analogRead(tempPin) * 100.0) / 1024.0; 
 
 
 char reply_text[40];
				
sprintf_P(reply_text,PSTR("Current Temperature: %02d C"),tempc);
 
  sms.SendSMS("+8801723807161", reply_text);
  //sms.SendSMS("+8801912237361", reply_text);
  Serial.println("\nSMS sent OK");
    delay(1000000);
  
}
