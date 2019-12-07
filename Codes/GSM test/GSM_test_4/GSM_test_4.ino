#include "SIM900.h"
#include <SoftwareSerial.h>
//If not used, is better to exclude the HTTP library,
//for RAM saving.
//If your sketch reboots itself proprably you have finished,
//your memory available.
//#include "inetGSM.h"

//If you want to use the Arduino functions to manage SMS, uncomment the lines below.
#include "sms.h"
#include "call.h"
SMSGSM sms;
CallGSM call;

//To change pins for Software Serial, use the two lines in GSM.cpp.

//GSM Shield for Arduino
//www.open-electronics.org
//this code is based on the example of Arduino Labs.




char number[20];
byte stat=0;
int value=0;
int pin=1;
char value_str[5];


int tempc = 0;  // variable for holding Celcius temp (floating for decimal points precision)
int tempPin = 0;  // Declaring the Analog input to be 0 (A0) of Arduino board.
int numdata;
boolean started=false;
char smsbuffer[160];
char n[20];
float rhsense = 0;
float rh = 0;
float k=0;

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
      //Serial.println("\nSMS sent OK");
  }

};

void loop() 
{
  
  
    //Chekcs status of call
  stat=call.CallStatusWithAuth(number,1,7);
  //If the incoming call is from an authorized number
  //saved on SIM in the positions range from 1 to 3.
  if(stat==CALL_INCOM_VOICE_AUTH){
    //Hang up the call.
    call.HangUp();
    delay(2000);
 tempc=( 4 * analogRead(tempPin) * 100.0) / 1024.0; 
  
             rhsense = (analogRead(A1));
           k=(rhsense*5)/1024;   
      
          // rh = ((30.855*(rhsense/204.6))-11.504);
          rh=4.8008*k*k*k*k*k-47.772*k*k*k*k+ 184.64*k*k*k-343.78*k*k+334.28* k + -110.6 ;     
            
 int rh2= floor(rh);
 

 char reply_text[160];
 char tempc2[6];
 //char rh2[6];
   dtostrf(tempc,6, 2, tempc2);
 //  dtostrf(rh,6, 2, rh2);
   				
sprintf_P(reply_text,PSTR("Current Temperature: %3d C and relative humidity is %3d percent"), tempc, rh2);

  sms.SendSMS(number, reply_text);
  //sms.SendSMS("+8801912237361", reply_text);
  Serial.println(reply_text);
  Serial.println("\nSMS sent OK");




  }
  delay(1000);
  
 
  
}
