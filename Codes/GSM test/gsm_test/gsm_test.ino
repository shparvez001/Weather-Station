#include <call.h>
#include <gps.h>
#include <GSM.h> 
#include <HWSerial.h>
#include <inetGSM.h> 
#include <LOG.h> 
#include <SIM900.h> 
#include <sms.h> 
#include <Streaming.h> 
#include <WideTextFinder.h> 
#include "SIM900.h" 
#include <SoftwareSerial.h> 
#include "sms.h"
SMSGSM sms;
boolean started; 
void setup() {
  Serial.begin(9600); 
Serial.println("GSM Testing to send SMS"); 
if (gsm.begin(2400)){ Serial.println("\nstatus=READY");
started=true; } 
else Serial.println("\nstatus=IDLE"); 
//if(started){ if (sms.SendSMS("+8801712280548", "Arduino SMS"))
if(started){ if (sms.SendSMS("+8801723807161", "Test SMS Done!!!"))
// number to which you want to send the sms and the sms text// Serial.println("\nSMS sent OK"); 
}
