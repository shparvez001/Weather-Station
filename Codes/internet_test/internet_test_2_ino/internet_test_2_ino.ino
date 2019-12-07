#include "SIM900.h"
 
#include <SoftwareSerial.h>
 
#include "inetGSM.h"
 
InetGSM inet;
 
int k=0;
 
int j=0;
 
char msg[150];
 
boolean found=false;
 
char data;
 
int numdata;
 
char inSerial[50];
 
int i=0;
 
boolean started=false;
 
void setup()
 
{
 
Serial.begin(9600);
 
Serial.println("GSM Shield testing.");
 
if (gsm.begin(9600)){
 
  Serial.println("\nstatus=READY");
 
  started=true;  
 
}
 
else Serial.println("\nstatus=IDLE");
 
if(started){
 
  if (inet.attachGPRS("WAP", "", ""))
 
    Serial.println("status=ATTACHED");
 
  else Serial.println("status=ERROR");
 
  delay(1000);
 
  numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/test_ac.php", "text=243",msg, 50);
  
  //numdata=inet.httpGET("weather.shparvez.net", 80, "/", msg, 100);
 
}
 
};
 
void loop()
 
{
 
serialswread();
 
};
 
void serialswread(){
 
gsm.SimpleRead();
 
}
