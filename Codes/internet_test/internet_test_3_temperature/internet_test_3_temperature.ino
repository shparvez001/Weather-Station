#include "SIM900.h"
 
#include <SoftwareSerial.h>
 
#include "inetGSM.h"
 
InetGSM inet;
 

int k=0; 
int j=0;
 
char msg[50];
char msg2[50];
 
boolean found=false;
 
char data;
 
int numdata;
 
char inSerial[50];
 
int i=0;
 
boolean started=false;


int analog=0;
int analog2=0;


float tempc = 0;  // variable for holding Celcius temp (floating for decimal points precision)
int tempPin = 0;  // Declaring the Analog input to be 0 (A0) of Arduino board.
float samples[8]; // array to hold 8 samples for Average temp calculation
float maxi = 0,mini = 100; // max/min temperature variables with initial values. LM35 in simple setup only measures Temp above 0.

float rhsense = 0;
float rh = 0;
float k1=0;


void setup()
 
{
 
Serial.begin(9600);
 
Serial.println("GSM Shield testing.");
 
if (gsm.begin(2400)){
 
  Serial.println("\nstatus=READY");
 
  started=true;  
 
}
 
else Serial.println("\nstatus=IDLE");
 
if(started){
 
  if (inet.attachGPRS("WAP", "", ""))
 
    Serial.println("status=ATTACHED");
 
  else Serial.println("status=ERROR");
 
  delay(1000);
 
 // numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/test_ac.php", "text=241",msg, 50);
  

 
}
 
};
 
void loop()
 
{
/*
sending both fumidity and temperature in a way
    init_temp1();
    init_humidity1();

 
  
       char reply_text[160];
       
         sprintf_P(reply_text,PSTR("temp=%4d&humidity=%4d"), analog, analog2);
       
 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/combo.php", reply_text ,msg, 200);
 */    
  
 
 //Sending Only Temperature data to a server
 
  init_temp1();
  init_humidity1();

     char reply_text2[20];
 
  sprintf_P(reply_text2,PSTR("hum=%4d"), analog2);

 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/hum.php", reply_text2 ,msg2, 200);


inet.attachGPRS("WAP", "", "");

     char reply_text[20];
 
  sprintf_P(reply_text,PSTR("temp=%4d"), analog);

 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/temp.php", reply_text ,msg, 200);
 
 
 
 



 /*
 
 //Sending temperature and humidity data to a server in a different way
 
 
 static char reply_text2[160] ;
  //Get Temperature and humidity data
  init_temp();
  init_humidity( );
  
  // temperature
   static char _tempc1[15];
   sprintf(_tempc1, "temp=");
     
    char _tempc2[10];
    double tempc2=tempc;
  dtostrf(tempc2,2,2,_tempc2);

   
// humidity   
      static char _rh1[15];
   sprintf(_rh1, "&humidity=");
     
    char _rh2[10];
  dtostrf(rh,2,2,_rh2);
        
  
 // the data to send to server
 
     memcpy(reply_text2, _tempc1, 5);
 // *(reply_text2+11) = ',';
  memcpy(reply_text2+5, _tempc2, 5);
  
  memcpy(reply_text2+10, _rh1, 10);
  memcpy(reply_text2+20, _rh2, 5);
 
 
 
 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/combo.php", reply_text2 ,msg, 200);
 
 */
 delay(20000);
  
serialswread();
 
};
 
void serialswread(){
 
gsm.SimpleRead();
 
}

void init_humidity1( )
{


analog2=analogRead(A1);

}


void init_temp1( )
{

     tempc = ( 4.25 * analogRead(tempPin) * 100.0) / 1024.0; // conversion math of LM35 sample to readable temperature and stores result to samples array. 1024 is the Bit depth (quantization) of Arduino.

analog=analogRead(tempPin);

}


/*
void init_temp( ) {

     //      tempc=( 4 * analogRead(tempPin) * 100.0) / 1024.0;          
  
for(i = 0;i<=7;i++){
samples[i] = 20;
}  
     
     
     // Start of calculations FOR loop.
for(i = 0;i<=7;i++){ // gets 8 samples of temperature
samples[i] = (4.2 * analogRead(tempPin) * 100.0) / 1024.0; 

if ((samples[i] - samples[i-1])>35) 
{
  i--;
    
    delay(500);

continue;
}

tempc = tempc + samples[i]; // do the addition for average temperature
delay(120); // wait 80ms
}

tempc = tempc/8.0; // calculated the averare of 8 samples in Celcius
       
 

}


void init_humidity( ) {
  
  for(i = 0;i<=7;i++){
samples[i] = 100;
} 
  
  for(i = 0;i<=7;i++){ // gets 8 samples of temperature
  
  
   rhsense = (analogRead(A2));
           k1=(rhsense*5)/1024;   
      
          // rh = ((30.855*(rhsense/204.6))-11.504);
//          rh=4.8008*k*k*k*k*k-47.772*k*k*k*k+ 184.64*k*k*k-343.78*k*k+334.28* k + -110.6 ;     
  
  
samples[i] = 4.8008*k1*k1*k1*k1*k1-47.772*k1*k1*k1*k1+ 184.64*k1*k1*k1-343.78*k1*k1+334.28* k1 + -110.6 ;

if ((samples[i] - samples[i-1])>95) 
{
  i--;
      
    delay(500);
continue;
}
rh = rh + samples[i]; // do the addition for average HUmidity
delay(120); // wait 800ms
}

rh = rh/8.0; 
       
 

}

*/



