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


//for pressure starts

#include <SPI.h>

#define PRESH   0x80
#define   PRESL   0x82
#define   TEMPH   0x84
#define   TEMPL   0x86

#define A0MSB   0x88
#define A0LSB   0x8A
#define B1MSB   0x8C
#define B1LSB   0x8E
#define   B2MSB   0x90
#define B2LSB   0x92
#define C12MSB   0x94
#define   C12LSB   0x96

#define CONVERT   0x24   

#define chipSelectPin 10
#define shutDown 7

float A0_;
float B1_;
float B2_;
float C12_;

//for pressure ends



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

init_baro();
 
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
 
 // numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/test_ac.php", "text=241",msg, 50);
  

 
}
 
};
 
void loop()
 
{

  
 
 //Sending Only Temperature data to a server
 
  float presf = baropPessure();
int pres=floor(presf);
     char reply_text3[20];
 
  sprintf_P(reply_text3,PSTR("baro=%4d"), pres);

 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/baro.php", reply_text3 ,msg2, 200);


 
 
 




 delay(20000);
  
serialswread();
 
};
 
void serialswread(){
 
gsm.SimpleRead();
 
}


void init_baro()
{

    // start the SPI library:
    SPI.begin();

    // initalize the data ready and chip select pins:
    pinMode(shutDown, OUTPUT);
    digitalWrite(shutDown, HIGH);
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);
    delay (10);

    // read registers that contain the chip-unique parameters to do the math
    unsigned int A0H = readRegister(A0MSB);
    unsigned int A0L = readRegister(A0LSB);
    A0_ = (A0H << 5) + (A0L >> 3) + (A0L & 0x07) / 8.0;

    unsigned int B1H = readRegister(B1MSB);
    unsigned int B1L = readRegister(B1LSB);
    B1_ = ( ( ( (B1H & 0x1F) * 0x100)+B1L) / 8192.0) - 3 ;

    unsigned int B2H = readRegister(B2MSB);
    unsigned int B2L = readRegister(B2LSB);
    B2_ = ( ( ( (B2H - 0x80) << 8) + B2L) / 16384.0 ) - 2 ;

    unsigned int C12H = readRegister(C12MSB);
    unsigned int C12L = readRegister(C12LSB);
    C12_ = ( ( ( C12H * 0x100 ) + C12L) / 16777216.0 )  ;

}

//Read registers
unsigned int readRegister(byte thisRegister )
{
    unsigned int result = 0;   // result to return
    digitalWrite(chipSelectPin, LOW);
    delay(10);
    SPI.transfer(thisRegister);
    result = SPI.transfer(0x00);
    digitalWrite(chipSelectPin, HIGH);
    return(result);
}

//read pressure
float baropPessure()
{
    digitalWrite(chipSelectPin, LOW);
    delay(3);
    SPI.transfer(0x24);
    SPI.transfer(0x00);
    digitalWrite(chipSelectPin, HIGH);
    delay(3);
    digitalWrite(chipSelectPin, LOW);
    SPI.transfer(PRESH);
    unsigned int presH = SPI.transfer(0x00);
    delay(3);
    SPI.transfer(PRESL);
    unsigned int presL = SPI.transfer(0x00);
    delay(3);
    SPI.transfer(TEMPH);
    unsigned int tempH = SPI.transfer(0x00);
    delay(3);
    SPI.transfer(TEMPL);
    unsigned int tempL = SPI.transfer(0x00);
    delay(3);
    SPI.transfer(0x00);
    delay(3);
    digitalWrite(chipSelectPin, HIGH);

    unsigned long press = ((presH *256) + presL)/64;
    unsigned long temp  = ((tempH *256) + tempL)/64;

    float pressure = A0_+(B1_+C12_*temp)*press+B2_*temp;
    float preskPa = pressure*  (65.0/1023.0)+50.0;

    return(preskPa);
}


