#include "SIM900.h"
 
#include <SoftwareSerial.h>
 
#include "inetGSM.h"
 
InetGSM inet;
 

int k=0; 
int j=0;
 
char msg[50];
char msg2[50];
char msg3[50];
char msg4[50];
 
boolean found=false;
 
char data;
 
int numdata;
 
char inSerial[50];
 
int i=0;
 
boolean started=false;



//FOr wind and rain starts
volatile uint16_t count=0;		//Main revolution counter

volatile uint16_t rp4s=0000;	//Revolution per second

//For wind and rain ends


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
#define shutDown 9

float A0_;
float B1_;
float B2_;
float C12_;


//for pressure ends


// For temperature and humidity starts
int analog=0;
int analog2=0;

// For temperature and humidity ends

void setup()
 
{
 
Serial.begin(9600);
init_baro();

init_anemo(  );
 
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

 
}
 
};
 
void loop()
 
{

   sei();
 
 
  init_temp1();
  init_humidity1();

     char reply_text2[10];
 
  sprintf_P(reply_text2,PSTR("hum=%4d"), analog2);

 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/hum.php", reply_text2 ,msg2, 50);


inet.attachGPRS("WAP", "", "");

     char reply_text[10];
 
  sprintf_P(reply_text,PSTR("temp=%4d"), analog);

 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/temp.php", reply_text ,msg, 50);
 
inet.attachGPRS("WAP", "", "");

  float presf = baropPessure();
int pres=floor(presf);
     char reply_text3[10];
 
  sprintf_P(reply_text3,PSTR("baro=%4d"), pres);
Serial.println(presf);
 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/baro.php", reply_text3 ,msg3, 50);



inet.attachGPRS("WAP", "", "");
     char reply_text4[10];
 
  sprintf_P(reply_text4,PSTR("wind=%4d"), rp4s);

 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/wind.php", reply_text4 ,msg4, 200);


 delay(20000);
  
serialswread();
 
};
 
void serialswread(){
 
gsm.SimpleRead();
 
}


ISR(INT0_vect)
{
    //Jumps here automatically when INT0 pin detect a falling edge
    count++;
    // Wait();
}

ISR(TIMER1_COMPA_vect)
{
    //Jumps here every 4 sec exactly!
    rp4s=count;
    count=0;

}



void init_anemo(  )
{

    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0


//INitialisations for anemo
    //TCCR0B|=(1<<CS02); // prescaler fcpu/1024

    EICRA |=(1<<ISC01)|(1<<ISC00);	//Raising edge on INT0 triggers interrupt.
    EICRA |=(1<<ISC11)|(1<<ISC10);	//Raising edge on INT1 triggers interrupt.
    //MCUCR|=(1<<ISC00);
    //MCUCR|=(1<<ISC01)
    EIMSK|=(1<<INT0);
    EIMSK|=(1<<INT1);
    //TCCR1B|=((1<<WGM12)|(1<<CS12)|(1<<CS10));
    TCCR1B|=((1<<WGM12)|(1<<CS12)|(1<<CS10));

    //OCR1A=65530;
    OCR1A = 62499;// = (16*10^6) / (.25*1024) - 1 (must be <65536)
    TIMSK1|=(1<<OCIE1A);
    //sei();


    DDRB|=(1<<DDB1);



}




void init_humidity1( )
{


analog2=analogRead(A1);

}


void init_temp1( )
{

 
analog=analogRead(A0);

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

void baro()
{




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



