//Code for ATMEGA328
// including temperature and humidity and rain
// Code by Shahadat Hussain Parvez. Mail at shparvez001@gmail.com, http://www.shparvez.net

#include <stdlib.h> 
#include <math.h> 

#include <avr/io.h>
#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <util/delay.h>
#include <LiquidCrystal.h>
#include <SPI.h>
//RTC
#include <Wire.h>
#include "RTClib.h"

RTC_DS1307 RTC;


//GSM includes. Is added as in example
#include "SIM900.h"
#include <SoftwareSerial.h>

#include "sms.h"
SMSGSM sms;


int numdata;
boolean started=false;
char smsbuffer[160];
char n[20];


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

int  m=0,s=0,h=0 ;

byte deg[8] =
{
    0b00011,
    0b00011,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};

//volatile uint16_t count=0;		//Main revolution counter
//volatile uint16_t count2=0;		//Counter for Rain gauze

//volatile uint16_t rpm=0000;	//Revolution per minute
//volatile uint16_t rp4s=0000;	//Revolution per second

double  count=000;
double count2=000;
double rpm=0000;
double rp4s=0000;

double speeds;
double speedskmph;
double tempc = 0;  // variable for holding Celcius temp (floating for decimal points precision)
int tempPin = 0;  // Declaring the Analog input to be 0 (A0) of Arduino board.
double samples[8]; // array to hold 8 samples for Average temp calculation
double maxi = 0,mini = 100; // max/min temperature variables with initial values. LM35 in simple setup only measures Temp above 0.
int i;
double rhsense = 0;
double rh = 0;
double k=0;


//LiquidCrystal lcd(13, 12, 11, 10, 9, 8);
LiquidCrystal lcd(1, 0, 17, 6, 7, 8);


// for board made for control project use LiquidCrystal lcd(8, 7, 10, 11, 12, 13);

void Wait()
{
    uint8_t i;
    for(i=0; i<2; i++)
    {
        _delay_loop_2(0);
    }
}

void Wait2()
{
    uint8_t i;
    for(i=0; i<20; i++)
    {
        _delay_loop_2(0);
    }
}
void setup ()
{



    lcd.createChar(1, deg);
   //Serial.begin(9600); //opens serial port, sets data rate to 9600 bps

    lcd.begin(20, 4);


    lcd.setCursor(0, 0);
    lcd.print("Portable weather Station");
    for (int positionCounter = 0; positionCounter < 20; positionCounter++)
    {
        // scroll one position left:
        lcd.scrollDisplayLeft();
        // wait a bit:
        delay(200);
    }
    //lcd.setCursor(0, 1);
    //lcd.print("Code By SHP");

    Wait2();
    Wait2();

    init_anemo(  );

    init_baro();

    init_rtc();

    //Serial.println("Starting GSM module");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Starting GSM module");
    
         lcd.setCursor(0,1);
    lcd.print("This can take");
    
         lcd.setCursor(0,2);
    lcd.print("some time!!!");
    
             lcd.setCursor(0,3);
    lcd.print("Keep patience!!!");
           

    Wait();
    gsm.begin(2400);



}

void loop()
{

    init_temp();

    init_humidity( );

    lcd_all( );

    sei();
    
    

}

void send_sms()
{
   /* 
    char reply_text[160];



    int count2_r= floor(count2);
    int tempc_r= floor(tempc);
    int speedskmph_r= floor(speedskmph);
    int rh_r= floor(rh);

    int tempc2_r= floor (baroptemp());
    int preskPa_r= floor(baropPessure());
    
    sprintf_P(reply_text,PSTR("Rain:  %3d mm ,\nWind Speed: %3d kmph ,\nTemperature: %3d C ,\nHumidity: %3d percent\nAnother Temparature: %3d C\n Pressure: %4d kPa"),count2_r, speedskmph_r, tempc_r, rh_r, tempc2_r, preskPa_r);
    

  //test code
    int count2_r= floor(count2);
     double tempc2_r= baroptemp();
     //float speedskmph_r= speedskmph;
     //float rh_r= rh;
     double preskPa_r = baropPessure();
    


    sprintf_P(reply_text,PSTR("Rain:  %3d mm ,\nWind Speed: %3.2f kmph ,\nHumidity: %3.2f percent\nTemparature: %3.2f C\n Pressure: %3.2f kPa"),count2, speedskmph,  rh, tempc2_r, preskPa_r);
   
 //   sprintf_P(reply_text,"Rain:  %3d mm ,\nWind Speed: %3f kmph ,\nHumidity: %3f percent\nTemparature: %3f C\n Pressure: %4f kPa",count2, speedskmph,  rh, tempc2_r, preskPa_r);


  
    
    sms.SendSMS("+8801723807161", reply_text);
    Serial.println("\nSMS sent OK");
    //sms.SendSMS("+8801912237361", reply_text);
    // Serial.println("\nSMS sent OK2");
   //  sms.SendSMS("+8801832557272", reply_text);
    //Serial.println("\nSMS sent OK3");
    //sms.SendSMS("+8801550153939", reply_text);
    //Serial.println("\nSMS sent OK4");
   
   */  
    //new type

  
     static char reply_text2[160] ;
 
  

  
  
// temperature
   static char _tempc1[15];
   sprintf(_tempc1, "Temperature: ");
     
    char _tempc2[10];
    double tempc2=baroptemp();
  dtostrf(tempc2,2,2,_tempc2);
        static char _tempc3[15];
   sprintf(_tempc3, "C\n");  
   
// humidity   
      static char _rh1[15];
   sprintf(_rh1, "Humidity: ");
     
    char _rh2[10];
  dtostrf(rh,2,2,_rh2);
      static char _rh3[15];
   sprintf(_rh3, "percent\n");  
   
   //pressure
         static char _pr1[15];
   sprintf(_pr1, "Pressure: ");
     
    char _pr2[10];
    double presh=baropPessure();
  dtostrf(presh,2,2,_pr2);
      static char _pr3[15];
   sprintf(_pr3, "KPa\n");  
   
   // wind
         static char _w1[15];
   sprintf(_w1, "Wind: ");
     
    char _w2[10];
  dtostrf(speedskmph,2,2,_w2);
      static char _w3[15];
   sprintf(_w3, "kmph\n");  



 //Rain  
         static char _ra1[15];
   sprintf(_ra1, "Rain: ");
     
    char _ra2[10];
  dtostrf(count2,2,2,_ra2);
      static char _ra3[15];
   sprintf(_ra3, "mm\n");  
   
   

   
   
   
    
    memcpy(reply_text2, _tempc1, 13);
 // *(reply_text2+11) = ',';
  memcpy(reply_text2+13, _tempc2, 5);
  memcpy(reply_text2+18, _tempc3, 2);
  
  memcpy(reply_text2+20, _rh1, 10);
  memcpy(reply_text2+30, _rh2, 5);
  memcpy(reply_text2+35, _rh3, 8);
  
  memcpy(reply_text2+43, _pr1, 10);
  memcpy(reply_text2+53, _pr2, 5);
  memcpy(reply_text2+58, _pr3, 4);
  
  memcpy(reply_text2+62, _w1, 6);
  memcpy(reply_text2+68, _w2, 4);
  memcpy(reply_text2+72, _w3, 5);
  
  memcpy(reply_text2+77, _ra1, 6);
  memcpy(reply_text2+83, _ra2, 4);
  memcpy(reply_text2+87, _ra3, 3);
  

  

   
   // sprintf_P(reply_text2,PSTR("Rain:  %3d mm  \nTemparature: %6s C\n "),count2, _tempc2);
   
    sms.SendSMS("+8801723807161", reply_text2);
    Serial.println("\nSMS sent OK");


  
}

ISR(INT0_vect)
{
    //Jumps here automatically when INT0 pin detect a falling edge
    count++;
    // Wait();
}
ISR(INT1_vect)
{
    //Jumps here automatically when INT1 pin detect a falling edge
    count2++;
    // Wait();
}

ISR(TIMER1_COMPA_vect)
{
    //Jumps here every 4 sec exactly!
    rp4s=count;
    rpm=rp4s*60*.25;
    speeds= rp4s*3.14*.27*.25;
    speedskmph= rp4s*3.14*.27*.25*3.6*1.1;

    //lcd_anemo();
    // lcd_all( );

    count=0;

//      cli(); //Disables interrupt




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


void init_temp( )
{

    //      tempc=( 4 * analogRead(tempPin) * 100.0) / 1024.0;


    // Start of calculations FOR loop.
    for(i = 0; i<=7; i++) // gets 8 samples of temperature
    {
        samples[i] = ( 4.25 * analogRead(tempPin) * 100.0) / 1024.0; // conversion math of LM35 sample to readable temperature and stores result to samples array. 1024 is the Bit depth (quantization) of Arduino.
// 5 is the supply volts of LM35. Change appropriatelly to have correct measurement. My case is 4.4Volts.



        tempc = tempc + samples[i]; // do the addition for average temperature
        delay(80); // wait 800ms
    }

    tempc = tempc/8.0; // calculated the averare of 8 samples in Celcius



}


void init_humidity( )
{

    rhsense = (analogRead(A1));
    k=(rhsense*5)/1024;

    // rh = ((30.855*(rhsense/204.6))-11.504);
    rh=4.8008*k*k*k*k*k-47.772*k*k*k*k+ 184.64*k*k*k-343.78*k*k+334.28* k + -110.6 ;



}

void init_rtc()
{

    Wire.begin();
    RTC.begin();

}


void lcd_anemo(  )
{

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Count ");
    lcd.setCursor(0, 1);
    lcd.print("SPEED");

    lcd.setCursor(9, 0);
    lcd.print(count);
    lcd.setCursor(7, 1);
    lcd.print(speeds);
    lcd.setCursor(13, 1);
    lcd.print("m/S");

    count=0;

}

void lcd_all( )
{

    //change it to other place with function

    DateTime now = RTC.now();

    if ((now.minute()==00 ) && (now.hour()==00))

    {
        count2=0;
        delay(60000);   
    }
    
    
    


    s= now.second(), DEC;
    m= now.minute(), DEC;
    h= now.hour(), DEC;
    
    if (m%10==0)
    {
        lcd.clear();
             lcd.setCursor(0,0);
    lcd.print("Sending SMS");
    
             lcd.setCursor(0,1);
    lcd.print("This can take");
    
         lcd.setCursor(0,2);
    lcd.print("some time!!!");
    
             lcd.setCursor(0,3);
    lcd.print("Keep patience!!!");
    
      send_sms();
      
      lcd.clear();
              lcd.setCursor(0,0);
    lcd.print("SMS Sent");
                  lcd.setCursor(0,1);
    lcd.print("   ");
     lcd.setCursor(0,2);
    lcd.print("Showing Current datas");
    
         lcd.setCursor(0,3);
    lcd.print("In a minute...");
          delay(50000);     
      
        

        
    
    }

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print(h);
    lcd.setCursor(2,0);
    lcd.print(":");
    lcd.setCursor(3, 0);
    lcd.print(m);
    lcd.setCursor(5,0);
    lcd.print("   | BY EEESUST");
  //  lcd.print(":");
    //lcd.setCursor(6, 0);
    //lcd.print(s);




    lcd.setCursor(0, 1);
    lcd.print("Rain    :");
    lcd.setCursor(10, 1);
    lcd.print(count2);
    lcd.setCursor(15, 1);
    lcd.print("mm");


    lcd.setCursor(0, 2);
    lcd.print("TEMP    :");

    lcd.setCursor(10, 2);
    lcd.print(tempc);

    lcd.setCursor(15, 2);
    lcd.write(1);
    lcd.setCursor(16, 2);
    lcd.print("C");

    lcd.setCursor(0, 3);
    lcd.print("HUMIDITY:");
    lcd.setCursor(10, 3);
    lcd.print(rh);
    lcd.setCursor(16, 3);
    lcd.print(" %");

    delay(1000);


    lcd.clear();


    lcd.setCursor(0, 0);
    lcd.print(h);
    lcd.setCursor(2,0);
    lcd.print(":");
    lcd.setCursor(3, 0);
    lcd.print(m);
    lcd.setCursor(5,0);
       
    lcd.print("   | BY EEESUST");
  //  lcd.print(":");
   // lcd.setCursor(6, 0);
   // lcd.print(s);

    lcd.setCursor(0, 1);
    lcd.print("WIND    :");

    lcd.setCursor(10, 1);
    lcd.print(speedskmph);
    lcd.setCursor(16, 1);
    lcd.print("Km/h");


    lcd.setCursor(0, 2);
    lcd.print("TEMP2   :");

    lcd.setCursor(10, 2);
    lcd.print(baroptemp());

    lcd.setCursor(15, 2);
    lcd.write(1);
    lcd.setCursor(16, 2);
    lcd.print("C");

    lcd.setCursor(0, 3);
    lcd.print("Pressure:");
    lcd.setCursor(10, 3);
    lcd.print(baropPessure());
    lcd.setCursor(16, 3);
    lcd.print(" KPa");

    delay(1000);

    // Wait2();
}


// Not using the following function
void sms_text ()
{

    char reply_text[160];



    int count2_r= floor(count2);
    int tempc_r= floor(tempc);
    int speedskmph_r= floor(speedskmph);
    int rh_r= floor(rh);

    int tempc2_r= floor (baroptemp());
    int preskPa_r= floor(baropPessure());
    

    /*
     int count2_r= count2;
     int tempc_r= tempc;
     int speedskmph_r= speedskmph;
     int rh_r= rh;
     */
    sprintf_P(reply_text,PSTR("R_Count:  %3d ,\nWind Speed: %3d kmph ,\nTemperature: %3d C ,\nHumidity: %3d percent\nAnother Temparature: %3d C\n Pressure: %4d kPa"),count2_r, speedskmph_r, tempc_r, rh_r, tempc2_r, preskPa_r);


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


float baroptemp()
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


    float tempc2 = temp*  (145.0/1023.0)-40.0;

    return(tempc2);
}




