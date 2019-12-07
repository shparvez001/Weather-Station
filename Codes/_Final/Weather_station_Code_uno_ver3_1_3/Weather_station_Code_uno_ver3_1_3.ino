//Code for ATMEGA328
// including
// Code by Shahadat Hussain Parvez. Mail at shparvez001@gmail.com, http://www.shparvez.net
//#include pgmspace.h
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>
#include <SPI.h>

#include <Wire.h>


//Defining all address and mast for DS1307 Check datasheet for the details
#define	DS1307_ID	0x68	// I2C ID for DS1307
#define	RAM_BASE_READ	0	// smallest read address
#define	RAM_BASE_WRITE	8	// smallest write address

// Addresses for the parts of the date/time in RAM
//#define	ADDR_SEC	0x0
#define	ADDR_MIN	0x1
#define	ADDR_HR		0x2



// Address for the special control bytes
#define	ADDR_CTL_CH	0x0
#define	ADDR_CTL_12H	0x2
#define	ADDR_CTL_OUT	0x7
#define	ADDR_CTL_SQWE	0x7
#define	ADDR_CTL_RS	0x7

// Bit masks for the control/testable bits
#define	CTL_CH		0x80
#define	CTL_12H		0x40
#define	CTL_PM		0x20
#define	CTL_OUT		0x80
#define	CTL_SQWE	0x10
#define	CTL_RS		0x03



// Define a global buffer we can use in thses functions
#define	MAX_BUF		8			// time message is the biggest message we need to handle (7 bytes)
uint8_t	bufRTC[MAX_BUF];

//End of RTC  definations

// Variables for reading and writing data on RTC


uint8_t h=0;		// hour of the day (1-12) or (0-23) depending on the mode
uint8_t m=0;		// minutes past the hour (0-59)
//uint8_t s=0;		// Seconds past the minute (0-59)
uint8_t	pm;		// set to non-zero if 12 hour clock and PM indicator

// ENd for variable of time






//GSM includes. Is added as in example
#include "SIM900.h"
#include <SoftwareSerial.h>

#include "inetGSM.h"

InetGSM inet;



boolean found=false;
char data;
char inSerial[50];
boolean started=false;




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


volatile uint16_t count=0;		//Main revolution counter
volatile uint16_t count2=0;		//Counter for Rain gauze


volatile uint16_t rp4s=0;	//Revolution per second


double speedskmph;
int tempc = 0;  // variable for holding analog value of  Celcius temp (floating for decimal points precision)
float tempcLm35=0;
int samples[8]; // array to hold 8 samples for Average temp calculation
int i=0;
int rhsense = 0;
float rh = 0;
float k=0;




//LCD Connection as in the bord made
LiquidCrystal lcd(1, 0, 17, 6, 7, 8);



//Main Setups
void setup ()

{
  
  Wire.begin();

    lcd.createChar(1, deg);

    // Serial.begin(9600); //opens serial port, sets data rate to 9600 bps

    lcd.begin(20, 4);

/*
    lcd.setCursor(0, 0);
    lcd.print(F("Portable weather Station"));
    
    for (int positionCounter = 0; positionCounter < 20; positionCounter++)
    {
        // scroll one position left:
        lcd.scrollDisplayLeft();
        // wait a bit:
        delay(200);
    }
    */
    //lcd.setCursor(0, 1);
    //lcd.print("Code By SHP");

   // delay(500);

    // Initializing of all sensors needed specifically Anemometer (Timer and interrupt), Barometer, RTC
    init_anemo(  );
    init_baro();
    //init_rtc();


    //Print for starting GSM module


    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Starting GSM module"));

      lcd.setCursor(0,1);
    lcd.print(F("This can take"));

    lcd.setCursor(0,2);
    lcd.print(F("some time!!!"));

    lcd.setCursor(0,3);
    lcd.print(F("Keep patience!!!"));


    delay(80);
    gsm.begin(2400);

    delay(80);
    
    


  sei();

}


void loop()
{


    
    find_temp();

    find_humidity( );
    
    ReadTime();

    lcd_all( );





}


void lcd_all( )
{


    if ((m==00 ) && (h==00))

    {
        count2=0;
        delay(60000);
    }



    if (m%5==0)
    {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F("Uploading data"));


      lcd.setCursor(0,1);
    lcd.print(F("This can take"));

    lcd.setCursor(0,2);
    lcd.print(F("some time!!!"));

    lcd.setCursor(0,3);
    lcd.print(F("Keep patience!!!"));

    delay(80);
        
        upload2();

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(F("Data uploaded"));
        //lcd.setCursor(0,1);
        //lcd.print(F("   "));

        lcd.setCursor(0,2);
        lcd.print(F("Showing Current data"));

        lcd.setCursor(0,3);
        lcd.print(F("In a minute..."));
        delay(5000);


    }


    

      lcd.clear();


    lcd.setCursor(0, 0);
    lcd.print(h);
    lcd.setCursor(2,0);
    lcd.print(F(":"));
    lcd.setCursor(3, 0);
    lcd.print(m);
    lcd.setCursor(5,0);
    lcd.print(F("   | BY EEESUST"));



    lcd.setCursor(0, 1);
    lcd.print(F("Rain    :"));
    lcd.setCursor(10, 1);
    lcd.print(count2);
    lcd.setCursor(15, 1);
    lcd.print(F("mm"));

    lcd.setCursor(0, 2);
    lcd.print(F("TEMP    :"));

    lcd.setCursor(10, 2);
    lcd.print(tempcLm35);

    lcd.setCursor(15, 2);
    lcd.write(1);
    lcd.setCursor(16, 2);
    lcd.print(F("C")); 


    lcd.setCursor(0, 3);
    lcd.print(F("HUMIDITY:"));
    lcd.setCursor(10, 3);
    lcd.print(rh);
    lcd.setCursor(16, 3);
    lcd.print(F(" %"));

    delay(1200);


      lcd.clear();


    lcd.setCursor(0, 0);
    lcd.print(h);
    lcd.setCursor(2,0);
    lcd.print(F(":"));
    lcd.setCursor(3, 0);
    lcd.print(m);
    lcd.setCursor(5,0);
    lcd.print(F("   | BY EEESUST"));


    lcd.setCursor(0, 1);
    lcd.print(F("WIND    :"));

    lcd.setCursor(10, 1);
    lcd.print(speedskmph);
    lcd.setCursor(16, 1);
    lcd.print(F("Km/h"));


     lcd.setCursor(0, 2);
    lcd.print(F("TEMP    :"));

    lcd.setCursor(10, 2);
    lcd.print(tempcLm35);

    lcd.setCursor(15, 2);
    lcd.write(1);
    lcd.setCursor(16, 2);
    lcd.print(F("C")); 


    lcd.setCursor(0, 3);
    lcd.print(F("Pressure:"));
    lcd.setCursor(10, 3);
    lcd.print(baropPessure());
    lcd.setCursor(16, 3);
    lcd.print(F(" KPa"));

    delay(1000);

   
}






//Method to upload data to server
void upload2()
{


  
 /*  
  
//trying a way to send data in a packet
    char reply_text[26];

  
inet.attachGPRS("WAP", "", "");
  
  float presf = baropPessure();
int pres=floor(presf*10);


   sprintf_P(reply_text,PSTR("x=%04d,%04d,%04d,%40d,%04d"),tempc , rhsense, rp4s, count2, pres);
   lcd.setCursor(0,0);
   lcd.print(reply_text);
   delay(5000);
  inet.httpPOST2("weather.shparvez.net", 80, "/a/c1.php", reply_text );
 */
 
 

char reply_text[6];
  
inet.attachGPRS("WAP", "", "");
   sprintf_P(reply_text,PSTR("h=%4d"), rhsense);
inet.httpPOST2("weather.shparvez.net", 80, "/a/h.php", reply_text );


inet.attachGPRS("WAP", "", "");
  sprintf_P(reply_text,PSTR("t=%4d"), tempc);
 inet.httpPOST2("weather.shparvez.net", 80, "/a/t.php", reply_text);
 
 
inet.attachGPRS("WAP", "", "");
float presf = baropPessure();
//int pres=floor(presf);
int pres=floor(presf*10);// use this line for sending the pressure multiplied by 10 so can be show in server to 1 decimal place
  sprintf_P(reply_text,PSTR("b=%4d"), pres);
Serial.println(presf);
inet.httpPOST2("weather.shparvez.net", 80, "/a/b.php", reply_text);


inet.attachGPRS("WAP", "", "");
  sprintf_P(reply_text,PSTR("w=%4d"), rp4s);
inet.httpPOST2("weather.shparvez.net", 80, "/a/w.php", reply_text);

inet.attachGPRS("WAP", "", "");
  sprintf_P(reply_text,PSTR("r=%4d"), count2);
inet.httpPOST2("weather.shparvez.net", 80, "/a/r.php", reply_text);
 
 


    delay(80);

   // gsm.SimpleRead();



}




ISR(INT0_vect)
{
    //Jumps here automatically when INT0 pin detect a falling edge
    count++;
}
ISR(INT1_vect)
{
    //Jumps here automatically when INT0 pin detect a falling edge
    count2++;
}

ISR(TIMER1_COMPA_vect)
{
    //Jumps here every 4 sec exactly!
    rp4s=count;
    speedskmph= rp4s*3.14*.27*.25*3.6*1.1;

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


//Initializing Barometric sensor
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


//End of barometer work





void find_temp( )
{
    tempc=0;

    // Start of calculations FOR loop.
    for(i = 0; i<=7; i++) // gets 8 samples of temperature
    {
      //  samples[i] = analogRead(A0);

        tempc = tempc + analogRead(A0); // do the addition for average temperature
        delay(80); // wait 800ms
    }

    tempc=floor(tempc/8);

    tempcLm35 = ( 460 * tempc ) / 1024.0;

}



void find_humidity( )
{

  
    rhsense=0;

    // Start of calculations FOR loop.
    for(i = 0; i<=7; i++) // gets 8 samples of temperature
    {
        //samples[i] = (analogRead(A1));

        rhsense = rhsense + analogRead(A1); // do the addition for average temperature
        delay(80); // wait 800ms
    }

    rhsense = floor(rhsense/8);

   // k= (rhsense*5/1024);

    // rh = ((30.855*(rhsense/204.6))-11.504);
    //rh=4.8008*k*k*k*k*k-47.772*k*k*k*k+ 184.64*k*k*k-343.78*k*k+334.28* k + -110.6 ;
    
    k= (5.0 *rhsense/1024.0) ;
     rh= 4.8008*k*k*k*k*k - 47.772*k*k*k*k + 184.64 *k*k*k -343.78*k*k +334.28*k +- 110.6;

}


//RTC code starts Donot Change this function unless you understand.....


// BCD to binary number packing/unpacking functions
static uint8_t BCD2bin(uint8_t v)
{
    return v - 6 * (v >> 4);
}
static uint8_t bin2BCD (uint8_t v)
{
    return v + 6 * (v / 10);
}


// Interface functions for the RTC device
uint8_t readDevice(uint8_t addr, uint8_t* buf, uint8_t len)
{
    Wire.beginTransmission(DS1307_ID);
    Wire.write(addr);				// set register address
    if (Wire.endTransmission() != 0)
        return(0);

    Wire.requestFrom(DS1307_ID, (int)len);
    while (!Wire.available()) ;	// wait
    for (uint8_t i=0; i<len; i++) // Read x data from given address upwards...
    {
        buf[i] = Wire.read();       // ... and store it in the buffer
    }

    return(len);
}




void ReadTime( )
// Read the current time from the RTC and unpack it into the object variables
{
    readDevice(RAM_BASE_READ, bufRTC, 7);		// get the data

    // unpack it
    //s = BCD2bin(bufRTC[ADDR_SEC] & ~CTL_CH);	// mask off the 'CH' bit
    m = BCD2bin(bufRTC[ADDR_MIN]);
    if (bufRTC[ADDR_HR] & CTL_12H) 			// 12 hour clock
    {
        h = BCD2bin(bufRTC[ADDR_HR] & 0x1f);
        pm = 0;
    }
    else
    {
        h = BCD2bin(bufRTC[ADDR_HR] & 0x3f);
        pm = (bufRTC[ADDR_HR] & CTL_PM);
    }

}



//End of RTC code




