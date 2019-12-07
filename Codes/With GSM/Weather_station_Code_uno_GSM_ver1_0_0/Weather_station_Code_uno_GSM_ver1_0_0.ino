//Code for ATMEGA328
// including temperature and humidity and rain
// Code by Shahadat Hussain Parvez. Mail at shparvez001@gmail.com, http://www.shparvez.net
#include <avr/io.h>
#define F_CPU 16000000UL 
#include <avr/interrupt.h>
#include <util/delay.h>
#include <LiquidCrystal.h>

//GSM includes. Is added as in example 
#include "SIM900.h"
#include <SoftwareSerial.h>

#include "sms.h"
SMSGSM sms;



//int numdata;
boolean started=false;
char smsbuffer[160];
char n[20];

byte deg[8] = {
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

volatile uint16_t rpm=0000;	//Revolution per minute
volatile uint16_t rp4s=0000;	//Revolution per second
double speeds;
double speedskmph;
float tempc = 0;  // variable for holding Celcius temp (floating for decimal points precision)
int tempPin = 0;  // Declaring the Analog input to be 0 (A0) of Arduino board.
float rhsense = 0;
float rh = 0;
float k=0;


LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

// for board made for control project use LiquidCrystal lcd(8, 7, 10, 11, 12, 13);

void Wait()
{
	uint8_t i;
	for(i=0;i<2;i++)
	{
		_delay_loop_2(0);
	}
}

void Wait2()
{
	uint8_t i;
	for(i=0;i<20;i++)
	{
		_delay_loop_2(0);
	}
}
void setup ()
{
  
  
  
    lcd.createChar(1, deg);
    Serial.begin(9600); //opens serial port, sets data rate to 9600 bps
    
      lcd.begin(20, 4);
      
          // init_gsm();



 
        lcd.setCursor(0, 0);
        lcd.print("Portable weather Station");
          for (int positionCounter = 0; positionCounter < 20; positionCounter++) {
    // scroll one position left:
    lcd.scrollDisplayLeft(); 
    // wait a bit:
    delay(450);
  }
        lcd.setCursor(0, 1);
        lcd.print("Code By SHP");
        
                  init_anemo(  );   
                          lcd.setCursor(0, 0);
        lcd.print("Anemo initialized");
	Wait2();
	Wait2();
delay(2000);



}

	void loop()
	{         


  
               
  
                 sei();
	}
	


ISR(INT0_vect)
{
	
	count++;
}
ISR(INT1_vect)
{
		count2++;
       
}

ISR(TIMER1_COMPA_vect)
{
	//Jumps here every 4 sec exactly!
	rp4s=count;
	rpm=rp4s*60*.25;
speeds= rp4s*3.14*.27*.25;
speedskmph= rp4s*3.14*.27*.25*3.6;
 // Serial print for debugging
             Serial.print("Count ");
             Serial.println(count, DEC);
             Serial.print("Speed ");
             Serial.println(speeds, DEC);      
 
                init_temp();
               
              init_humidity( ); 
       
       //lcd_anemo();
         lcd_all( );
       
      sms123();
 
     count=0;
     
     delay(1000);
} 



void init_anemo(  ) {

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

void init_gsm() {
 
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
    sms.SendSMS("+8801723807161", "GSM Initialized");
     Serial.println("\nSMS sent OK");
  } 
  
}





void init_temp( ) {

           tempc=( 4 * analogRead(tempPin) * 100.0) / 1024.0;          
     }


void init_humidity( ) {

           rhsense = (analogRead(A1));
           k=(rhsense*5)/1024;   
      
          // rh = ((30.855*(rhsense/204.6))-11.504);
          rh=4.8008*k*k*k*k*k-47.772*k*k*k*k+ 184.64*k*k*k-343.78*k*k+334.28* k + -110.6 ;     
 }


void lcd_anemo(  ) {

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

void lcd_all( ) {

             lcd.clear();
             lcd.setCursor(0, 0);
             lcd.print("R_Count ");
             lcd.setCursor(0, 1);
             lcd.print("SPEED");
       
                lcd.setCursor(9, 0);
                lcd.print(count2);
                lcd.setCursor(7, 1);
                lcd.print(speedskmph);
                lcd.setCursor(13, 1);
                lcd.print("Km/h");  
  
             lcd.setCursor(0, 2);
             lcd.print("TEMP"); 
             
             
 

                lcd.setCursor(7, 2);
                lcd.print(tempc);
                
                lcd.setCursor(13, 2);
                lcd.write(1);
                lcd.setCursor(14, 2);
                lcd.print("C");              
                
             lcd.setCursor(0, 3);
             lcd.print("HUMIDITY");                 
                
              

                lcd.setCursor(9, 3);
                lcd.print(rh);
                lcd.setCursor(15, 3);
                lcd.print(" %");              
                
                              
              //  Wait2();
                
                
            
            
 }
 
 
 void sms123 (){

 char reply_text[160];
				
sprintf_P(reply_text,PSTR("R_Count:  %02d , Speed: %3.2f kmph , temp: %3.2f C, Humidity: %3.2f /%"),count2, speedskmph, tempc, rh);
 
  sms.SendSMS("+8801723807161", reply_text);
  //sms.SendSMS("+8801912237361", reply_text);
  Serial.println("\nSMS sent OK");
    //delay(1000000);
                   
 }
 
