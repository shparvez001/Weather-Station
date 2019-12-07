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

//FOr wind starts
volatile uint16_t count=0;		//Main revolution counter
volatile uint16_t rpm=0000;	//Revolution per minute
volatile uint16_t rp4s=0000;	//Revolution per second
double speeds;
double speedskmph;

//For wind ends


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

init_anemo(  );
 
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
  sei();
  
 
 //Sending Only Temperature data to a server
 


     char reply_text4[20];
 
  sprintf_P(reply_text4,PSTR("wind=%4d"), rp4s);

 numdata=inet.httpPOST("weather.shparvez.net", 80, "/Add/wind.php", reply_text4 ,msg2, 200);


 
 
 




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


