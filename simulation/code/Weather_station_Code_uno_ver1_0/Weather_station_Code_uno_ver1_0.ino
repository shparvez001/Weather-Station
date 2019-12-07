//Code for ATMEGA328
#include <avr/io.h>
#define F_CPU 16000000UL 

#include <util/delay.h>
#include <LiquidCrystal.h>


volatile uint16_t count=0;		//Main revolution counter

volatile uint16_t rpm=0000;	//Revolution per minute
volatile uint16_t rp4s=0000;	//Revolution per second
double speeds;
float tempc = 0;  // variable for holding Celcius temp (floating for decimal points precision)
int tempPin = 0;  // Declaring the Analog input to be 0 (A0) of Arduino board.

//LiquidCrystal lcd(12, 11, 6, 5, 4, 3);
LiquidCrystal lcd(1, 0, 17, 6, 7, 8);
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
    //Serial.begin(9600); //opens serial port, sets data rate to 9600 bps
    
      lcd.begin(20, 4);
      

  lcd.clear();
             lcd.setCursor(0, 0);
             lcd.print("Temp:  32 C");
                          lcd.setCursor(0, 1);
             lcd.print("Humidity:  67 %");
                          lcd.setCursor(0, 2);
             lcd.print("Rain:  20 mm");
                          lcd.setCursor(0, 3);
             lcd.print("Wind:  5 km/h");
                    
                
                Wait2();


}

	void loop()
	{
  
    
             lcd.setCursor(0, 0);
             lcd.print("Temp:  32 C");
                          lcd.setCursor(0, 1);
             lcd.print("Humidity:  67 %C");
                          lcd.setCursor(0, 2);
             lcd.print("Rain:  20 mm");
                          lcd.setCursor(0, 3);
             lcd.print("Wind:  5 km/h");
                    
                
                Wait2();
              


	}
	


           
 
