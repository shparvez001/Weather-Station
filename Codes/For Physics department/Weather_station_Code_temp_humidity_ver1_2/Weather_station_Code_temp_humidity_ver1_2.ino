//Code for ATMEGA328
// including temperature and humidity and rain
// Code by Shahadat Hussain Parvez. Mail at shparvez001@gmail.com, http://www.shparvez.net
#include <avr/io.h>

#include <LiquidCrystal.h>


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


float tempc = 0;  // variable for holding Celcius temp (floating for decimal points precision)
int tempPin = 3;  // Declaring the Analog input to be 0 (A0) of Arduino board.
float samples[8]; // array to hold 8 samples for Average temp calculation
float maxi = 0,mini = 100; // max/min temperature variables with initial values. LM35 in simple setup only measures Temp above 0.
int i;
float rhsense = 0;
float rh = 0;
float k=0;



LiquidCrystal lcd(10, 9, 5, 6, 7, 8);



void setup ()
{

    lcd.begin(16, 2);


    lcd.setCursor(0, 0);
    lcd.print("Made By EEESUST");


    delay(1000);

}

void loop()
{

  //  lcd.clear();

   
          
    lcd_all();
    lcd_all();
    lcd_all();
    lcd_eee();
    


}







void lcd_all( )
{
      init_temp();
    init_humidity( );

    lcd.setCursor(0, 0);
    lcd.print("NLO Lab                ");



    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.setCursor(2, 1);
    lcd.print(tempc);


    lcd.setCursor(7, 2);
    lcd.print("|");



    lcd.setCursor(8, 1);
    lcd.print("H:");
    lcd.setCursor(10, 1);
    lcd.print(rh);
    //lcd.setCursor(7, 1);
    //lcd.print(" %");

    delay(1500);



}

void lcd_eee( )
{


    init_temp();
    init_humidity( );



    lcd.setCursor(0, 0);
    lcd.print("Made By EEESUST");

    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.setCursor(2, 1);
    lcd.print(tempc);


    lcd.setCursor(7, 2);
    lcd.print("|");



    lcd.setCursor(8, 1);
    lcd.print("H:");
    lcd.setCursor(10, 1);
    lcd.print(rh);


    delay(2000);



}



void init_temp( )
{
    tempc=0;

    //      tempc=( 4 * analogRead(tempPin) * 100.0) / 1024.0;

    for(i = 0; i<=7; i++)
    {
        samples[i] = 20;
    }


    // Start of calculations FOR loop.
    for(i = 0; i<=7; i++) // gets 8 samples of temperature
    {
        samples[i] = (4.6 * analogRead(tempPin) * 100.0) / 1024.0;

        if ((samples[i] - samples[i-1])>35)
        {
            i--;
            lcd.clear();
            lcd.setCursor(0, 1);
            lcd.print("Sensor Problem! ");
            delay(500);
            lcd.clear();
            delay(100);
            continue;
        }

        tempc = tempc + samples[i]; // do the addition for average temperature
        delay(120); // wait 80ms
    }

    tempc = tempc/8.0; // calculated the averare of 8 samples in Celcius



}


void init_humidity( )
{
    rh=0;

    for(i = 0; i<=7; i++)
    {
        samples[i] = 100;
    }

    for(i = 0; i<=7; i++) // gets 8 samples of temperature
    {


        rhsense = (analogRead(A2));
        k=(rhsense*5)/1024;

        // rh = ((30.855*(rhsense/204.6))-11.504);
//          rh=4.8008*k*k*k*k*k-47.772*k*k*k*k+ 184.64*k*k*k-343.78*k*k+334.28* k + -110.6 ;


        samples[i] = 4.8008*k*k*k*k*k-47.772*k*k*k*k+ 184.64*k*k*k-343.78*k*k+334.28* k + -110.6 ;

        if ((samples[i] - samples[i-1])>95)
        {
            i--;
            lcd.clear();
            lcd.setCursor(0, 1);
            lcd.print("Sensor Problem! ");
            delay(500);

            lcd.clear();
            delay(100);
            continue;
        }
        rh = rh + samples[i]; // do the addition for average HUmidity
        delay(120); // wait 800ms
    }

    rh = rh/8.0;






}
