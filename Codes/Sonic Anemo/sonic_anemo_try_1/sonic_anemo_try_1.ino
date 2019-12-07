/* 
 Testing a HC-SR04 Ultrasonic Ranging Module.
 25th september 2014
 by Shahadat Parvez
 http://www.shparvez.net
 Needs:  An arduino and a HC-SR04
 connection:
 Vcc  --> pin 4 of arduino
 Trig --> pin 5 of arduino
 Echo --> pin 6 of arduino
 Gnd  --> pin 7 of arduino 
 */
#define vcc 4    // Connecting Vcc pin of HC-SR04 to digital pin 4 of arduino
#define trig 5    // Connecting trig pin of HC-SR04 to digital pin 5 of arduino
#define echo 6    // Connecting echo pin of HC-SR04 to digital pin 6 of arduino
#define gnd 7
// Connecting Ground pin of HC-SR04 to digital pin 7 of arduino



void setup(){
  //Make Vcc and Ground
  pinMode(vcc, OUTPUT);
  digitalWrite(vcc, HIGH);
  pinMode(gnd, OUTPUT);
  digitalWrite(gnd, LOW);
  
  // Define Trigger and echo
  pinMode(trig,OUTPUT);       
  pinMode(echo,INPUT);        
  // Serial Begin
  
  Serial.begin(9600);      // init serial 9600
  Serial.println("-------------------Ultra_Demo_Start---------------------------------");
}

void loop(){  
  long microseconds = trig_init();
  Serial.print("ret=");      //
  Serial.println(microseconds);
  long distacne_cm = distance_cm(microseconds);
  Serial.print("Distacne_CM = ");
  Serial.println(distacne_cm);
    long distacne_inch = distance_inch(microseconds);
  Serial.print("Distacne_inch = ");
  Serial.println(distacne_inch);
  delay(100);
}



long trig_init()
{                     
  digitalWrite(trig, LOW);                    
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);                 // pull the Trig pin to high level for more than 10us impulse 
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long microseconds = pulseIn(echo,HIGH);   // waits for the pin to go HIGH, and returns the length of the pulse in microseconds
  return microseconds;                    // return microseconds
}

long distance_cm (long microseconds)
{
  return microseconds / 29 / 2;
}

long distance_inch (long microseconds)
{
  return microseconds / 74 / 2;
}
