#include <FastPID.h>
#include <max6675.h>


#define PIN_OUTPUT    3

float Kp=20, Ki=1, Kd=2, Hz=600;

int output_bits = 8;
bool output_signed = false;
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

double Input_1;

//thermocople max6675 config
int thermoSO = 12;
int thermoCS =11;
int thermoCLK = 10;
int vccPin = 9;     //option to easier connection board remove if you need pin
int gndPin = 8;     //option to easier connection board remove if 
MAX6675 thermocouple(thermoCLK, thermoCS, thermoSO);


//motor setup

int analogInput = A0;
int Motor_Speed = 0;
int Motor_PWM = 4;


//interval
unsigned long previousMillis = 0; 
const long interval = 280;   

unsigned long previousMillis_2 = 0; 
const long interval_2 = 250;                                                   //Set interval time [ms]
  
void setup()
{
  Serial.begin(9600);
  pinMode(1, INPUT_PULLUP); //#Setpoint up
  pinMode(0, INPUT_PULLUP); //#Setpoint down
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);
  
  pinMode(Motor_PWM, OUTPUT); //Motor speed PWM
  
  pinMode(thermoCS, OUTPUT);
  pinMode(thermoSO, INPUT);
  pinMode(thermoCLK, OUTPUT);



  Input_1 = thermocouple.readCelsius(); 


 delay(500); //load Max6675 config please wait...

}

void loop()
     {
      int Setpoint =150;
//delay(240);
      unsigned long currentMillis = millis();
      
    if (1==0)//(digitalRead(1) == LOW)
       {
       Setpoint = Setpoint + 1;
       delay(250);
       } 
       
    if (1==0)  // (digitalRead(0) == LOW)
       {
       Setpoint = Setpoint -1;
       delay(250);
       }     


  int setpoint = 150;
  int feedback = Input_1;
  uint32_t before, after;
  before = micros();
  uint8_t output = myPID.step(setpoint, feedback);
  after = micros();
analogWrite(PIN_OUTPUT, output);






  if (currentMillis - previousMillis >= interval) //cyclic interrupts
     {
      previousMillis = currentMillis;
      Input_1 = thermocouple.readCelsius(); //You have to call readCelsius in intervals otherwise crash MAX6675
     }



      if (currentMillis - previousMillis_2 >= interval_2) //cyclic interrupts
     {
      previousMillis_2 = currentMillis;
     
//  Serial.print(Input_1);
//  Serial.print("    ");
//  Serial.println(Setpoint);


  Serial.print("runtime: "); 
  Serial.print(after - before);
  Serial.print(" sp: "); 
  Serial.print(setpoint); 
  Serial.print(" fb: "); 
  Serial.print(feedback);
  Serial.print(" out: ");
  Serial.println(output);
  delay(100);
     }
 
 //logs




/*
*
* PWM Motor control below
*
*/

 int Motor_Speed = analogRead(analogInput);
 analogWrite(Motor_PWM, Motor_Speed /4);

}


/*     
*******************     
*****Used Pins*****
*******************
*
* Analog: 
*       A0 [x]
*       A1 []
*       A2 []
*       A3 []
*       A4 []     
*       A5 []
* ----------------
* Digital:       
*       0  [x]  Setpoint down
*       1  [x]  Setpoint up
*       2  []
*       3  [x]  PID PWM output 0-255
*       4  [x]  Motor PWM output 0-255
*       5  []
*       6  []
*       7  []
*       8  [x]  MAX6675 GND [remove if you need pin] 
*       9  [x]  MAX6675 VCC [remove if you need pin]
*       10 [x]  MAX6675 CLK
*       11 [x]  MAX6675 CS
*       12 [x]  MAX6675 SO
*       13 []
*-------------------
*/
