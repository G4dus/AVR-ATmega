#include <max6675.h>
#include <PID_v1.h>

//PID_1
double Setpoint = 150; //define Setpoint 1 of 2
double Input_1;
double Output_1;                          
double Kp_1=2, Ki_1=1, Kd_1=0.2;                                         //Set Kp, Ki, Kd            
PID PID_1(&Input_1, &Output_1, &Setpoint, Kp_1, Ki_1, Kd_1, DIRECT); 


//thermocople max6675 config
int thermoSO = 12;
int thermoCS =11;
int thermoCLK = 10;
int vccPin = 9;     //option to easier connection board remove if you need pin
int gndPin = 8;     //option to easier connection board remove if 
MAX6675 thermocouple(thermoCLK, thermoCS, thermoSO);


unsigned long previousMillis = 0; 
const long interval = 40;                                                   //Set interval time [ms]
  
void setup()
{
  Serial.begin(115200);
  pinMode(1, INPUT_PULLUP); //#Setpoint up
  pinMode(0, INPUT_PULLUP); //#Setpoint down
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);

  
  pinMode(thermoCS, OUTPUT);
  pinMode(thermoSO, INPUT);
  pinMode(thermoCLK, OUTPUT);

 double Setpoint = 150; //defile setpoint 2 of 2
  Input_1 = thermocouple.readCelsius(); 
  PID_1.SetMode(AUTOMATIC);
  PID_1.SetTunings(Kp_1, Ki_1, Kd_1);

 delay(500); //load Max6675 config please wait...

}

void loop()
     {

      unsigned long currentMillis = millis();
      
    if (digitalRead(1) == LOW)
       {
       Setpoint = Setpoint + 1;
       delay(250);
       } 
       
    if (digitalRead(0) == LOW)
       {
       Setpoint = Setpoint -1;
       delay(250);
       }     

  PID_1.Compute();                        //Call and calculate PID 
  analogWrite(3,Output_1);                  //output PID controler

  if (currentMillis - previousMillis >= interval) //cyclic interrupts                                         ??????????????????????????????????????????????????????
     {
      previousMillis = currentMillis;
      Input_1 = thermocouple.readCelsius(); //You have to call readCelsius in intervals diffrent crash MAX6675 ???????????????????????????????????????????????????
     }
 
 //logs
  Serial.println(Input_1);
  Serial.println (Setpoint);                      


/*
*
* PWM Motor control below
*
*/



}


/*     
*******************     
*****Used Pins*****
*******************
*
* Analog: 
*       A0 []
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
*       4  []
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
