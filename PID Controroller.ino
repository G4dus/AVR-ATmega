//
//Download library bellow
#include <max6675.h>
#include <PID_v1.h>

//PID_1
double Setpoint = 220;              //define Setpoint 1 of 2 216..
double Input_1;
double Output_1;                          
double Kp_1=2.8, Ki_1=0.01, Kd_1=150;                                         //Set Kp, Ki, Kd            
PID PID_1(&Input_1, &Output_1, &Setpoint, Kp_1, Ki_1, Kd_1, DIRECT); 


//thermocople max6675 config
int thermoSO = 12;
int thermoCS =11;
int thermoCLK = 10;
int vccPin = 9;     //option to easier connection board remove if you need pin
int gndPin = 8;     //option to easier connection board remove if 
MAX6675 thermocouple(thermoCLK, thermoCS, thermoSO);


//motor setup
int analogInput = A0;
int Motor_Speed =0;
int Motor_PWM = 5;


//puller motor
int StepPin = 6;
int analogInputStepSpeed = A1;
int StepSpeed = 300;


//interval
unsigned long previousMillis = 0; 
const long interval_1 = 250;  
unsigned long previousMillis2 = 0; 
const long interval_2 = 9;    


  
void setup()
{
  Serial.begin(115200);

TCCR0B = TCCR0B & B11111000 | B00000101; // for PWM frequency of 61.04 Hz

  pinMode(Motor_PWM, OUTPUT); //Motor speed PWM

  pinMode(StepPin, OUTPUT); //Puller step stepper motor
  
  pinMode(thermoCS, OUTPUT);
  pinMode(thermoSO, INPUT);
  pinMode(thermoCLK, OUTPUT);
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);

// double Setpoint = 216;                                                //defile setpoint 2 of 2
  Input_1 = thermocouple.readCelsius(); 
  if (Input_1 == 0)
  {Input_1 = 999;}
  PID_1.SetMode(AUTOMATIC);
  PID_1.SetTunings(Kp_1, Ki_1, Kd_1);

 delay(500); //load Max6675 config please wait...

}

void loop()
     {
      
      unsigned long currentMillis = millis();

      PID_1.Compute();                        //Call and calculate PID 
      analogWrite(3,Output_1);                  //output PID controler

  
      if (currentMillis - previousMillis >= interval_1) //cyclic interrupts_1
         {
           previousMillis = currentMillis;
           Input_1 = thermocouple.readCelsius(); //You have to call readCelsius in intervals otherwise crash MAX6675
           
           if (Input_1 == 0)
              {
                Input_1 = 999;
              }
         }


      if (currentMillis - previousMillis2 >= interval_2) //cyclic interrupts_2
         {
           previousMillis2 = currentMillis;
          digitalWrite(StepPin, HIGH);
            Serial.println("High");
         }
         
         digitalWrite(StepPin, LOW);
                            


  
  Serial.println(Input_1);



 //PWM Motor control below
 Motor_Speed = analogRead(analogInput);
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
*       0  []  
*       1  []
*       2  []
*       3  [x]  PID PWM output 0-255
*       4  []  
*       5  [x]  Motor PWM output 0-255
*       6  [x]  Puller motor step
*       7  []
*       8  [x]  MAX6675 GND [remove it if you need pin] 
*       9  [x]  MAX6675 VCC [remove it if you need pin]
*       10 [x]  MAX6675 CLK
*       11 [x]  MAX6675 CS
*       12 [x]  MAX6675 SO
*       13 []
*-------------------
*/
