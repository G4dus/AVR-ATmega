#include <max6675.h>
#include <PID_v1.h>

double Setpoint = 150;
//PID_1 Temperature
double Setpoint_1;
double Input_1;
double Output_1;                    
double Kp_1=200, Ki_1=10, Kd_1=58;      
PID PID_1(&Input_1, &Output_1, &Setpoint, Kp_1, Ki_1, Kd_1, DIRECT); 


//thermocople max6675 config
int thermoSO = 12;
int thermoCS =11;
int thermoCLK = 10;
int vccPin = 9;
int gndPin = 8;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoSO);


unsigned long previousMillis = 0;  // test
const long interval = 40;        //test

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

 double Setpoint = 150;
  Input_1 = thermocouple.readCelsius(); // 
  Setpoint_1 = 150;
  PID_1.SetMode(AUTOMATIC);
  PID_1.SetTunings(Kp_1, Ki_1, Kd_1);

 delay(500); //load Max6675 config please wait...


}

void loop()
     {


      unsigned long currentMillis = millis(); //test
      
    if (digitalRead(1) == LOW)
       {
       Setpoint_1 = Setpoint_1 + 1;
       delay(250);
       } 
       
    if (digitalRead(0) == LOW)
       {
       Setpoint_1 = Setpoint_1 -1;
       delay(250);
       }     

  // Input_1 = thermocouple.readCelsius();     //Send temperature form MAX6675 to PID
  PID_1.Compute();                        //Call and calculate PID 
  analogWrite(3,Output_1);                  //output PID controler


  if (currentMillis - previousMillis >= interval) 
     {
      previousMillis = currentMillis;
      Input_1 = thermocouple.readCelsius(); 
     }
 

 //logs
  Serial.println(Input_1);
  //Serial.println(" *C");
  //Serial.println(Output_1);
        Serial.println (Setpoint);                      


}