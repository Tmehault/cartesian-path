// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// as posted on Arduino Forum at http://forum.arduino.cc/index.php?topic=208905.0
#include <PID_v1.h>//library for PID
//PINS
byte directionPin = 28;
byte stepPin = 26;
byte enablePin = 24;
#define thermistor A13
#define heat 10

//PARAMETERS
float temp=0;
int goal= 200;//goal in celsius
int freq_heat = 10;
float freq_extrusion = 250;//in microseconds
float mm_per_step = 8.16*pow(10,-4);
float per_us = 1*pow(10,6)/mm_per_step;

//VAR
unsigned long prev_extr_micros=0;
unsigned long previousMillis=0;
unsigned long currentMillis=0;
unsigned long t_print_temp=0;
bool f_d=0;
int i=0;
float value;
bool i_extrusion=false;
bool i_heat=false; 

//PID
double Setpoint, Input, Output;
PID tempPID(&Input, &Output, &Setpoint, 22.2, 1.08,114, DIRECT);


float rtodeg(float temp)
{
  float deg= 7.62*pow(10,-10)*pow(temp,4);//polynome 4 to fit celsius on resistance value
  deg= deg - 2.11*pow(10,-6)*pow(temp,3);
  deg= deg + 2.02*pow(10,-3)*pow(temp,2);
  deg= deg - 9.38*pow(10,-1)*temp + 2.92*pow(10,2);
  return deg;
}
float degtor(float deg)
{
  float r= -1.45*pow(10,-6)*pow(deg,4);//polynome 4 to fit celsius on resistance value
  r= r + 9.23*pow(10,-4)*pow(deg,3);
  r= r - 1.85*pow(10,-1)*pow(deg,2);
  r= r + 7.94*pow(10,0)*deg + 8.62*pow(10,2);
  return r;
}


void setup() 
{ 

  Serial.begin(115200);
  
  delay(1000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(heat,OUTPUT);
  
  digitalWrite(enablePin, LOW);
  digitalWrite(directionPin, HIGH);
  digitalWrite(heat,LOW);

  Setpoint=degtor(goal-8);
  tempPID.SetOutputLimits(0,1);
  tempPID.SetMode(AUTOMATIC);
  
}


void loop() 
{ 
  digitalWrite(enablePin, LOW);
  temp=analogRead(thermistor);
  if(Serial.read()-'0'==72)
  {value=Serial.parseFloat(SKIP_WHITESPACE);//parse function is blocking so only used when receiving data 'xfloat'
  if(value!=0.0)
  {
    if(value==250)//temperature request
    {
      Serial.println(rtodeg(temp));
    }
    if(value==251)//start/stop extrusion
    {
      i_extrusion=not(i_extrusion);
    }
    if(value==252)//start/stop heating
    {
      i_heat=not(i_heat);
    }
    if(value>=0 and value<100)//speed request
    {
       freq_extrusion=1/(value*pow(10,-2)/(8.16));
       Serial.println(freq_extrusion);
    }
  }
  if(temp>=220) //overheating security
  {
    i_heat=false;
    i_extrusion=false;
  }
  }
  currentMillis = millis();//get time
  if (currentMillis - previousMillis >= freq_heat) //time to heat ?
  {
    Input=temp;
    tempPID.Compute();
    if(currentMillis-t_print_temp >= 1000)//publish temperature rate : 1 sec
    {
      //Serial.println(rtodeg(temp));
      t_print_temp=currentMillis;
    }
    if(Output==0 and i_heat)
    {
      digitalWrite(heat,HIGH);
    }
    else
    {
      digitalWrite(heat,LOW);
    }
    previousMillis = millis();//update time at the end of actions
  }
  digitalWrite(directionPin, HIGH);
  f_d=1;
  if(micros() - prev_extr_micros >= freq_extrusion and i_extrusion)//limit frequency to fix extrusion speed and 1000 steps
  {
    digitalWrite(stepPin,HIGH); // Output high
    delayMicroseconds(1); // minimal Wait (resolution is 4 microseconds)
    digitalWrite(stepPin,LOW); // Output low
    prev_extr_micros=micros();
  }
}
