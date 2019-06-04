// Arduino programm for Surmoul3D

#include <PID_v1.h>//library for PID
//PINS
byte directionPin = 28;
byte stepPin = 26;
byte enablePin = 24;
#define thermistor A13
#define heat 10
const byte interruptpin=2;

//PID
double Setpoint, Input, Output;
PID tempPID(&Input, &Output, &Setpoint, 22.2, 1.08,114, DIRECT);

//PARAMETERS
uint16_t temp=0;
uint8_t goal= 200;//goal in celsius
uint16_t freq_heat = 200;//milliseconds
volatile uint16_t freq_extrusion = 20000;//in microseconds

//VAR
unsigned long prev_extr_micros=0;
unsigned long previousMillis=0;
unsigned long currentMillis=0;
unsigned long currentMicros=0;
unsigned long t_print_temp=0;
unsigned long start_time=0;
uint16_t val_extrusion[1000];
bool i_extrusion=false;
bool i_heat=false;
volatile uint16_t indice=0; 



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

  Serial.begin(57600);
  
  delay(1000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(heat,OUTPUT);
  
  digitalWrite(enablePin, LOW);
  digitalWrite(directionPin, HIGH);
  digitalWrite(heat,LOW);
  
//-----------------------------------
//  | PID init  |
//-----------------------------------

  Setpoint=degtor(goal-8);
  tempPID.SetOutputLimits(0,1);
  tempPID.SetMode(AUTOMATIC);

//-----------------------------------
//  | Interruption init  |
//-----------------------------------

  pinMode(21, INPUT);
  attachInterrupt(2,ISR_extru,CHANGE);   

//-----------------------------------
//  | Get data  |
//-----------------------------------

  Serial.println("Ready to get orders");
  while(Serial.read()-'0' != 72)
  {
    //empty
  }
  //here we should have received first x
  int i_max=Serial.parseInt(); //number of data to receive
  for(int i=0;i<i_max;i++) //we are blocking the loop to get our values 'x250x51x65x61x23'
  {
      while(Serial.read()-'0' != 72){}//wait data and skip the x
      val_extrusion[i]=Serial.parseInt(); ////extrusion value in mm/s
      //Serial.println(val_extrusion[i]);
  }
  Serial.println("Acquisition terminee");
  i_heat=true;
}

// x4x4x0001x8x7593x10x22063x20x105492    x55x55555


void loop() 
{ 
  digitalWrite(enablePin, LOW);
  currentMillis = millis();//get time
  currentMicros = micros();
  
  // ----------- CHAUFFE ---------------
  if( (currentMillis - previousMillis >= freq_heat) and (i_heat) ) //time to heat ?
  {
    temp=analogRead(thermistor);
    Input=temp;
    tempPID.Compute();
    if(currentMillis-t_print_temp >= 1000)//publish temperature rate : 1 sec
    {
      t_print_temp=currentMillis;
      Serial.println(rtodeg(temp));
    }
    if(Output==0)
    {
      digitalWrite(heat,HIGH);
    }
    else
    {
      digitalWrite(heat,LOW);
    }
    previousMillis = millis();//update time at the end of actions
  }
  
  // -------- EXTRUSION -------------
  if((currentMicros - prev_extr_micros >= freq_extrusion))// and (i_extrusion))
  {
    digitalWrite(directionPin, HIGH);
    digitalWrite(stepPin,HIGH); // Output high
    delayMicroseconds(1); // minimal Wait (resolution is 4 microseconds)
    digitalWrite(stepPin,LOW); // Output low
    prev_extr_micros=micros();
  }
}

void ISR_extru()
{
    freq_extrusion=round( 1/((val_extrusion[indice]+1)*pow(10,-2)/(8.16)) );
    indice=indice+1;
}
