// Arduino programm for Surmoul3D

#include <PID_v1.h>//library for PID
//PINS
byte directionPin = 28;
byte stepPin = 26;
byte enablePin = 24;
#define thermistor A13
#define heat 10
const byte interruptpin = 2;

//PID
double Setpoint, Input, Output;
PID tempPID(&Input, &Output, &Setpoint, 22.2, 1.08, 114, DIRECT);

//PARAMETERS
uint16_t temp = 100;
uint8_t goal = 200; //goal in celsius
uint16_t freq_heat = 200;//milliseconds
volatile uint16_t freq_extrusion = 20000;//in microseconds
bool direction_extrudeur = HIGH;
bool outils = false;

//VAR
unsigned long prev_extr_micros = 0;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long currentMicros = 0;
unsigned long t_print_temp = 0;
unsigned long start_time = 0;
uint16_t val_extrusion[3000];
bool i_extrusion = false;
bool i_heat = false;
volatile uint16_t indice = 0;



float rtodeg(float temp)
{
  float deg = 7.62 * pow(10, -10) * pow(temp, 4); //polynome 4 to fit celsius on resistance value
  deg = deg - 2.11 * pow(10, -6) * pow(temp, 3);
  deg = deg + 2.02 * pow(10, -3) * pow(temp, 2);
  deg = deg - 9.38 * pow(10, -1) * temp + 2.92 * pow(10, 2);
  return deg;
}
float degtor(float deg)
{
  float r = -1.45 * pow(10, -6) * pow(deg, 4); //polynome 4 to fit celsius on resistance value
  r = r + 9.23 * pow(10, -4) * pow(deg, 3);
  r = r - 1.85 * pow(10, -1) * pow(deg, 2);
  r = r + 7.94 * pow(10, 0) * deg + 8.62 * pow(10, 2);
  return r;
}


void setup()
{

  Serial.begin(57600);

  delay(1000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(heat, OUTPUT);

  digitalWrite(enablePin, LOW);
  digitalWrite(directionPin, HIGH);
  digitalWrite(heat, LOW);

  //-----------------------------------
  //  | PID init  |
  //-----------------------------------

  Setpoint = degtor(goal - 8);
  tempPID.SetOutputLimits(0, 1);
  tempPID.SetMode(AUTOMATIC);

  //-----------------------------------
  //  | Interruption init  |
  //-----------------------------------

  pinMode(21, INPUT);
  attachInterrupt(2, ISR_extru, CHANGE);
  pinMode(20, INPUT);
  attachInterrupt(digitalPinToInterrupt(20), Interrupt_outils, RISING);

  //-----------------------------------
  //  | Get data  |
  //-----------------------------------

  Serial.println("arduino prete");

  
}

// x4x4x0001x8x7593x10x22063x20x105492    x55x55555


void loop()
{
  digitalWrite(enablePin, LOW);
  currentMillis = millis();//get time
  currentMicros = micros();

  // ----------- CHAUFFE ---------------
  if(i_heat)
  {
    if ( (currentMillis - previousMillis >= freq_heat) ) //time to heat ?
    {
      temp = analogRead(thermistor);
      Input = temp;
      tempPID.Compute();
      if (Output == 0 and temp > 45) //resistance=45 is equivalent to 250deg lower values is too hot (security)
      {
        digitalWrite(heat, HIGH);
      }
      else
      {
         digitalWrite(heat, LOW);
      }
      previousMillis = millis();//update time at the end of actions
    }
  }
  else
  {
     digitalWrite(heat, LOW);
  }
  

  // -------- EXTRUSION -------------
  if ((currentMicros - prev_extr_micros >= freq_extrusion) and (i_extrusion))
  {
    digitalWrite(directionPin, direction_extrudeur);
    digitalWrite(stepPin, HIGH); // Output high
    delayMicroseconds(1); // minimal Wait (resolution is 4 microseconds)
    digitalWrite(stepPin, LOW); // Output low
    prev_extr_micros = micros();
  }
  if (outils)
  {
    Outils_commandes();
  }
}

//------------------INTERRUPTIONS-------------

void ISR_extru()
{
  freq_extrusion = round( 1 / ((val_extrusion[indice] + 1) * pow(10, -2) / (8.16)) );
  i_extrusion=true;
  indice = indice + 1;
}
void Interrupt_outils()
{
  outils = true;
}

//------------------COMMANDES OUTILS----------------

void Outils_commandes()
{
  /*Actions possibles:
    t1-chauffe
    t2-maj objectif temperature
    t3-affiche temperature
    t4-chargement materiau
    t5-déchargement materiau
    t6-remplir tableau d'extrusion
    t7-extrusion vitesse donnee
    t8-stop extrusion
    t9-stop chauffe
  */
  //Serial.println("Mode outils active");
  /*while (Serial.read() - '0' != 68) //attente recevoir t
  {
    //empty
  }*/
  Serial.read();
  int commande = Serial.parseInt();
  //Serial.println(commande);
  
  if(commande==0)
  {
    Serial.println("test");
  }
  if(commande==1)  //chauffe
  {
    Serial.println("Chauffe");
    i_heat = true;
  }
  if(commande==2)   //maj objectif temperature
  {
    Serial.println("Objectif temperature");
    Serial.read();  //empty separator
    int a = Serial.parseInt();
    a = a - 8;
    Setpoint = degtor(float(a));
  }
      
  if(commande==3)   //Affiche temperature
  {
    Serial.println("Affiche temperature");
    temp = analogRead(thermistor);
    Serial.println(rtodeg(temp));
  }
      
  if(commande==4)   //chargement materiau
  {
    Serial.println("Chargement materiau");
    if (i_heat)
    {
      freq_extrusion = 100;
      i_extrusion = true;
      direction_extrudeur = HIGH;
    }
  }
      
  if(commande==5)   //dechargement materiau
  {
    Serial.println("Dechargement materiau");
    if (i_heat)
    {
      freq_extrusion = 100;
      i_extrusion = true;
      direction_extrudeur = LOW;
    }
  }
      
  if(commande==6)  //remplir tableau des extrusions
  {
    Serial.println("Tableau extrusions");
    while (Serial.read() - '0' != 72)
    {
      //empty
    }
    //here we should have received first x
    int i_max = Serial.parseInt(); //number of data to receive
    for (int i = 0; i < i_max; i++) //we are blocking the loop to get our values 'x250x51x65x61x23'
    {
      while (Serial.read() - '0' != 72) {} //wait data and skip the x
      val_extrusion[i] = Serial.parseInt(); ////extrusion value in mm/s
      //Serial.println(val_extrusion[i]);
    }
    Serial.println("Acquisition terminee");
    indice=0;
  }
  if(commande==7) //extrusion a vitesse donnee
  {
    Serial.println("Extrusion");
    Serial.read();  //empty separator
    int a = Serial.parseInt();
    if (i_heat)
    {
      freq_extrusion = round( 1 / ((a + 1) * pow(10, -2) / (8.16)) );
      i_extrusion = true;
      direction_extrudeur = HIGH;
    }
  }
  if(commande==8) //arret extrusion
  {
    Serial.println("Arret extrusion");
    i_extrusion=false;
  }
  if(commande==9) //arret chauffe
  {
    Serial.println("Arret chauffe");
    i_heat=false;
  }
  
  //Serial.println("Termine");
  outils = false;
}
