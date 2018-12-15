#include <hcsr04.h>
#include <DHT.h>
#include <PinChangeInt.h>
//#include <RF69Mod.h>

#define TRIG_PIN      3
#define ECHO_PIN      4
#define DHTPin        5  
#define COOLER        6
#define PUMP          7
#define LED           8
#define SW_NOT_TANK   9 
#define SW_HI_LEVEL   14

#define RF_freq           RF69_433MHZ                                                               //Frecuencia de transmision 433Mhz
#define LO_LEVEL_MM   50    //max 78mm
#define HI_LEVEL_MM   24   // sw 35 mm
//300ml 52mm
//bombeo 75 segundos


HCSR04 sensorLevel;
DHT dht;
float tankLevel;
short levelmm;
int i,errCode,status;
const int nodeIdEmonPi=5;                                                                            // Id del EmonPi
const int nodeId = 13;                                                                               // emonTx RFM12B nodo ID
const int netGroup = 210;                                                                            // emonTx RFM12B  grupo ID
unsigned short distance;
float humAmb,tempAmb;

void printdata(){
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" Tanque: ");
  Serial.print(tankLevel);
  Serial.print ("  Temperatura: ");
  Serial.print(tempAmb);
  Serial.print("   Humedad: ");
  Serial.print(humAmb);
  Serial.print("  SW: ");
  Serial.println(!digitalRead(SW_HI_LEVEL));

}

short getTankLevel (){
  short distance;
  distance= sensorLevel.readAccurateDisctanceInMm(); //aprox 400ms
  return (100*(distance - LO_LEVEL_MM) ) / (HI_LEVEL_MM - LO_LEVEL_MM); 
}

void setup() {    

  pinMode(SW_NOT_TANK, INPUT);
  digitalWrite(SW_NOT_TANK, HIGH);
  pinMode(SW_HI_LEVEL, INPUT);
  digitalWrite(SW_HI_LEVEL, HIGH);
  pinMode(COOLER, OUTPUT);
  digitalWrite(COOLER, LOW);
  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP, HIGH);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  sensorLevel.init(TRIG_PIN, ECHO_PIN);
  sensorLevel.setDelayBetweenAvgMeasurementsInMs(35);
  dht.setup(DHTPin);

  for(int j=0;j<15;j++){
    digitalWrite(LED, !digitalRead(LED));
    delay(200);
  }
  Serial.begin(9600); // Starts the serial communication

}

void loop(){

  /*if (!digitalRead(SW_NOT_TANK) && digitalRead(SW_HI_LEVEL)){
    digitalWrite(COOLER, LOW);
    status=1;
  } else digitalWrite(COOLER, HIGH);
  */
  if (!digitalRead(PUMP)) digitalWrite(COOLER,HIGH);
  //levelmm=sensorLevel.readAccurateDisctanceInMm();
  levelmm=sensorLevel.readAvgDisctanceInMm(40);
  Serial.print("Distance: ");
  Serial.println(levelmm);
  if(levelmm<40 || !digitalRead(SW_HI_LEVEL)){ //40mm
    digitalWrite(PUMP, LOW);
    digitalWrite(COOLER, HIGH);
  }
  if(levelmm>85){
    digitalWrite(PUMP, HIGH);
  }
  delay(500);

  
/*
  if(distance > LO_LEVEL_MM){ //70
    tankLevel = 0;
    if(distance > LO_LEVEL_MM + 10){
      status=2;
      errCode=20;
    }
  }
  else {
    if(distance < HI_LEVEL_MM){//42
      tankLevel = 100;
      if(distance < HI_LEVEL_MM - 10 || !digitalRead(SW_HI_LEVEL)){
        status=2;
        errCode=30;
      }
    }
    else tankLevel=(100*(float(distance) - LO_LEVEL_MM) ) / (HI_LEVEL_MM - LO_LEVEL_MM);
  }

  
  if (tankLevel==100){
    digitalWrite(PUMP, LOW);
    do{
      distance = sensorLevel.readAccurateDisctanceInMm();
      tankLevel=(100*(float(distance) - LO_LEVEL_MM) ) / (HI_LEVEL_MM - LO_LEVEL_MM);
    } while()
  }
  delay(1000);
  // Do something with measured distance value
*/}