#include <hcsr04.h>
#include <DHT.h>
#include <PinChangeInt.h>
//#include <RF69Mod.h>

#define ECHO_PIN      3
#define TRIG_PIN      4

#define DHTPin        5  
#define COOLER        6
#define PUMP          7
#define LED           8
#define SW_NOT_TANK   9 
#define SW_HI_LEVEL   14

#define SET_HI_LEVEL  20
#define SET_LOW_LEVEL 40
#define RF_freq           RF69_433MHZ                                                               //Frecuencia de transmision 433Mhz


HCSR04 mySensor;
DHT dht;
int i;
const int nodeIdEmonPi=5;                                                                            // Id del EmonPi
const int nodeId = 13;                                                                               // emonTx RFM12B nodo ID
const int netGroup = 210;                                                                            // emonTx RFM12B  grupo ID
unsigned short distance;
float humAmb,tempAmb;

void setup() {    

  pinMode(SW_NOT_TANK, INPUT);
  digitalWrite(SW_NOT_TANK, HIGH);
  pinMode(SW_HI_LEVEL, INPUT);
  digitalWrite(SW_HI_LEVEL, HIGH);
  pinMode(COOLER, OUTPUT);
  digitalWrite(COOLER, HIGH);
  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP, HIGH);
  pinMode(LED, OUTPUT);
  mySensor.init(TRIG_PIN, ECHO_PIN);
  //sensorLevel.setDelayBetweenAvgMeasurementsInMs(100);
  dht.setup(DHTPin);

  for(int j=0;j<15;j++){
    digitalWrite(LED, !digitalRead(LED));
    delay(200);
  }
  Serial.begin(9600); // Starts the serial communication

}

void loop(){
  i++;
  distance = mySensor.readDisctanceInMm(); //aprox 400ms
  tempAmb=dht.getTemperature();
  tempAmb=isnan(tempAmb)?-127 :tempAmb;
  humAmb=dht.getHumidity();
  humAmb=isnan(humAmb)?-127 :humAmb;
  Serial.print(i);
  Serial.print(") Distance: ");
  Serial.print(distance);
  Serial.print ("  Temperatura: ");
  Serial.print(tempAmb);
  Serial.print("   Humedad: ");
  Serial.println(humAmb);
  delay(1000);
  // Do something with measured distance value
}