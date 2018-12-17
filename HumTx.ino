#include <hcsr04.h>
#include <DHT.h>
#include <RF69Mod.h>

#define TRIG_PIN      3
#define ECHO_PIN      4
#define DHTPin        5  
#define COOLER        6
#define PUMP          7
#define LED           8
#define SW_NOT_TANK   9 
#define SW_HI_LEVEL   14

#define RF_freq       RF69_433MHZ                                                               //Frecuencia de transmision 433Mhz
#define LO_LEVEL_MM   70                                                                        //Distancia en mm bajo nivel
#define HI_LEVEL_MM   40                                                                        //Distancia en mm alto nivel (minima distancia que puede leer el sensor 33mm)
#define alfa          0.2                                                                       //Factor filtro EMA

HCSR04 sensorLevel;
DHT dht;

unsigned long currentTime,cloopTime1,cloopTime2;
int levelperc,status,errCode;
const int nodeIdEmonPi=5;                                                                            // Id del EmonPi
const int nodeId = 13;                                                                               // emonTx RFM12B nodo ID
const int netGroup = 210;                                                                            // emonTx RFM12B  grupo ID
float levelmm,humAmb,tempAmb;
typedef struct { int dat0,dat1,dat2,dat3,dat4;} PayloadTX;       
PayloadTX humtx; 

void getTankLevel (){ //Función que obtiene el nivel del tanque en porcentaje
  levelmm=alfa*float(sensorLevel.readAvgDisctanceInMm(50))+(levelmm*(1-alfa)); //Se filtra el promedio de 50 lecturas del sensor de nivel
  if(levelmm > LO_LEVEL_MM){
      levelperc=0;
      if(levelmm > LO_LEVEL_MM + 40){
        status=2;
        errCode=20;
      }
  }
  else {
    if (levelmm < HI_LEVEL_MM){
      levelperc=100;
      if(levelmm < HI_LEVEL_MM - 5){
        status=2;
        errCode=30;
      }
    }
    else levelperc=(0,0.0633*pow(levelmm,2)-10.096*levelmm+402.22); // Se lineariza la señal del nivel del tanque
  }
}

void sendData (){
   humtx.dat0=tempAmb*10;
   humtx.dat1=humAmb*10;
   humtx.dat2=levelperc;
   humtx.dat3=status;
   humtx.dat4=errCode;
   rf69_sendNow (nodeId, &humtx, sizeof humtx);                //Envio de datos          
}

void printdata (){
  Serial.print("Distance: ");
  Serial.print(levelmm);
  Serial.print("mm");
  Serial.print(" -- ");
  Serial.print(levelperc);
  Serial.print("%");
  Serial.print(" Temp: ");
  Serial.print(tempAmb);
  Serial.print(" Hum: ");
  Serial.print(humAmb);
  Serial.print(" status: ");
  Serial.print(status);
  Serial.print(" errcode: ");
  Serial.print(errCode);
  Serial.print(" TK: ");
  Serial.print(!digitalRead(SW_NOT_TANK));
  Serial.print(" SW-HI: ");
  Serial.println(!digitalRead(SW_HI_LEVEL));
}

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
  digitalWrite(LED, HIGH);
  sensorLevel.init(TRIG_PIN, ECHO_PIN);
  sensorLevel.setDelayBetweenAvgMeasurementsInMs(9);
  levelmm=sensorLevel.readAccurateDisctanceInMm();
  
  //Serial.begin(9600); // Starts the serial communication

  for(int j=0;j<20;j++){
    digitalWrite(LED, !digitalRead(LED));
    delay(250);
  }
  dht.setup(DHTPin);
  rf69_initialize(nodeId, RF_freq, netGroup);
  cloopTime1,cloopTime2 = millis();
}

void loop(){

  currentTime = millis();

  if (!digitalRead(SW_NOT_TANK) && digitalRead(SW_HI_LEVEL)){   // Encendido del enfriador 
    digitalWrite(COOLER, LOW);
    status=1;
    errCode=0;
  } else{
    digitalWrite(COOLER, HIGH);                                 //Apagado del enfriador
    status=2;
    if (digitalRead(SW_NOT_TANK))   errCode=10;
    if (!digitalRead(SW_HI_LEVEL))  errCode=30;
  }

  getTankLevel ();

  if(levelmm<40  && !digitalRead(SW_NOT_TANK)){                 //Encendido bomba cuando el tanque esta lleno
    digitalWrite(PUMP, LOW);
  }
  if(levelmm>85){                                               //Apagado bomba cuando el tanque esta vacio
    delay (2000);
    digitalWrite(PUMP, HIGH);
  }

  if(currentTime-cloopTime1 >= 1000) {
    cloopTime1 = currentTime;
    sendData();                                                 //Transmisión de datos por RF cada 1 segundo
    //printdata();
  }
  
  if(currentTime-cloopTime2 >= 5000) {                          //Lectura del sensor de temperatura y humedad cada 5 segundos
    cloopTime2 = currentTime;
    for (int k=0;k<3;k++){
      tempAmb=dht.getTemperature();
      if(!isnan(tempAmb)) break;
      tempAmb=-127;
      delay(500);
    }
    for (int k=0;k<3;k++){
      humAmb=dht.getHumidity();
      if(!isnan(humAmb)) break;
      humAmb=-127;
      delay(500);
    }
  }
}