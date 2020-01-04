#include <Arduino.h>
//Einbinden der Bibliotheken
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FastLED.h>

#define ONE_WIRE_BUS 2 //Sensor DS18B20 am digitalen Pin 2
#define NUM_LEDS 10
#define DATA_PIN 0
CRGB leds[NUM_LEDS];

OneWire oneWire(ONE_WIRE_BUS); //

//Übergabe der OnewWire Referenz zum kommunizieren mit dem Sensor.
DallasTemperature sensors(&oneWire);

int sensorCount;
int Messung = 0;
int temp = 60;

int maxStrom = 1000; 

int min_Angezeigte_Temperatur = 20;  //minimale Temperatur der Anzeige
int max_Angezeigte_Temperatur = 30;  //maximale Temperatur der Anzeige

void printValue(float value, String text){
  Serial.print("\t\t");
  Serial.print(value);
  Serial.println(text);
}

void LEDAnzeige(){ //Hier wird die Temperatur auf die LED übertragen (inkl. Skala)
  int numLedsToLight = map(temp, min_Angezeigte_Temperatur, max_Angezeigte_Temperatur, 0, NUM_LEDS);
  for(int led =0; led <= numLedsToLight; led++) {
    leds[led]=CRGB(55,0,0);
  }
  
  //Hier die Skala einbauen
  for(int zehner = min_Angezeigte_Temperatur; zehner <= max_Angezeigte_Temperatur; zehner = zehner +10){
    int Zehnerstelle = map(zehner, min_Angezeigte_Temperatur, max_Angezeigte_Temperatur, 0, NUM_LEDS);
    Serial.print("Zehnerstellen: ");
    Serial.println(Zehnerstelle);
    leds[Zehnerstelle]=CRGB(0,55,0);
  }

    for(int fuenfer = min_Angezeigte_Temperatur+5; fuenfer <= max_Angezeigte_Temperatur; fuenfer = fuenfer +10){
    int Fuenferstelle = map(fuenfer, min_Angezeigte_Temperatur, max_Angezeigte_Temperatur, 0, NUM_LEDS);
    //Serial.print("Fuenferstellen: ");
    //Serial.println(Fuenferstelle);
    leds[Fuenferstelle]=CRGB(55,55,0);
  }
  
  
  FastLED.show();
  Serial.print("NUMTOLIGHT ");
  Serial.println(numLedsToLight);
}




void setup(void) { 
 Serial.begin(9600); //Starten der seriellen Kommunikation mit 9600 baud
 Serial.println("Temperatursensor - DS18B20"); 
 sensors.begin(); //Starten der Kommunikation mit dem Sensor
 sensorCount = sensors.getDS18Count(); //Lesen der Anzahl der angeschlossenen Temperatursensoren.
 FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
 
 // limit my draw to 1A at 5v of power draw
   FastLED.setMaxPowerInVoltsAndMilliamps(5,maxStrom);
} 

void loop(void){ 
 if(sensorCount ==0){
   Serial.println("Es wurde kein Temperatursensor gefunden!");
   Serial.println("Bitte überprüfe deine Schaltung!");
 }
 //Es können mehr als 1 Temperatursensor am Datenbus angschlossen werden.
 //Anfordern der Temperaturwerte aller angeschlossenen Temperatursensoren.
 sensors.requestTemperatures(); 

 //Ausgabe aller Werte der angeschlossenen Temperatursensoren.
 for(int i=0;i<sensorCount;i++){
  Serial.print(i); 
  Serial.print(". Temperatur: "); 
  Serial.println(Messung);
  Messung = Messung + 1;
  printValue(sensors.getTempCByIndex(i), "°C");
  //printValue(sensors.getTempFByIndex(i), "°F");
  temp=sensors.getTempCByIndex(i);
  Serial.print("temp= ");
  Serial.println(temp);
 }
 FastLED.clear();
 LEDAnzeige();
 delay(1000); //Pause von 1 Sekunde.
} 

