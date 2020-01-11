#include <Arduino.h>
#include "FastLED.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

OneWire oneWire(ONE_WIRE_BUS); //

//Übergabe der OnewWire Referenz zum kommunizieren mit dem Sensor.
DallasTemperature sensors(&oneWire);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}

void RGBLoop(){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAll(k,0,0); break;
        case 1: setAll(0,k,0); break;
        case 2: setAll(0,0,k); break;
      }
      showStrip();
      delay(3);
    }
    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAll(k,0,0); break;
        case 1: setAll(0,k,0); break;
        case 2: setAll(0,0,k); break;
      }
      showStrip();
      delay(3);
    }
  }
}

void BouncingBalls(byte red, byte green, byte blue, int BallCount) {
  float Gravity = -9.81;
  int StartHeight = 1;
  
  float Height[BallCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[BallCount];
  float TimeSinceLastBounce[BallCount];
  int   Position[BallCount];
  long  ClockTimeSinceLastBounce[BallCount];
  float Dampening[BallCount];
  
  for (int i = 0 ; i < BallCount ; i++) {   
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0; 
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i)/pow(BallCount,2); 
  }

  for (int i=0; i<5000; i++) {
    wdt_reset();
    for (int i = 0 ; i < BallCount ; i++) {
      TimeSinceLastBounce[i] =  millis() - ClockTimeSinceLastBounce[i];
      Height[i] = 0.5 * Gravity * pow( TimeSinceLastBounce[i]/1000 , 2.0 ) + ImpactVelocity[i] * TimeSinceLastBounce[i]/1000;
  
      if ( Height[i] < 0 ) {                      
        Height[i] = 0;
        ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
        ClockTimeSinceLastBounce[i] = millis();
  
        if ( ImpactVelocity[i] < 0.01 ) {
          ImpactVelocity[i] = ImpactVelocityStart;
        }
      }
      Position[i] = round( Height[i] * (NUM_LEDS - 1) / StartHeight);
    }
  
    for (int i = 0 ; i < BallCount ; i++) {
      setPixel(Position[i],red,green,blue);
    }
    
    showStrip();
    setAll(0,0,0);
  }
}

void BouncingColoredBalls(int BallCount, byte colors[][3]) {
  float Gravity = -9.81;
  int StartHeight = 1;
  
  float Height[BallCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[BallCount];
  float TimeSinceLastBounce[BallCount];
  int   Position[BallCount];
  long  ClockTimeSinceLastBounce[BallCount];
  float Dampening[BallCount];
  
  for (int i = 0 ; i < BallCount ; i++) {   
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0; 
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i)/pow(BallCount,2); 
  }

  for (int h=0; h<5000; h++) {
    wdt_reset ();
    for (int i = 0 ; i < BallCount ; i++) {
      TimeSinceLastBounce[i] =  millis() - ClockTimeSinceLastBounce[i];
      Height[i] = 0.5 * Gravity * pow( TimeSinceLastBounce[i]/1000 , 2.0 ) + ImpactVelocity[i] * TimeSinceLastBounce[i]/1000;
  
      if ( Height[i] < 0 ) {                      
        Height[i] = 0;
        ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
        ClockTimeSinceLastBounce[i] = millis();
  
        if ( ImpactVelocity[i] < 0.01 ) {
          ImpactVelocity[i] = ImpactVelocityStart;
        }
      }
      Position[i] = round( Height[i] * (NUM_LEDS - 1) / StartHeight);
    }
  
    for (int i = 0 ; i < BallCount ; i++) {
      setPixel(Position[i],colors[i][0],colors[i][1],colors[i][2]);
    }
    
    showStrip();
    setAll(0,0,0);
  }
}


void setPixelHeatColor (int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixel(Pixel, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixel(Pixel, 255, heatramp, 0);
  } else {                               // coolest
    setPixel(Pixel, heatramp, 0, 0);
  }
}


void Fire(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[NUM_LEDS];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < NUM_LEDS; i++) {
    cooldown = random(0, ((Cooling * 10) / NUM_LEDS) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < NUM_LEDS; j++) {
    setPixelHeatColor(j, heat[j] );
  }

  showStrip();
  delay(SpeedDelay);
}

void printValue(float value, String text){
  Serial.print("\t\t");
  Serial.print(value);
  Serial.println(text);
}

void LEDAnzeige(){ //Hier wird die Temperatur auf die LED übertragen (inkl. Skala)
  int numLedsToLight = map(temp, min_Angezeigte_Temperatur, max_Angezeigte_Temperatur, 0, NUM_LEDS);
  for(int led =0; led <= numLedsToLight; led++) {
    //leds[led]=CRGB(55,0,0);
    setPixel(led, 55,0,0);
  }
  
  //Hier die Skala einbauen
  for(int zehner = min_Angezeigte_Temperatur; zehner <= max_Angezeigte_Temperatur; zehner = zehner +10){
    int Zehnerstelle = map(zehner, min_Angezeigte_Temperatur, max_Angezeigte_Temperatur, 0, NUM_LEDS);
    Serial.print("Zehnerstellen: ");
    Serial.println(Zehnerstelle);
    //leds[Zehnerstelle]=CRGB(0,55,0);
    setPixel(Zehnerstelle, 0,55,0);
  }

    for(int fuenfer = min_Angezeigte_Temperatur+5; fuenfer <= max_Angezeigte_Temperatur; fuenfer = fuenfer +10){
    int Fuenferstelle = map(fuenfer, min_Angezeigte_Temperatur, max_Angezeigte_Temperatur, 0, NUM_LEDS);
    //Serial.print("Fuenferstellen: ");
    //Serial.println(Fuenferstelle);
    //leds[Fuenferstelle]=CRGB(55,55,0);
    setPixel(Fuenferstelle, 0, 55, 55);
  }
  
  
  //FastLED.show();
  showStrip();
  Serial.print("NUMTOLIGHT ");
  Serial.println(numLedsToLight);
}

void TempMessung(){
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
}

void Temperaturanzeige(){
  TempMessung();
  //FastLED.clear();
  setAll(0,0,0);
  LEDAnzeige();
}

void setup()
{
 Serial.begin(9600); //Starten der seriellen Kommunikation mit 9600 baud
 Serial.println("Temperatursensor - DS18B20"); 
 sensors.begin(); //Starten der Kommunikation mit dem Sensor
 sensorCount = sensors.getDS18Count(); //Lesen der Anzahl der angeschlossenen Temperatursensoren.
 
 FastLED.addLeds<WS2811, PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );


 // limit my draw to xA at 5v of power draw
 FastLED.setMaxPowerInVoltsAndMilliamps(5,maxStrom);
 wdt_disable (); //wdt_enable(WDTO_8S);

 WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  timeClient.begin();
}


void loop() { 
  timeClient.update();

  Serial.println(timeClient.getFormattedTime());

  do {
    Fire(55,120,15);
    zaehler++;
    wdt_reset ();
  } while (zaehler<1000);
  zaehler = 0;
  
  
Temperaturanzeige();
for (int i=0; i<10;i++){
  wdt_reset ();
  delay (1000);
}
  

//BouncingBalls(0xff,0,0, 3);

byte colors[3][3] = { {0xff, 0,0}, 
                     {0, 0xff, 0}, 
                    {00, 0 ,0xff} };
BouncingColoredBalls(3, colors);
  
BouncingBalls(0xff,0,0, 3);
  //Temperaturanzeige();
  //delay(1000); //Pause von 1 Sekunde.
  }



