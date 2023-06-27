// C++ code
#include <LiquidCrystal.h>
#include "DHT.h"
//======================LCD===
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire oled;
//======================END======
//LiquidCrystal lcd_1(12, 11, 5, 4, 3, 2);
DHT dht(9, DHT11);
int output_value;
void setup()
{
  //=========================
  Wire.begin();
  Wire.setClock(400000L);

  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0
  
  //=========================
  Serial.begin(38400);
  //Soil&LED
  pinMode(A1, INPUT);
  pinMode(8, OUTPUT);
  pinMode(7,OUTPUT); // WaterPin
  digitalWrite(7, HIGH);
  //
  pinMode(A0, INPUT);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
  //
  dht.begin();
}
void loop()
{
  oled.setFont(System5x7);
  oled.println("Welcome");
  
  DHT11Function(); // About temperature or humidity
  soilMoisture(); // About Soil moisture
  waterLevel(); // About water Level
  
}

void DHT11Function(){
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  if(hic > 36.0){
      //digitalWrite(8, HIGH);
    }else{
      //digitalWrite(8, LOW);
      }
  oled.print(F("Humidity: "));
  oled.print(h);
  oled.println(F("%"));
  oled.print(F("Temperature: "));
  oled.println(t);
  oled.print(F("°C "));
  ///=================
//  Serial.print(F("Humidity: "));
//  Serial.print(h);
//  Serial.print(F("%  Temperature: "));
//  Serial.print(t);
//  Serial.print(F("°C "));
//  Serial.print(f);
//  Serial.print(F("°F  Heat index: "));
//  Serial.print(hic);
//  Serial.print(F("°C "));
//  Serial.print(hif);
//  Serial.println(F("°F"));
  }

  void soilMoisture(){
    int output_value = analogRead(A1);
    output_value = map(output_value,0,982,148,0);
    if(output_value > 100) {
      output_value = 100;
    } else if(output_value < 0) {
      output_value = 0;
    }
    if(output_value < 10) {
      if(waterLevel()==1){
        digitalWrite(7, LOW);
        digitalWrite(8, HIGH);
        //oled.setFont(System5x7);
        oled.clear();
        oled.print("Irrigation Going On");
        }else{
          digitalWrite(7, HIGH);
          digitalWrite(8, LOW);
          }
   } else {
        digitalWrite(7, HIGH);
        digitalWrite(8, LOW);
        //oled.setFont(System5x7);
        oled.clear();
        oled.print("Irrigation OFF");
    }
//    Serial.print("Mositure : ");
//    Serial.print(output_value);
//    Serial.println("%");

    oled.print(F("Mositure: "));
    oled.print(output_value);
    oled.println(F("%"));
    delay(1000);
 }
 
int waterLevel(){
  digitalWrite(6, HIGH);
  delay(10); 
  int volume = analogRead(A0);
  digitalWrite(6, LOW);
  oled.print(F("WaterLevel: "));
  oled.print(volume);
  oled.println(F("%"));
  Serial.println("Water");
 Serial.println(volume);
  if(volume < 300){
      digitalWrite(8, HIGH);
//      oled.setFont(System5x7);
//      oled.clear();
      oled.println("Tank is Empty");
      oled.println("No irrigation");
      return 0;
    }else{
      digitalWrite(8, LOW);
      return 1;
      }
 Serial.println("Water");
 Serial.println(volume);
  //delay(2000);                 
  }
