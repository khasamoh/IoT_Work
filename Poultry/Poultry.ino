#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h> //LIBRARY FOR HTTP
#include <SPI.h>
#include <Wire.h>
#include "MQ135.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"
#include <Arduino.h>

//test yetu
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire oled;

//end yetu

#define DHTPIN 0
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, & Wire, OLED_RESET);

int sensorVal = 0;
const int ANALOG_READ_PIN = A0; 
 
//String apiKey = "14K8UL2QEK8BTHN6"; // Enter your Write API key from ThingSpeak
//const char *ssid = "icc";     // replace with your wifi ssid and wpa2 key
//const char *pass = "11111111";
//const char* server = "api.thingspeak.com"

const char *ssid = "icc";     // replace with your wifi ssid and wpa2 key
const char *pass = "11111111";
const char* host = "api.waziup.io"; //WAZIUP SERVER
const char* devID = "D001"; //DEVICE ID - example 605db108af408600066050ae
const int http_port = 443; //HTTPS PORT

const char* sensorID_1 = "S001"; //SENSOR 1 ID -  temperatureSensor_1
const char* sensorID_1_1 = "S001-1"; //SENSOR 1-1 ID - Humidity
const char* sensorID_2 = "S002"; //SENSOR 2 ID -  PIR
const char* sensorID_3 = "S003"; //SENSOR 2 ID -  WaterLevel
const char* sensorID_4 = "S004"; //SENSOR 2 ID -  PhotoResistor
const char* sensorID_5 = "S005"; //SENSOR 2 ID -  GasSensor
 
WiFiClient client;
//Bulb,Fan pinmode
int PIR = 13;
int bulbrelay1Pin = 14;
int fanrelay1Pin = 12;
int pumprelayPin = 15;
void setup()
{
  dht.begin();
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //initialize with the I2C addr 0x3C (128x64)
  display.clearDisplay();
  delay(10);
 
  Serial.println("Connecting to ");
  Serial.println(ssid);
  
  display.clearDisplay();
  display.setCursor(0,0);  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("Connecting to ");
  display.setTextSize(2);
  display.print(ssid);
  display.display();
  
  WiFi.begin(ssid, pass);
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
    Serial.println("");
    Serial.println("WiFi connected");
    
    display.clearDisplay();
    display.setCursor(0,0);  
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print("WiFi connected");
    display.display();
    delay(4000);

    //test yetu
    Wire.begin();
  Wire.setClock(400000L);

  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0
    //end yetu
pinMode(PIR, INPUT);
pinMode(bulbrelay1Pin, OUTPUT);// define a pin as output for relay on or off
pinMode(fanrelay1Pin, OUTPUT);// define a pin as output for relay on or off
}
 
 
  void loop()
  {
    
    oled.setFont(System5x7);
    //oled.println("Welcome");
    temperature();
    gasSensor();
    photoResistor();
    waterLevel();
    PIRfun();
    delay(5000);
    //delay(2000);      // thingspeak needs minimum 15 sec delay between updates.

//  send_to_cloud(sensorID_1, 24); //SEDNING SENSOR 1 DATA
//
//  delay(2000);  //WAIT 2 SECONDS
//  send_to_cloud(sensorID_1-1, 24); //SEDNING SENSOR 1 DATA
//
//  delay(2000);  //WAIT 2 SECONDS
//  send_to_cloud(sensorID_2, 24); //SEDNING SENSOR 1 DATA
//
//  delay(2000);  //WAIT 2 SECONDS
//  send_to_cloud(sensorID_3, 24); //SEDNING SENSOR 1 DATA
//
//  delay(2000);  //WAIT 2 SECONDS
//
//  send_to_cloud(sensorID_4, 20.5); //SEDNING SENSOR 2 DATA
//  
//  delay(2000);  //WAIT 2 SECONDS

  //send_to_cloud(sensorID_5, gasSensor()); //SEDNING SENSOR 2 DATA
  
   //SENDING SENSOR DATA EVERY 10 SECONDS
}
void send_to_cloud(String sens_id, int sense_val) {
  
  //PARSING URL PATH - DONT TOUCH THIS
  String url = "/api/v2/devices/";
  url += devID;
  url += "/sensors/";
  url += sens_id;
  url += "/value";
  
  if (WiFi.status() == WL_CONNECTED) { //CHECK WIFI CONNECTION STATUS
    
    WiFiClientSecure client; //CONNECTION FOR HTTPS
    HTTPClient http;    //DECLARE OBJECT OF CLASS HTTPClient

    client.setInsecure(); //USING THIS BECAUSE WE DONT HAVE SSL CERTIFICATE FINGERPRINT
    
    http.begin(client ,host ,http_port ,url);      //SPECIFY REQUEST DESTINATION
     http.addHeader("Content-Type", "application/json;charset=utf-8");  //SPECIFY CONTENT TYPE
 
    int value = sense_val; //SENDING VALUES TO THE CLOUD
    char body[20]; //TO HOLD JSON DATA
    sprintf( body, "{\"value\": %d}", value);//PARSING & FORMATTING DATA
    
    int httpCode = http.POST(body); //SENDING JSON DATA
    
    // HTTP CODE WILL BE NEGATIVE IF THERE'S AN ERROR
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);

      // FILE FOUND ON SERVER
      if (httpCode == HTTP_CODE_OK) {
        const String& payload = http.getString();
        Serial.println("received payload:\n<<");
        Serial.println(payload);
        Serial.println(">>");
      }
    } else {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    
    http.end();  //CLOSING HTTP CONNECTION
 
  } else {
    Serial.println("WiFi Connection Fail");//IF ESP CAN'T CONNECTED TO WIFI
  }
}

void temperature(){
  // Wait a few seconds between measurements.
  //delay(2000);
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
  if(t < 34){
    digitalWrite(bulbrelay1Pin, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(fanrelay1Pin, LOW);
  }else if(t >= 35){
    digitalWrite(fanrelay1Pin, HIGH);
    digitalWrite(bulbrelay1Pin, HIGH);
  }else{
    digitalWrite(bulbrelay1Pin, HIGH);    // turn the LED off by making the voltage LOW
    digitalWrite(fanrelay1Pin, LOW);
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));

  oled.setCursor(0,0);
  oled.println();
  oled.print(F("Humidity: "));
  oled.print(h);
  oled.println(F("%"));
  oled.println();
  oled.print(F("Temperature: "));
  oled.println(t);
  oled.print(F("°C "));
  oled.println();
    send_to_cloud(sensorID_1, t);
    send_to_cloud(sensorID_1_1, h);
}

void gasSensor(){
  MQ135 gasSensor = MQ135(A0);
    float air_quality = gasSensor.getPPM();
    Serial.print("Air Quality: ");  
    Serial.print(air_quality);
    Serial.println("  PPM");   
    Serial.println();
 
  oled.setCursor(0,30);
  oled.print(F("Air Quality: "));
  oled.print(air_quality);
  oled.println(F("%"));
  oled.println();
  send_to_cloud(sensorID_5, air_quality);
 // return air_quality;
 
}


void photoResistor(){
  sensorVal = analogRead(ANALOG_READ_PIN);

  // Values from 0-1024
  Serial.println(sensorVal);

  // Convert the analog reading to voltage
  float voltage = sensorVal * (3.3 / 1023.0);

  // print the voltage
  Serial.println(voltage);

  oled.setCursor(0,50);
  oled.println();
  oled.print(F("Light: "));
  oled.print(voltage);
  oled.print(F("V "));
  oled.print(sensorVal);
  oled.println(F("A0"));
  send_to_cloud(sensorID_4, voltage);
  }

void waterLevel(){
  int volume = analogRead(A0);
  send_to_cloud(sensorID_3, volume);
  if(volume>=500){
    digitalWrite(pumprelayPin, LOW);
  }else{
    digitalWrite(pumprelayPin, HIGH);
  }
  Serial.println(volume);   
}         

void PIRfun(){
  int motion =  digitalRead(PIR);
  send_to_cloud(sensorID_2, motion);
}