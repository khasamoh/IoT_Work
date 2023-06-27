
#include <Fuzzy.h>
#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include "pitches.h"
#include <LiquidCrystal_I2C.h>
#include "Protocentral_MAX30205.h"
#include "MAX30102_PulseOximeter.h"

#define REPORTING_PERIOD_MS     1000
MAX30205 tempSensor;
PulseOximeter pox;
uint32_t tsLastReport = 0;
// Callback (registered below) fired when a pulse is detected 
void onBeatDetected()
{
    Serial.println("Beat!");
}

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

Fuzzy *fuzzy = new Fuzzy();

 FuzzyOutput *Triage_Level;
 FuzzySet *Triage_Critical;
 FuzzySet *Triage_Urgent;
 FuzzySet *Triage_Standard;
 int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };
volatile float temperature[100];
volatile float spo2[100];
volatile float heart_rate[100];
volatile int avgTemp = 0;
volatile int avgSpo2 = 0;
volatile bool reading_state = LOW;

volatile int avgHeart = 0;
volatile uint8_t array_index = 0;
// constants won't change. They're used here to set pin numbers:
const int buttonPin = D6;  // the number of the pushbutton pin
const int ledPin = 13;    // the number of the LED pin
const int buzzerPin = D8;
const int redPin = D3; // select the pin for the red LED
const int bluePin = D4; // select the pin for the blue LED
const int greenPin = D5 ;// select the pin for the green LED
// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status
volatile bool state = LOW;

void ICACHE_RAM_ATTR buttonPressed(){
  state = !state;
}
void setup() {
    Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on  
  // Print a message on both lines of the LCD.
  lcd.setCursor(3,0);   //Set cursor to character 2 on line 0
  lcd.print("Welcome"); 
  lcd.setCursor(0,1);   //Set cursor to character 2 on line 0
  lcd.print("Patient Triage");    
  attachInterrupt(buttonPin, buttonPressed, RISING);
  fuzzy_process();
  Wire.begin();

 //Wire1.setSDA(6);
 // Wire1.setSCL(7);

  // Initialize the PulseOximeter instance
    if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
    // The default current for the IR LED is 50mA and is changed below
    pox.setIRLedCurrent(MAX30102_LED_CURR_7_6MA);
    // Register a callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);
 //Wire1.setSDA(6);
 // Wire1.setSCL(7);

  // tempSensor.scanAvailableSensors();
  tempSensor.begin(Wire, true, MAX30205_ADDRESS2);

  delay (5000);
}

void loop() {

  float temp = tempSensor.getTemperature(); // read temperature for every 100ms
  Serial.print(temp, 2);
  Serial.println("'c");
  delay(100);

  // Make sure to call update as fast as possible
    pox.update();
   // long irValue = pox.getHeartRate();
    // Asynchronously dump heart rate and oxidation levels to the serial
    // For both, a value of 0 means "invalid"
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
        Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.print("%");
 // if (irValue < 70)
   // Serial.print(",  No finger?");
    Serial.println();
         
        tsLastReport = millis();
    }

  //sensorReadingTask(); //nimecomment hii kuangalia izo sensor 2 kuread data tu kawaida ila haifnyi kazi,ila temperature inafnya. nimefikia hapo ila ktk file lingine nimezirun pamoja zinafnya kazi vzr t
  //delay(10);
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
}

void sensorReadingTask()
{
   pox.update();
  #define SENSOR_READING_INTERVAL  50
  static uint32_t previousMillis = 0;
  uint32_t currentMillis = millis();
  if ( currentMillis - previousMillis < SENSOR_READING_INTERVAL )
  {
    return;
  }
  previousMillis = currentMillis;
  //Serial.print("Heart rate:");

  // float t = random(350,400);
  // float h = random(400,1600);
  // float s = random(700,1000); 
  float h = pox.getHeartRate()*10;
  float s = pox.getSpO2()*10;
  float t = tempSensor.getTemperature()*10;

  //>>>>>>>>>>>>
  float temp = tempSensor.getTemperature(); // read temperature for every 100ms
  Serial.print(temp, 2);
  Serial.println("'c");
  delay(100);

  // Make sure to call update as fast as possible
    pox.update();
   // long irValue = pox.getHeartRate();
    // Asynchronously dump heart rate and oxidation levels to the serial
    // For both, a value of 0 means "invalid"
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("Heart rate:");
        Serial.print(pox.getHeartRate());
        Serial.print("bpm / SpO2:");
        Serial.print(pox.getSpO2());
        Serial.print("%");
 // if (irValue < 70)
   // Serial.print(",  No finger?");
    Serial.println();
         
        tsLastReport = millis();
    }

    
  if(state){

  Serial.print( "State: " );
  Serial.print(state );
   Serial.print( "Temp: " );
  Serial.print(t );
  Serial.print( " spo2: " );
  Serial.print(s );
  Serial.print( " heartf_rate: " );
  Serial.print(h);



   if(h>=400 && s>=700 && t >=350 && array_index<=99){
  //temperature[array_index] = random(200, 490)/10.0;
  temperature[array_index] = t;
//  spo2[array_index] = random(700, 1110)/10.0;
//  heart_rate[array_index] = random(440, 1390)/10.0;
  spo2[array_index] = s;
  heart_rate[array_index] = h;
  Serial.print(" index: " );
  Serial.print(array_index );
  array_index++;
  }
  Serial.print(" in: " );
  Serial.print(array_index );


 if(array_index==100){
      reading_state =HIGH;   
      int len = sizeof(temperature)/sizeof(float);
      avgTemp = average(temperature,100 );
      avgSpo2 = average(spo2, 100);
      avgHeart = average(heart_rate, 100);
      lcd.setCursor(0,0);   //Move cursor to character 2 on line 1
      double temp = avgTemp/10.0;
      int sp = (int)(avgSpo2/10);
      int hr = (int)(avgHeart/10);      
      lcd.print(round(temp*10)/10);
      lcd.print("C ");
     
      lcd.print(sp);
      lcd.print("% "); 
      lcd.print(hr);
      lcd.print("bpm");  
      Serial.print( "Avg Temp: " );
      Serial.print(avgTemp);
      Serial.print( " Avg spo2: " );
      Serial.print(avgSpo2);
      Serial.print( " Avg heart_rate: " );
      Serial.print(avgHeart );   

    fuzzy->setInput(1, avgTemp);
    fuzzy->setInput(2, avgHeart);
    fuzzy->setInput(3,  avgSpo2);
  
    // Running the Fuzzification
    fuzzy->fuzzify();
    // Running the Defuzzification
    float Triage_Level_output = fuzzy->defuzzify(1);
  
    // Printing Inputs and Outputs
    
    Serial.print(" Triage_Level_output: ");
    Serial.print(Triage_Level_output);  

    Serial.print("Triage_Level_Critical: ");
    Serial.print(Triage_Critical->getPertinence());
    Serial.print(" Triage_Level_Urgent: ");
    Serial.print(Triage_Urgent->getPertinence());
    Serial.print(" Triage_Level_Standard: ");
    Serial.print(Triage_Standard->getPertinence());
  
    double triage_fuzzy_values[3] = {Triage_Critical->getPertinence(), Triage_Urgent->getPertinence(), Triage_Standard->getPertinence()};
    int triage_index = maxArrayIndex(triage_fuzzy_values);
    String Triage_lab[] = {"Critical", "Urgent", "Standard"};
    Serial.print(" Sizeof: ");
    Serial.print(sizeof(triage_fuzzy_values)/sizeof(double));
    Serial.print(" Triage_index: ");
    Serial.print(triage_index);
    Serial.print(" Triage_Level Lable: ");
    Serial.println(Triage_lab[triage_index]);
    lcd.setCursor(0,1);
    lcd.print("Status: ");     
    lcd.print(Triage_lab[triage_index]);       
    lcd.print("   ");     
    
    actuation(triage_index);
  
             
          array_index=0;
 }else{
if(!reading_state){  
  lcd.setCursor(0,0);   //Set cursor to character 2 on line 0
  lcd.print("Sensor Reading.."); 
  lcd.setCursor(0,1);   //Set cursor to character 2 on line 0
  lcd.print("                ");  
}
 }
   Serial.println("");
  
  }else{
   analogWrite(redPin, 255);
   analogWrite(greenPin, 255);
   analogWrite(bluePin, 255);        
  noTone(buzzerPin);
   lcd.clear();         
  // Print a message on both lines of the LCD.
  lcd.setCursor(3,0);   //Set cursor to character 2 on line 0
  lcd.print("Welcome"); 
  lcd.setCursor(0,1);   //Set cursor to character 2 on line 0
  lcd.print("Patient Triage");
  array_index=0;
  reading_state=LOW;
  
 }
  //counter = 0;
  
}
int average(volatile float * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  (int)(((float) sum) / len) ;  // average will be fractional, so float may be appropriate.
}

void actuation(int Triage_index){
  if (Triage_index==2){
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 0);
   // Serial.print("noTone(buzzerPin)");    
    noTone(buzzerPin);
    
  }else if(Triage_index==1){
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 255);   
        Serial.print("tone(buzzerPin, 784);");    
    tone(buzzerPin, NOTE_G3);
  }
  else if(Triage_index==0){
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0); 
    Serial.print("tone(buzzerPin, *****);");    
    tone(buzzerPin, NOTE_C4);    
  }
}

void fuzzy_process(){
 FuzzyInput *temperature = new FuzzyInput(1);
 FuzzySet *t_low = new FuzzySet(285, 385, 320, 330);
 temperature->addFuzzySet(t_low);
  FuzzySet *t_normal = new FuzzySet(320, 370, 370, 380);
 temperature->addFuzzySet(t_normal);
  FuzzySet *t_high = new FuzzySet(375, 380, 400, 400);
 temperature->addFuzzySet(t_high);



  FuzzyInput *heart_rate = new FuzzyInput(2);
 FuzzySet *heart_rate_low = new FuzzySet(400, 400, 600, 800);
 heart_rate->addFuzzySet(heart_rate_low);
 FuzzySet *heart_rate_normal = new FuzzySet(600, 800, 800, 1200);
 heart_rate->addFuzzySet(heart_rate_normal);
  FuzzySet *heart_rate_high = new FuzzySet(1000, 1200, 1600, 1600);
 heart_rate->addFuzzySet(heart_rate_high);
 
 //multiply by 100
 FuzzyInput *spo2 = new FuzzyInput(3);

 FuzzySet *spo2_low = new FuzzySet(700, 700, 850, 900);
 spo2->addFuzzySet(spo2_low);
 FuzzySet *spo2_normal = new FuzzySet(850, 900, 900, 960);
 spo2->addFuzzySet(spo2_normal);
   FuzzySet *spo2_high = new FuzzySet(950, 960, 1000, 1000);
 spo2->addFuzzySet(spo2_high);

 fuzzy->addFuzzyInput(temperature);
 fuzzy->addFuzzyInput(heart_rate);
 fuzzy->addFuzzyInput(spo2);

//  FuzzyOutput *Triage_Level;
//  FuzzySet *Triage_Critical;
//  FuzzySet *Triage_Urgent;
//  FuzzySet *Triage_Standard;

 Triage_Level = new FuzzyOutput(1);
 Triage_Critical = new FuzzySet(0, 0, 25, 50);
 Triage_Level->addFuzzySet(Triage_Critical);
 Triage_Urgent = new FuzzySet(45, 75, 75, 105);
 Triage_Level->addFuzzySet(Triage_Urgent);
 Triage_Standard = new FuzzySet(100, 125, 150, 150);
 Triage_Level->addFuzzySet(Triage_Standard);
 
 fuzzy->addFuzzyOutput(Triage_Level);

 FuzzyRuleConsequent *then_Triage_Critical = new FuzzyRuleConsequent();
 then_Triage_Critical->addOutput(Triage_Critical);
 FuzzyRuleConsequent *then_Triage_Urgent = new FuzzyRuleConsequent();
 then_Triage_Urgent->addOutput(Triage_Urgent);
 FuzzyRuleConsequent *then_Triage_Standard = new FuzzyRuleConsequent();
 then_Triage_Standard->addOutput(Triage_Standard);

 FuzzyRuleAntecedent *if_t_low_and_heart_rate_low = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_low->joinWithAND(t_low, heart_rate_low);
 FuzzyRuleAntecedent *if_spo2_low = new FuzzyRuleAntecedent();
 if_spo2_low->joinSingle(spo2_low);
 
 FuzzyRuleAntecedent *if_t_low_and_heart_rate_low_and_spo2_low = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_low_and_spo2_low->joinWithAND(if_t_low_and_heart_rate_low, if_spo2_low);
 FuzzyRule *fuzzyRule1 = new FuzzyRule(1, if_t_low_and_heart_rate_low_and_spo2_low, then_Triage_Critical);
 fuzzy->addFuzzyRule(fuzzyRule1);

 
 FuzzyRuleAntecedent *if_spo2_normal = new FuzzyRuleAntecedent();
 if_spo2_normal->joinSingle(spo2_normal); 
 FuzzyRuleAntecedent *if_t_low_and_heart_rate_low_and_spo2_normal = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_low_and_spo2_normal->joinWithAND(if_t_low_and_heart_rate_low, if_spo2_normal);
 FuzzyRule *fuzzyRule2 = new FuzzyRule(2, if_t_low_and_heart_rate_low_and_spo2_normal, then_Triage_Critical);
 fuzzy->addFuzzyRule(fuzzyRule2);


FuzzyRuleAntecedent *if_spo2_high = new FuzzyRuleAntecedent();
 if_spo2_high->joinSingle(spo2_high); 
 FuzzyRuleAntecedent *if_t_low_and_heart_rate_low_and_spo2_high = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_low_and_spo2_high->joinWithAND(if_t_low_and_heart_rate_low, if_spo2_high);
 FuzzyRule *fuzzyRule3 = new FuzzyRule(3, if_t_low_and_heart_rate_low_and_spo2_high, then_Triage_Critical);
 fuzzy->addFuzzyRule(fuzzyRule3);
 
 
 FuzzyRuleAntecedent *if_t_low_and_heart_rate_normal = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_normal->joinWithAND(t_low, heart_rate_normal);
 FuzzyRuleAntecedent *if_t_low_and_heart_rate_normal_and_spo2_low = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_normal_and_spo2_low->joinWithAND(if_t_low_and_heart_rate_normal, if_spo2_low);
 FuzzyRule *fuzzyRule4 = new FuzzyRule(4, if_t_low_and_heart_rate_normal_and_spo2_low, then_Triage_Critical);
 fuzzy->addFuzzyRule(fuzzyRule4);


 FuzzyRuleAntecedent *if_t_low_and_heart_rate_normal_and_spo2_normal = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_normal_and_spo2_normal->joinWithAND(if_t_low_and_heart_rate_normal, if_spo2_normal);
 FuzzyRule *fuzzyRule5 = new FuzzyRule(5, if_t_low_and_heart_rate_normal_and_spo2_normal, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule5);

 FuzzyRuleAntecedent *if_t_low_and_heart_rate_normal_and_spo2_high = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_normal_and_spo2_high->joinWithAND(if_t_low_and_heart_rate_normal, if_spo2_high);
 FuzzyRule *fuzzyRule6 = new FuzzyRule(6, if_t_low_and_heart_rate_normal_and_spo2_high, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule6); 

 FuzzyRuleAntecedent *if_t_low_and_heart_rate_high = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_high->joinWithAND(t_low, heart_rate_high);

 FuzzyRuleAntecedent *if_t_low_and_heart_rate_high_and_spo2_low = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_high_and_spo2_low->joinWithAND(if_t_low_and_heart_rate_high, if_spo2_low);
 FuzzyRule *fuzzyRule7 = new FuzzyRule(7, if_t_low_and_heart_rate_high_and_spo2_low, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule7); 
 

 FuzzyRuleAntecedent *if_t_low_and_heart_rate_high_and_spo2_normal = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_high_and_spo2_normal->joinWithAND(if_t_low_and_heart_rate_high, if_spo2_normal);
 FuzzyRule *fuzzyRule8 = new FuzzyRule(8, if_t_low_and_heart_rate_high_and_spo2_normal, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule8); 

 FuzzyRuleAntecedent *if_t_low_and_heart_rate_high_and_spo2_high = new FuzzyRuleAntecedent();
 if_t_low_and_heart_rate_high_and_spo2_high->joinWithAND(if_t_low_and_heart_rate_high, if_spo2_high);
 FuzzyRule *fuzzyRule9 = new FuzzyRule(9, if_t_low_and_heart_rate_high_and_spo2_high, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule9); 


 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_low = new FuzzyRuleAntecedent();
 if_t_normal_and_heart_rate_low->joinWithAND(t_normal, heart_rate_low);
 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_low_and_spo2_low = new FuzzyRuleAntecedent();
 if_t_normal_and_heart_rate_low_and_spo2_low->joinWithAND(if_t_normal_and_heart_rate_low, if_spo2_low);
 FuzzyRule *fuzzyRule10 = new FuzzyRule(10, if_t_normal_and_heart_rate_low_and_spo2_low, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule10);

 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_low_and_spo2_normal = new FuzzyRuleAntecedent();
 if_t_normal_and_heart_rate_low_and_spo2_normal->joinWithAND(if_t_normal_and_heart_rate_low, if_spo2_normal);
 FuzzyRule *fuzzyRule11 = new FuzzyRule(11, if_t_normal_and_heart_rate_low_and_spo2_normal, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule11);
 
 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_low_and_spo2_high = new FuzzyRuleAntecedent();
 if_t_normal_and_heart_rate_low_and_spo2_high->joinWithAND(if_t_normal_and_heart_rate_low, if_spo2_high);
 FuzzyRule *fuzzyRule12 = new FuzzyRule(12, if_t_normal_and_heart_rate_low_and_spo2_high, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule12);

 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_normal = new FuzzyRuleAntecedent();
 if_t_normal_and_heart_rate_normal->joinWithAND(t_normal, heart_rate_normal);
 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_normal_and_spo2_low = new FuzzyRuleAntecedent();
 if_t_normal_and_heart_rate_normal_and_spo2_low->joinWithAND(if_t_normal_and_heart_rate_normal, if_spo2_low);
 FuzzyRule *fuzzyRule13 = new FuzzyRule(13, if_t_normal_and_heart_rate_normal_and_spo2_low, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule13);
 
 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_normal_and_spo2_normal = new FuzzyRuleAntecedent();
 if_t_normal_and_heart_rate_normal_and_spo2_normal->joinWithAND(if_t_normal_and_heart_rate_normal, if_spo2_normal);
 FuzzyRule *fuzzyRule14 = new FuzzyRule(14, if_t_normal_and_heart_rate_normal_and_spo2_normal, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule14);

 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_normal_and_spo2_high = new FuzzyRuleAntecedent();
 if_t_normal_and_heart_rate_normal_and_spo2_high->joinWithAND(if_t_normal_and_heart_rate_normal, if_spo2_high);
 FuzzyRule *fuzzyRule15 = new FuzzyRule(15, if_t_normal_and_heart_rate_normal_and_spo2_high, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule15); 

 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_high = new FuzzyRuleAntecedent();
 if_t_normal_and_heart_rate_high->joinWithAND(t_normal, heart_rate_high);
 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_high_and_spo2_low = new FuzzyRuleAntecedent();
  if_t_normal_and_heart_rate_high_and_spo2_low->joinWithAND(if_t_normal_and_heart_rate_high, if_spo2_low);
 FuzzyRule *fuzzyRule16 = new FuzzyRule(16, if_t_normal_and_heart_rate_high_and_spo2_low, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule16); 

 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_high_and_spo2_normal = new FuzzyRuleAntecedent();
  if_t_normal_and_heart_rate_high_and_spo2_normal->joinWithAND(if_t_normal_and_heart_rate_high, if_spo2_normal);
 FuzzyRule *fuzzyRule17 = new FuzzyRule(17, if_t_normal_and_heart_rate_high_and_spo2_normal, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule17); 

 FuzzyRuleAntecedent *if_t_normal_and_heart_rate_high_and_spo2_high = new FuzzyRuleAntecedent();
  if_t_normal_and_heart_rate_high_and_spo2_high->joinWithAND(if_t_normal_and_heart_rate_high, if_spo2_high);
 FuzzyRule *fuzzyRule18 = new FuzzyRule(18, if_t_normal_and_heart_rate_high_and_spo2_high, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule18);

 FuzzyRuleAntecedent *if_t_high_and_heart_rate_low = new FuzzyRuleAntecedent();
 if_t_high_and_heart_rate_low->joinWithAND(t_high, heart_rate_low);
 FuzzyRuleAntecedent *if_t_high_and_heart_rate_low_and_spo2_low = new FuzzyRuleAntecedent();
  if_t_high_and_heart_rate_low_and_spo2_low->joinWithAND(if_t_high_and_heart_rate_low, if_spo2_low);
 FuzzyRule *fuzzyRule19 = new FuzzyRule(19, if_t_high_and_heart_rate_low_and_spo2_low, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule19);

 FuzzyRuleAntecedent *if_t_high_and_heart_rate_low_and_spo2_normal = new FuzzyRuleAntecedent();
  if_t_high_and_heart_rate_low_and_spo2_normal->joinWithAND(if_t_high_and_heart_rate_low, if_spo2_normal);
 FuzzyRule *fuzzyRule20 = new FuzzyRule(20, if_t_high_and_heart_rate_low_and_spo2_normal, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule20);
 
 FuzzyRuleAntecedent *if_t_high_and_heart_rate_low_and_spo2_high = new FuzzyRuleAntecedent();
  if_t_high_and_heart_rate_low_and_spo2_high->joinWithAND(if_t_high_and_heart_rate_low, if_spo2_high);
 FuzzyRule *fuzzyRule21 = new FuzzyRule(21, if_t_high_and_heart_rate_low_and_spo2_high, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule21);
 
  FuzzyRuleAntecedent *if_t_high_and_heart_rate_normal = new FuzzyRuleAntecedent();
 if_t_high_and_heart_rate_normal->joinWithAND(t_high, heart_rate_normal);
 FuzzyRuleAntecedent *if_t_high_and_heart_rate_normal_and_spo2_low = new FuzzyRuleAntecedent();
  if_t_high_and_heart_rate_normal_and_spo2_low->joinWithAND(if_t_high_and_heart_rate_normal, if_spo2_low);
 FuzzyRule *fuzzyRule22 = new FuzzyRule(22, if_t_high_and_heart_rate_normal_and_spo2_low, then_Triage_Urgent);
 fuzzy->addFuzzyRule(fuzzyRule22);

 FuzzyRuleAntecedent *if_t_high_and_heart_rate_normal_and_spo2_normal = new FuzzyRuleAntecedent();
  if_t_high_and_heart_rate_normal_and_spo2_normal->joinWithAND(if_t_high_and_heart_rate_normal, if_spo2_normal);
 FuzzyRule *fuzzyRule23 = new FuzzyRule(23, if_t_high_and_heart_rate_normal_and_spo2_normal, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule23);

 FuzzyRuleAntecedent *if_t_high_and_heart_rate_normal_and_spo2_high = new FuzzyRuleAntecedent();
  if_t_high_and_heart_rate_normal_and_spo2_high->joinWithAND(if_t_high_and_heart_rate_normal, if_spo2_high);
 FuzzyRule *fuzzyRule24 = new FuzzyRule(24, if_t_high_and_heart_rate_normal_and_spo2_high, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule24); 

 FuzzyRuleAntecedent *if_t_high_and_heart_rate_high = new FuzzyRuleAntecedent();
 if_t_high_and_heart_rate_high->joinWithAND(t_high, heart_rate_high);
 FuzzyRuleAntecedent *if_t_high_and_heart_rate_high_and_spo2_low = new FuzzyRuleAntecedent();
  if_t_high_and_heart_rate_high_and_spo2_low->joinWithAND(if_t_high_and_heart_rate_high, if_spo2_low);
 FuzzyRule *fuzzyRule25 = new FuzzyRule(25, if_t_high_and_heart_rate_high_and_spo2_low, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule25); 
 
 FuzzyRuleAntecedent *if_t_high_and_heart_rate_high_and_spo2_normal = new FuzzyRuleAntecedent();
  if_t_high_and_heart_rate_high_and_spo2_normal->joinWithAND(if_t_high_and_heart_rate_high, if_spo2_normal);
 FuzzyRule *fuzzyRule26 = new FuzzyRule(27, if_t_high_and_heart_rate_high_and_spo2_normal, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule26); 

 FuzzyRuleAntecedent *if_t_high_and_heart_rate_high_and_spo2_high = new FuzzyRuleAntecedent();
  if_t_high_and_heart_rate_high_and_spo2_high->joinWithAND(if_t_high_and_heart_rate_high, if_spo2_high);
 FuzzyRule *fuzzyRule27 = new FuzzyRule(27, if_t_high_and_heart_rate_high_and_spo2_high, then_Triage_Standard);
 fuzzy->addFuzzyRule(fuzzyRule27); 

 
}
int maxArrayIndex(double array_var[]){
    int maxI = 0;
    
    for(int i = 1;i <3;i++) {
       Serial.print("**");
       Serial.print(array_var[i]);
       
        if(array_var[i] > array_var[maxI]){
           maxI = i;

      }
     }
     Serial.print("##");
     return maxI;
 }


