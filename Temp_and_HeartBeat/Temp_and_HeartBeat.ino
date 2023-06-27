//////////////////////////////////////////////////////////////////////////////////////////
//
//    Arduino example for the MAX30205 body temperature sensor breakout board
//
//    Author: Ashwin Whitchurch
//    Copyright (c) 2020 ProtoCentral
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/protocentral/ProtoCentral_MAX30205
/////////////////////////////////////////////////////////////////////////////////////////

/*

This program Print temperature on terminal

Hardware Connections (Breakoutboard to Arduino):
Vin  - 5V (3.3V is allowed)
GND - GND
SDA - A4 (or SDA)
SCL - A5 (or SCL)

*/

#include <Wire.h>
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

void setup()
{

  Serial.begin(115200);
   Wire.begin();
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

  // scan for temperature in every 30 sec untill a sensor is found. Scan for both addresses 0x48 and 0x49
  /*while (!tempSensor.scanAvailableSensors())
  {
    Serial.println("Couldn't find the temperature sensor, please connect the sensor.");
    delay(30000);
  }*/

  // set continuos mode, active mode
}

void loop()
{

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
}
