/*
 * Example code for SHT1x or SHT7x sensors demonstrating blocking calls
 * for temperature and humidity measurement in the setup routine and
 * non-blocking calls in the main loop.  The pin 13 LED is flashed as a
 * background task while temperature and humidity measurements are made.
 * In addition, the sensor may be placed in low resolution mode by
 * uncommenting the status register write call in setup().
 */

#include <Sensirion.h>

const uint8_t dataPin =  2;            // SHT serial data
const uint8_t sclkPin =  3;            // SHT serial clock
const uint8_t ledPin  = 13;            // Arduino built-in LED
const uint32_t TRHSTEP   = 3000UL;     // Sensor query period
const uint32_t BLINKSTEP =  250UL;     // LED blink period

Sensirion sht = Sensirion(dataPin, sclkPin);

uint16_t rawData;
float temperature;
float humidity;
float dewpoint;

byte shtState = 0;
byte ledState = 0;

unsigned long curMillis;               // Time interval tracking
unsigned long trhMillis = 0;
unsigned long blinkMillis = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  delay(15);                           // Wait at least 11 ms before first cmd
//  sht.writeSR(LOW_RES);                // Set sensor to low resolution
  sht.measTemp(&rawData);              // Maps to: sht.meas(TEMP, &rawData, BLOCK)
  temperature = sht.calcTemp(rawData);
  sht.measHumi(&rawData);              // Maps to: sht.meas(HUMI, &rawData, BLOCK)
  humidity = sht.calcHumi(rawData, temperature);
  dewpoint = sht.calcDewpoint(humidity, temperature);
  logData();
}

void loop()
{
  curMillis = millis();

  if (curMillis - blinkMillis >= BLINKSTEP) {    // Time to toggle the LED state?
    ledState ^= 1;
    digitalWrite(ledPin, ledState);
    blinkMillis = curMillis;
  }

  switch (shtState) {
  case 0:
    if (curMillis - trhMillis >= TRHSTEP) {      // Start new temp/humi measurement?
      sht.meas(TEMP, &rawData, NONBLOCK);
      shtState++;
      trhMillis = curMillis;
    }
    break;
  case 1:
    if (sht.measRdy()) {                         // Process temperature measurement?
      temperature = sht.calcTemp(rawData);
      sht.meas(HUMI, &rawData, NONBLOCK);
      shtState++;
    }
    break;
  case 2:
    if (sht.measRdy()) {                         // Process humidity measurement?
      humidity = sht.calcHumi(rawData, temperature);
      dewpoint = sht.calcDewpoint(humidity, temperature);
      shtState = 0;
      logData();
    }
    break;
  default:
    Serial.println("How did I get here?");
    break;
  }
}

void logData() {
  Serial.print("Temperature = ");   Serial.print(temperature);
  Serial.print(" C, Humidity = ");  Serial.print(humidity);
  Serial.print(" %, Dewpoint = ");  Serial.print(dewpoint);
  Serial.println(" C");
}
