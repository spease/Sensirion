/*
 * Query a SHT10 temperature and humidity sensor
 *
 * A simple example that queries the sensor every 3 seconds
 * and communicates the result over a serial connection.
 * Error handling is omitted in this example.
 */

#include "Sensirion.h"

const uint8_t dataPin  =  2;
const uint8_t clockPin =  3;

float temperature;
float humidity;
float dewpoint;

Sensirion tempSensor = Sensirion(dataPin, clockPin);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  tempSensor.measure(&temperature, &humidity, &dewpoint);

  Serial.print("Temperature: ");
  serialPrintFloat(temperature);
  Serial.print(" C, Humidity: ");
  serialPrintFloat(humidity);
  Serial.print(" %, Dewpoint: ");
  serialPrintFloat(dewpoint);
  Serial.println(" C");
  
  delay(3000);  
}

void serialPrintFloat(float f){
  Serial.print((int)f);
  Serial.print(".");
  int decplace = (f - (int)f) * 100;
  Serial.print(abs(decplace));
}
