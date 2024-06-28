/*
 * Project Distributed Air Quality Monitors
 * Author: SubZero 5690
 * Date: June 22, 2024
 * Based off Brandon Satrom's example https://docs.particle.io/quickstart/aqmk-project/
 */

#include "Particle.h"

SYSTEM_MODE(AUTOMATIC);

SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);
Ledger ledger;

#include <math.h>
#include "Air_Quality_Sensor.h"
#include "SeeedOLED.h"

#define AQS_PIN A2
#define DUST_SENSOR_PIN D4
#define SENSOR_READING_INTERVAL 30000

AirQualitySensor aqSensor(AQS_PIN);

unsigned long lastInterval;
unsigned long lowpulseoccupancy = 0;
unsigned long last_lpo = 0;
unsigned long duration;

float ratio = 0;
float concentration = 0;

void getDustSensorReadings();
String getAirQuality();
void createEventPayload(String airQuality);
void updateDisplay(String airQuality);

void setup()
{
  ledger = Particle.ledger("air-quality");

  // Configure the dust sensor pin as an input
  pinMode(DUST_SENSOR_PIN, INPUT);

  if (aqSensor.init())
  {
    Serial.println("Air Quality Sensor ready.");
  }
  else
  {
    Serial.println("Air Quality Sensor ERROR!");
  }

  Wire.begin();
  SeeedOled.init();

  SeeedOled.clearDisplay();
  SeeedOled.setInverseDisplay();
  SeeedOled.setPageMode();

  SeeedOled.sendCommand(0xA8);
	SeeedOled.sendCommand(0x3F);
  SeeedOled.sendCommand(0xA0);
  SeeedOled.sendCommand(0xC9);
  SeeedOled.sendCommand(0xA1);

  SeeedOled.setTextXY(2, 0);
  SeeedOled.putString("Particle");
  SeeedOled.setTextXY(3, 0);
  SeeedOled.putString("Air Quality");
  SeeedOled.setTextXY(4, 0);
  SeeedOled.putString("Monitor");

  lastInterval = millis();
}

void loop()
{
  duration = pulseIn(DUST_SENSOR_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;

  if ((millis() - lastInterval) > SENSOR_READING_INTERVAL)
  {
    String quality = getAirQuality();
    Serial.printlnf("Air Quality: %s", quality.c_str());

    getDustSensorReadings();

    updateDisplay(quality);

    createEventPayload(quality);

    lowpulseoccupancy = 0;
    lastInterval = millis();
  }
}

String getAirQuality()
{
  int quality = aqSensor.slope();
  String qual = "None";

  if (quality == AirQualitySensor::FORCE_SIGNAL)
  {
    qual = "Danger";
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION)
  {
    qual = "High Pollution";
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION)
  {
    qual = "Low Pollution";
  }
  else if (quality == AirQualitySensor::FRESH_AIR)
  {
    qual = "Fresh Air";
  }

  return qual;
}

void getDustSensorReadings()
{
  // This particular dust sensor returns 0s often, so let's filter them out by making sure we only
  // capture and use non-zero LPO values for our calculations once we get a good reading.
  if (lowpulseoccupancy == 0)
  {
    lowpulseoccupancy = last_lpo;
  }
  else
  {
    last_lpo = lowpulseoccupancy;
  }

  ratio = lowpulseoccupancy / (SENSOR_READING_INTERVAL * 10.0);                   // Integer percentage 0=>100
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve

  Serial.printlnf("LPO: %lu", lowpulseoccupancy);
  Serial.printlnf("Ratio: %f%%", ratio);
  Serial.printlnf("Concentration: %f pcs/L", concentration);
}

void createEventPayload(String airQuality)
{
  Variant data;
  data.set("air_quality", airQuality);

  if (lowpulseoccupancy > 0)
  {
    Variant dustData;
    dustData.set("lpo", lowpulseoccupancy);
    dustData.set("ratio", ratio);
    dustData.set("concentration", concentration);
    data.set("dust", dustData);
  }

  ledger.set(data, Ledger::SetMode::MERGE);
}

void updateDisplay(String airQuality)
{
  SeeedOled.clearDisplay();

  SeeedOled.setTextXY(0, 3);
  SeeedOled.putString(airQuality);

  if (concentration > 1)
  {
    SeeedOled.setTextXY(5, 0);
    SeeedOled.putString("Dust: ");
    SeeedOled.putNumber(concentration); // Will cast our float to an int to make it more compact
    SeeedOled.putString(" pcs/L");
  }
}