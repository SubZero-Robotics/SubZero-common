#include "Configurator.h"

void Configurator::configSetup() {
  Configuration config;

  Serial.println(
      "Enter your selections with line ending set to LINE FEED only");
  Serial.print("Team number: ");
  config.teamNumber = Serial.parseInt();
  Serial.println();
  Serial.print("Other team number 1: ");
  config.initialTeams[0] = Serial.parseInt();
  Serial.print("\nOther team number 2: ");
  config.initialTeams[1] = Serial.parseInt();
  Serial.println();
  Serial.print("I2C address for comms with RIO: ");
  config.i2c0Addr = Serial.parseInt();
  Serial.println();
  Serial.print("LED0 count: ");
  config.led0.count = Serial.parseInt();
  Serial.println();
  Serial.print("LED0 brightness (1 - 100): ");
  config.led0.brightness = Serial.parseInt();
  Serial.println();
  Serial.print("LED1 count: ");
  config.led1.count = Serial.parseInt();
  Serial.println();
  Serial.print("LED1 brightness (1 - 100): ");
  config.led1.brightness = Serial.parseInt();
  Serial.println();

  Serial.println("Storing config:");
  Serial.println(toString(config).c_str());
  storeConfig(config);
  Serial.println("Done");
}

std::string Configurator::toString(Configuration config) {
  std::string str = "Slave address = " + std::to_string(config.i2c0Addr) + "\n";
  str += "LED 0:\n";
  str += "\tCount: " + std::to_string(config.led0.count) + "\n";
  str += "\tBrightness: " + std::to_string(config.led0.brightness) + "\n";
  str += "LED 1:\n";
  str += "\tCount: " + std::to_string(config.led1.count) + "\n";
  str += "\tBrightness: " + std::to_string(config.led1.brightness) + "\n";

  return str;
}