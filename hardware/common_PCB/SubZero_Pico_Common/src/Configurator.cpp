#include "Configurator.h"

void Configurator::configSetup() {
  Configuration config;

  while (Serial.available()) Serial.read();
    Serial.println(
      "Enter your selections with line ending set to NO LINE ENDING only");
  Serial.print("Team number: ");
  while (!Serial.available());
  config.teamNumber = Serial.parseInt();
  Serial.println(config.teamNumber);
  Serial.print("Other team number 1: ");
  while (!Serial.available());
  config.initialTeams[0] = Serial.parseInt();
  Serial.println(config.initialTeams[0]);
  Serial.print("Other team number 2: ");
  while (!Serial.available());
  config.initialTeams[1] = Serial.parseInt();
  Serial.println(config.initialTeams[1]);
  Serial.print("I2C address for comms with RIO: ");
  while (!Serial.available());
  config.i2c0Addr = Serial.parseInt();
  Serial.println(config.i2c0Addr);
  Serial.print("LED0 count: ");
  while (!Serial.available());
  config.led0.count = Serial.parseInt();
  Serial.println(config.led0.count);
  Serial.print("LED0 brightness (1 - 100): ");
  while (!Serial.available());
  config.led0.brightness = Serial.parseInt();
  Serial.println(config.led0.brightness);
  Serial.print("LED1 count: ");
  while (!Serial.available());
  config.led1.count = Serial.parseInt();
  Serial.println(config.led1.count);
  Serial.print("LED1 brightness (1 - 100): ");
  while (!Serial.available());
  config.led1.brightness = Serial.parseInt();
  Serial.println(config.led1.brightness);

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