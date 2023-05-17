#include "ConnectorX.h"

using namespace ConnectorX;

ConnectorX::ConnectorX(uint8_t slaveAddress, frc::I2C::Port port)
    : _i2c(std::make_unique<frc::I2C>(port, slaveAddress)),
      _slaveAddress(slaveAddress) {
  setOff(LedPort::P0);
  setOff(LedPort::P1);
}

bool ConnectorX::initialize() { return !_i2c->AddressOnly(); }

void ConnectorX::configureDigitalPin(DigitalPort port, PinMode mode) {
  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::DigitalSetup;
  cmd.commandData.commandDigitalSetup = {.port = (uint8_t)port,
                                         .mode = (uint8_t)mode};

  sendCommand(cmd);
}

void ConnectorX::writeDigitalPin(DigitalPort port, bool value) {
  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::DigitalWrite;
  cmd.commandData.commandDigitalWrite = {.port = (uint8_t)port,
                                         .value = (uint8_t)value};

  sendCommand(cmd);
}

bool ConnectorX::readDigitalPin(DigitalPort port) {
  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::DigitalRead;
  cmd.commandData.commandDigitalRead = {.port = (uint8_t)port};

  Commands::Response res = sendCommand(cmd, true);
  return res.responseData.responseDigitalRead.value;
}

uint16_t ConnectorX::readAnalogPin(AnalogPort port) {
  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::ReadAnalog;
  cmd.commandData.commandReadAnalog = {.port = (uint8_t)port};

  Commands::Response res = sendCommand(cmd, true);
  return res.responseData.responseReadAnalog.value;
}

void ConnectorX::setLedPort(LedPort port) {
  if (port != _currentLedPort) {
    _currentLedPort = port;
    Commands::Command cmd;
    cmd.commandType = Commands::CommandType::SetLedPort;
    cmd.commandData.commandSetLedPort.port = (uint8_t)port;
    sendCommand(cmd);
  }
}

void ConnectorX::setOn(LedPort port) {
  setLedPort(port);

  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::On;
  cmd.commandData.commandOn = {};
  sendCommand(cmd);
}

void ConnectorX::setOff(LedPort port) {
  setLedPort(port);

  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::Off;
  cmd.commandData.commandOff = {};
  sendCommand(cmd);
}

void ConnectorX::setPattern(LedPort port, PatternType pattern,
                            bool oneShot = false, int16_t delay = -1) {
  setLedPort(port);

  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::Pattern;
  cmd.commandData.commandPattern.pattern = (uint8_t)pattern;
  cmd.commandData.commandPattern.oneShot = (uint8_t)oneShot;
  cmd.commandData.commandPattern.delay = delay;
  sendCommand(cmd);
}

void ConnectorX::setColor(LedPort port, uint8_t red, uint8_t green,
                          uint8_t blue) {
  setLedPort(port);

  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::ChangeColor;
  cmd.commandData.commandColor.red = red;
  cmd.commandData.commandColor.green = green;
  cmd.commandData.commandColor.blue = blue;
  sendCommand(cmd);
}

bool ConnectorX::setColor(LedPort port, uint32_t color) {
  setLedPort(port);

  setColor(port, (color >> 16) & 255, (color >> 8) & 255, color & 255);
}

bool ConnectorX::getPatternDone(LedPort port) {
  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::ReadPatternDone;
  cmd.commandData.commandReadPatternDone = {};

  Commands::Response res = sendCommand(cmd, true);
  return res.responseData.responsePatternDone.done;
}

void ConnectorX::setConfig(Configuration config) {
  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::SetConfig;
  cmd.commandData.commandSetConfig.config = config;

  sendCommand(cmd);
}

Configuration ConnectorX::readConfig() {
  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::ReadConfig;
  cmd.commandData.commandReadConfig = {};

  Commands::Response res = sendCommand(cmd, true);
  return res.responseData.responseReadConfiguration.config;
}

void ConnectorX::sendRadioMessage(Message message) {
  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::RadioSend;
  cmd.commandData.commandRadioSend.msg = message;

  sendCommand(cmd);
}

Message ConnectorX::getLatestRadioMessage() {
  Commands::Command cmd;
  cmd.commandType = Commands::CommandType::RadioGetLatestReceived;
  cmd.commandData.commandRadioGetLatestReceived = {};

  Commands::Response res = sendCommand(cmd, true);
  return res.responseData.responseRadioLastReceived.msg;
}

Commands::Response ConnectorX::sendCommand(Commands::Command command,
                                           bool expectResponse) {
  using namespace Commands;

  _lastCommand = command.commandType;
  uint8_t sendLen;
  uint8_t recSize = 0;
  Response response;
  response.commandType = command.commandType;

  uint8_t sendBuf[sizeof(CommandData) + 1];
  sendBuf[0] = (uint8_t)command.commandType;

  switch (command.commandType) {
  case CommandType::On:
  case CommandType::Off:
    sendLen = 0;
    break;
  case CommandType::ReadConfig:
    sendLend = 0;
    recSize = sizeof(ResponseReadConfiguration);
    break;
  case CommandType::ReadPatternDone:
    sendLen = 0;
    recSize = sizeof(ResponsePatternDone);
    break;
  case CommandType::RadioGetLatestReceived:
    sendLen = 0;
    recSize = sizeof(ResponseRadioLastReceived);
    break;
  case CommandType::SetLedPort:
    sendLen = 1;
    break;
  case CommandType::ReadAnalog:
    sendLen = 1;
    recSize = sizeof(ResponseReadAnalog);
    break;
  case CommandType::DigitalRead:
    sendLen = 1;
    recSize = sizeof(ResponseDigitalRead);
    break;
  case CommandType::DigitalWrite:
    sendLen = sizeof(CommandDigitalWrite);
    break;
  case CommandType::DigitalSetup:
    sendLen = sizeof(CommandDigitalSetup);
    break;
  case CommandType::Pattern:
    sendLen = sizeof(CommandPattern);
    break;
  case CommandType::Color:
    sendLen = sizeof(CommandColor);
    break;
  case CommandType::SetConfig:
    sendLen = sizeof(CommandSetConfig);
    break;
  case CommandType::RadioSend:
    sendLen = sizeof(CommandRadioSend);
    break;
  }

  memcpy(sendBuf + 1, &command.commandData, sendLen);

  if (recSize == 0) {
    _i2c->WriteBulk(sendBuf, sendLen + 1);
    return response;
  }

  _i2c->Transaction(sendBuf, sendLen + 1, (uint8_t *)&response.responseData,
                    recSize);
  return responseData;
}