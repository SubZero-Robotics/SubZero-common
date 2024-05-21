#pragma once

#include <fmt/format.h>
#include <frc/geometry/Pose2d.h>
#include <wpi/json.h>
#include <wpi/sendable/Sendable.h>

#include <cstdarg>
#include <string>

#include "subzero/utils/UtilConstants.h"

/**
 * @brief Logger interface
 * 
 */
class ILogger {
 public:
  // string
  virtual void logVerbose(std::string key, const std::string format, ...) = 0;
  virtual void logInfo(std::string key, const std::string format, ...) = 0;
  virtual void logWarning(std::string key, const std::string format, ...) = 0;
  virtual void logError(std::string key, const std::string format, ...) = 0;
  virtual void logFatal(std::string key, const std::string format, ...) = 0;

  // int
  virtual void logVerbose(std::string key, int val) = 0;
  virtual void logInfo(std::string key, int val) = 0;
  virtual void logWarning(std::string key, int val) = 0;
  virtual void logError(std::string key, int val) = 0;
  virtual void logFatal(std::string key, int val) = 0;

  // double
  virtual void logVerbose(std::string key, double val) = 0;
  virtual void logInfo(std::string key, double val) = 0;
  virtual void logWarning(std::string key, double val) = 0;
  virtual void logError(std::string key, double val) = 0;
  virtual void logFatal(std::string key, double val) = 0;

  // bool
  virtual void logInfo(std::string key, bool val) = 0;
  virtual void logVerbose(std::string key, bool val) = 0;
  virtual void logWarning(std::string key, bool val) = 0;
  virtual void logError(std::string key, bool val) = 0;
  virtual void logFatal(std::string key, bool val) = 0;

  // Pose2d
  virtual void logInfo(std::string key, frc::Pose2d& val) = 0;
  virtual void logVerbose(std::string key, frc::Pose2d& val) = 0;
  virtual void logWarning(std::string key, frc::Pose2d& val) = 0;
  virtual void logError(std::string key, frc::Pose2d& val) = 0;
  virtual void logFatal(std::string key, frc::Pose2d& val) = 0;

  // Sendable
  virtual void logInfo(std::string key, wpi::Sendable* val) = 0;
  virtual void logVerbose(std::string key, wpi::Sendable* val) = 0;
  virtual void logWarning(std::string key, wpi::Sendable* val) = 0;
  virtual void logError(std::string key, wpi::Sendable* val) = 0;
  virtual void logFatal(std::string key, wpi::Sendable* val) = 0;

 protected:
  std::string levelToString(Logging::Level level) const {
    using namespace Logging;

    switch (level) {
      case Level::VERBOSE:
        return "VERBOSE";
      case Level::INFO:
        return "INFO";
      case Level::WARNING:
        return "WARNING";
      case Level::ERROR:
        return "ERROR";
      case Level::FATAL:
        return "FATAL";
      default:
        return "INVALID LOG LEVEL";
    }
  }

  std::string formatString(const std::string format, va_list ap) {
    char buf[1000];
    vsnprintf(buf, sizeof(buf), format.c_str(), ap);

    return std::string(buf);
  }

  std::string poseToString(const frc::Pose2d& pose) {
    wpi::json json;

    frc::to_json(json, pose);
    return json.dump();
  }

  inline bool shouldLog(Logging::Level level) {
    return static_cast<int>(level) >= static_cast<int>(Logging::kMinLogLevel);
  }
};