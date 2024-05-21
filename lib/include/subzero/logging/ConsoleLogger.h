#ifndef CONSOLE_LOGGER_H
#define CONSOLE_LOGGER_H

#include <iostream>
#include <string>

#include "subzero/logging/ILogger.h"

namespace subzero {
/**
 * @brief Outputs formatted strings to stdout
 *
 * @remark Singleton class
 */
class ConsoleLogger : ILogger {
 public:
  static ConsoleLogger &getInstance() {
    static ConsoleLogger instance;

    return instance;
  }

  void logVerbose(std::string key, const std::string format, ...) override;
  void logInfo(std::string key, const std::string format, ...) override;
  void logWarning(std::string key, const std::string format, ...) override;
  void logError(std::string key, const std::string format, ...) override;
  void logFatal(std::string key, const std::string format, ...) override;

  void logInfo(std::string key, int val) override;
  void logVerbose(std::string key, int val) override;
  void logWarning(std::string key, int val) override;
  void logError(std::string key, int val) override;
  void logFatal(std::string key, int val) override;

  void logInfo(std::string key, double val) override;
  void logVerbose(std::string key, double val) override;
  void logWarning(std::string key, double val) override;
  void logError(std::string key, double val) override;
  void logFatal(std::string key, double val) override;

  void logInfo(std::string key, bool val) override;
  void logVerbose(std::string key, bool val) override;
  void logWarning(std::string key, bool val) override;
  void logError(std::string key, bool val) override;
  void logFatal(std::string key, bool val) override;

  void logInfo(std::string key, frc::Pose2d &val) override;
  void logVerbose(std::string key, frc::Pose2d &val) override;
  void logWarning(std::string key, frc::Pose2d &val) override;
  void logError(std::string key, frc::Pose2d &val) override;
  void logFatal(std::string key, frc::Pose2d &val) override;

  void logInfo(std::string key, wpi::Sendable *val) override;
  void logVerbose(std::string key, wpi::Sendable *val) override;
  void logWarning(std::string key, wpi::Sendable *val) override;
  void logError(std::string key, wpi::Sendable *val) override;
  void logFatal(std::string key, wpi::Sendable *val) override;

 private:
  ConsoleLogger();

  void log(Logging::Level level, std::string key, std::string fmt, va_list ap) {
    if (!shouldLog(level)) return;

    std::string val = formatString(fmt, ap);
    std::cout << levelToString(level) << " - " << key << ": " << val
              << std::endl;
  }
};
}  // namespace subzero

#define ConsoleWriter subzero::ConsoleLogger::getInstance()
/**
 * @brief Shortcut to log at the Logging::Level::Info level from a command
 * composition
 *
 */
#define ConsoleInfo(key, fmt, ...)                \
  frc2::InstantCommand([] {                       \
    ConsoleWriter.logInfo(key, fmt, __VA_ARGS__); \
  }).ToPtr()
/**
 * @brief Shortcut to log at the Logging::Level::Verbose level from a command
 * composition
 *
 */
#define ConsoleVerbose(key, fmt, ...)                \
  frc2::InstantCommand([] {                          \
    ConsoleWriter.logVerbose(key, fmt, __VA_ARGS__); \
  }).ToPtr()
#endif