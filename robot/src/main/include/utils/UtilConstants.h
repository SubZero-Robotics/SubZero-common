#pragma once

namespace subzero {

namespace Logging {
enum class LogLevel { VERBOSE = 0, INFO, WARNING, ERROR, FATAL };
// Don't log at levels below this one
constexpr auto kMinLogLevel = LogLevel::VERBOSE;
} // namespace Logging

} // namespace subzero