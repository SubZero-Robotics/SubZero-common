#pragma once

namespace Logging {
enum class Level { VERBOSE = 0, INFO, WARNING, ERROR, FATAL };
// Don't log at levels below this one
constexpr auto kMinLogLevel = Level::VERBOSE;
} // namespace Logging

namespace DetectionParser {
enum class ObjectClasses { Cone = 0, Cube = 1, Merge = 2 };
}