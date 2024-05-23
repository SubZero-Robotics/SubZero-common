#pragma once

namespace subzero {
namespace Logging {
/**
 * @brief Levels of logging
 *
 */
enum class Level { VERBOSE = 0, INFO, WARNING, ERROR, FATAL };
/**
 * @brief Will not log messages that fall below this level
 *
 */
constexpr auto kMinLogLevel = Level::VERBOSE;
} // namespace Logging

namespace DetectionParser {
enum class ObjectClasses { Cone = 0, Cube = 1, Merge = 2 };
}
} // namespace subzero