#pragma once

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>
#include <wpi/MemoryBuffer.h>

#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "subzero/frc2/command/EmptyCommand.h"
#include "subzero/logging/ConsoleLogger.h"

namespace subzero
{

  /**
   * @brief Safely get an auto command from PPLib without the risk of crashing
   * from a missing auto file
   *
   * @tparam T Key for identifying the auto, typically an enum
   * @remark Will return an empty command if the auto does not exist
   */
  template <typename T>
  class AutoFactory
  {
  public:
    explicit AutoFactory(const std::map<T, std::string> &autos)
        : m_autos{autos} {}

  private:
    const std::map<T, std::string> &m_autos;

    bool AutoFileExists(const std::string fileName);

    frc2::CommandPtr PathPlannerPathFromName(const std::string autoName);

  public:
    /**
     * @brief Get the auto command specified by the key
     *
     * @param type
     * @return frc2::CommandPtr The schedulable auto command
     */
    frc2::CommandPtr GetAuto(T type);
  };
} // namespace subzero