#pragma once

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
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

    bool AutoFileExists(const std::string fileName)
    {
      const std::string filePath = frc::filesystem::GetDeployDirectory() +
                                    "/pathplanner/autos/" + fileName + ".auto";

      // Use wpi::MemoryBuffer::GetFile and handle the wpi::expected result
      auto fileBufferResult = wpi::MemoryBuffer::GetFile(filePath);

      if (!fileBufferResult)
      {
        // fileBufferResult is an error; the file does not exist
        return false;
      }

      // fileBufferResult contains a valid std::unique_ptr<wpi::MemoryBuffer>
      return true;
    }

    frc2::CommandPtr PathPlannerPathFromName(const std::string autoName)
    {
      if (!AutoFileExists(autoName))
      {
        ConsoleWriter.logError("Auto Factory",
                               "AUTO '%s' DOES NOT EXIST HELP US EVAN",
                               autoName.c_str());
        return EmptyCommand().ToPtr();
      }
      return pathplanner::PathPlannerAuto(autoName).ToPtr();
    }

  public:
    /**
     * @brief Get the auto command specified by the key
     *
     * @param type
     * @return frc2::CommandPtr The schedulable auto command
     */
    frc2::CommandPtr GetAuto(T type)
    {
      if (!m_autos.contains(type))
      {
        ConsoleWriter.logWarning("Auto Factory",
                                 "Auto type %d does not exist, defaulting to empty "
                                 "auto",
                                 static_cast<int>(type));
        return EmptyCommand().ToPtr();
      }

      return PathPlannerPathFromName(m_autos.at(type));
    }
  };

} // namespace subzero
