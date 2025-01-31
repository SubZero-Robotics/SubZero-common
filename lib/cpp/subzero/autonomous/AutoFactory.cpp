#pragma once

#include "subzero/autonomous/AutoFactory.h"

using namespace subzero;

template <typename T>
bool AutoFactory<T>::AutoFileExists(const std::string fileName) {
  const std::string filePath = frc::filesystem::GetDeployDirectory() +
                               "/pathplanner/autos/" + fileName + ".auto";

  std::error_code error_code;
  std::unique_ptr<wpi::MemoryBuffer> fileBuffer =
      wpi::MemoryBuffer::GetFile(filePath);

  if (fileBuffer == nullptr || error_code) {
    return false;
  }

  return true;
}

template <typename T>
frc2::CommandPtr
AutoFactory<T>::PathPlannerPathFromName(const std::string autoName) {
  if (!AutoFileExists(autoName)) {
    ConsoleWriter.logError("Auto Factory",
                           "AUTO '%s' DOES NOT EXIST HELP US EVAN",
                           autoName.c_str());
    return EmptyCommand().ToPtr();
  }
  return pathplanner::PathPlannerAuto(autoName).ToPtr();
}

template <typename T> frc2::CommandPtr AutoFactory<T>::GetAuto(T type) {
  if (!m_autos.contains(type)) {
    ConsoleWriter.logWarning("Auto Factory",
                             "Auto type %d does not exist, defaulting to empty "
                             "auto",
                             static_cast<int>(type));
    return EmptyCommand().ToPtr();
  }

  return PathPlannerPathFromName(m_autos.at(type));
}