#pragma once

#include <frc/geometry/Pose2d.h>

#include "Logger.h"

namespace LimelightParser {
    frc::Pose2d AssistSubsystem::GetPosition() {
        auto rawbot = nt::NetworkTableInstance::GetDefault()
                        .GetTable("limelight")
                        ->GetNumberArray("botpose", std::vector<double>(6));
        auto x = (units::meter_t)rawbot[0];
        auto y = (units::meter_t)rawbot[1];
        auto yaw = rawbot[5];
        auto pose = frc::Pose2d{x, y, frc::Rotation2d(units::degree_t(yaw))};
        Logging::logToSmartDashboard("AssistSubsystem",
                                    "X: " + std::to_string(x.value()) + "\n" +
                                        "Y: " + std::to_string(y.value()) + "\n" +
                                        "YAW: " + std::to_string(yaw) + "\n",
                                    Logging::Level::INFO);
        return pose;
    }
}