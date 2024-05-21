#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>

#include <string>
#include <vector>
#include <functional>

#include "subzero/vision/LimelightHelpers.h"

struct DetectedCorner {
  double x;
  double y;

  DetectedCorner() {}

  explicit DetectedCorner(const std::vector<double>& coord) {
    x = coord[0];
    y = coord[1];
  }
};

struct DetectedCorners {
  DetectedCorner topLeft;
  DetectedCorner topRight;
  DetectedCorner bottomLeft;
  DetectedCorner bottomRight;

  DetectedCorners() {}

  explicit DetectedCorners(const std::vector<std::vector<double>>& corners) {
    if (corners.empty()) return;

    topLeft = DetectedCorner(corners[0]);
    topRight = DetectedCorner(corners[1]);
    bottomLeft = DetectedCorner(corners[2]);
    bottomRight = DetectedCorner(corners[3]);
  }

  explicit DetectedCorners(const std::vector<double>& rawCorners) {
    if (rawCorners.size() < 8) return;

    topLeft = DetectedCorner({rawCorners[0], rawCorners[1]});
    bottomLeft = DetectedCorner({rawCorners[6], rawCorners[7]});
    bottomRight = DetectedCorner({rawCorners[4], rawCorners[5]});
    topRight = DetectedCorner({rawCorners[2], rawCorners[3]});
  }
};

struct DetectedObject {
  uint8_t classId;
  std::string className;
  double confidence;
  // Positive-right, center-zero
  units::degree_t centerX;
  // Positive-down, center-zero
  units::degree_t centerY;
  double areaPercentage;
  DetectedCorners detectedCorners;

  DetectedObject() {}

  explicit DetectedObject(uint8_t id, double conf, units::degree_t cX,
                          units::degree_t cY, double area,
                          std::vector<std::vector<double>> corners)
      : classId{id},
        className{"unknown"},
        confidence{conf},
        centerX{cX},
        centerY{cY},
        areaPercentage{area},
        detectedCorners{corners} {}

  explicit DetectedObject(
      const LimelightHelpers::DetectionResultClass& detectionResult)
      : classId{static_cast<uint8_t>(detectionResult.m_classID)},
        className{detectionResult.m_className},
        confidence{detectionResult.m_confidence},
        centerX{detectionResult.m_TargetXDegreesCrosshairAdjusted},
        centerY{detectionResult.m_TargetYDegreesCrosshairAdjusted},
        areaPercentage{detectionResult.m_TargetAreaNormalized},
        detectedCorners{detectionResult.m_TargetCorners} {}

  void withRawCorners(const std::vector<double>& rawCorners) {
    detectedCorners = DetectedCorners(rawCorners);
  }
};

struct TrackedTarget {
  DetectedObject object;
  frc::Pose2d currentPose;
  bool valid;
};

class TargetTracker {
 public:
  struct TargetTrackerConfig {
    units::degree_t cameraAngle;
    units::inch_t cameraLensHeight;
    double confidenceThreshold;
    std::string limelightName;
    units::inch_t gamepieceWidth;
    units::dimensionless::scalar_t focalLength;
    frc::Pose2d simGamepiecePose;
    units::degree_t gamepieceRotation;
    /// @brief Ranges from 0 to 1; Multiplies trig-based distances and then
    /// applies the inverse to width-based estimate
    double trigDistancePercentage;
    double areaPercentageThreshold;

    uint8_t maxTrackedItems;
    frc::Pose2d invalidTrackedPose;
  };

  TargetTracker(TargetTrackerConfig config,
                std::function<frc::Pose2d()> poseGetter,
                std::function<frc::Field2d*()> fieldGetter);
  std::vector<DetectedObject> GetTargets();
  void UpdateTrackedTargets(const std::vector<DetectedObject>& objects);
  std::optional<DetectedObject> GetBestTarget(std::vector<DetectedObject>&);
  bool HasTargetLock(std::vector<DetectedObject>&);
  std::optional<frc::Pose2d> GetTargetPose(const DetectedObject&);
  std::optional<frc::Pose2d> GetBestTargetPose(std::vector<DetectedObject>&);
  units::inch_t GetDistanceToTarget(const DetectedObject&);

 private:
  /**
   * Sort targets by ASC distance to camera
   */
  void SortTargetsByProximity(std::vector<DetectedObject>& objects);
  void PublishTrackedTarget(const TrackedTarget& target, int index);

  TargetTrackerConfig m_config;
  std::function<frc::Pose2d()> m_poseGetter;
  std::function<frc::Field2d*()> m_fieldGetter;
  std::vector<TrackedTarget> m_trackedTargets;
};