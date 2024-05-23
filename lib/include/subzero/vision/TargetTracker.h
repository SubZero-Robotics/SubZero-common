#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <pathplanner/lib/commands/FollowPathHolonomic.h>

#include <functional>
#include <string>
#include <vector>

#include "subzero/vision/LimelightHelpers.h"

namespace subzero {

struct DetectedCorner {
  double x;
  double y;

  DetectedCorner() {}

  explicit DetectedCorner(const std::vector<double> &coord) {
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

  explicit DetectedCorners(const std::vector<std::vector<double>> &corners) {
    if (corners.empty())
      return;

    topLeft = DetectedCorner(corners[0]);
    topRight = DetectedCorner(corners[1]);
    bottomLeft = DetectedCorner(corners[2]);
    bottomRight = DetectedCorner(corners[3]);
  }

  explicit DetectedCorners(const std::vector<double> &rawCorners) {
    if (rawCorners.size() < 8)
      return;

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
      : classId{id}, className{"unknown"}, confidence{conf}, centerX{cX},
        centerY{cY}, areaPercentage{area}, detectedCorners{corners} {}

  explicit DetectedObject(
      const LimelightHelpers::DetectionResultClass &detectionResult)
      : classId{static_cast<uint8_t>(detectionResult.m_classID)},
        className{detectionResult.m_className},
        confidence{detectionResult.m_confidence},
        centerX{detectionResult.m_TargetXDegreesCrosshairAdjusted},
        centerY{detectionResult.m_TargetYDegreesCrosshairAdjusted},
        areaPercentage{detectionResult.m_TargetAreaNormalized},
        detectedCorners{detectionResult.m_TargetCorners} {}

  void withRawCorners(const std::vector<double> &rawCorners) {
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
    /**
     * @brief Camera angle relative to the floor; positive = up
     *
     */
    units::degree_t cameraAngle;
    /**
     * @brief Camera height relative to the floor
     *
     */
    units::inch_t cameraLensHeight;
    /**
     * @brief Targets must have a greater value in order to be used
     *
     */
    double confidenceThreshold;
    /**
     * @brief Name of the limelight in shuffleboard
     *
     */
    std::string limelightName;
    /**
     * @brief Width of the gamepiece
     *
     */
    units::inch_t gamepieceWidth;
    /**
     * @brief Calculated by doing: (known distance / known width in pixels) *
     * gamepiece width
     *
     */
    units::dimensionless::scalar_t focalLength;
    /**
     * @brief Pose of the mock gamepiece in sim
     *
     */
    frc::Pose2d simGamepiecePose;
    /**
     * @brief Rotation of the gamepiece relative to the field; helpful for
     * automatic intaking
     *
     */
    units::degree_t gamepieceRotation;
    /**
     * @brief Ranges from 0 to 1; Multiplies trig-based distances and then
     * applies the inverse to width-based estimate
     *
     */
    double trigDistancePercentage;
    /**
     * @brief Ignore any targets below this percentage to exclude spurious
     * detections
     *
     */
    double areaPercentageThreshold;
    /**
     * @brief Max number of items that will be pushed to SmartDashboard as poses
     *
     */
    uint8_t maxTrackedItems;
    /**
     * @brief Move invalid objects to this pose
     *
     */
    frc::Pose2d invalidTrackedPose;
  };

  TargetTracker(TargetTrackerConfig config,
                std::function<frc::Pose2d()> poseGetter,
                std::function<frc::Field2d *()> fieldGetter);
  /**
   * @brief Get a list of all found, valid targets
   *
   * @return std::vector<DetectedObject>
   */
  std::vector<DetectedObject> GetTargets();

  /**
   * @brief Push targets to SmartDashboard
   *
   * @param objects
   */
  void UpdateTrackedTargets(const std::vector<DetectedObject> &objects);

  /**
   * @brief Get the best target for tracking/intaking
   *
   * @return std::optional<DetectedObject>
   */
  std::optional<DetectedObject> GetBestTarget(std::vector<DetectedObject> &);

  /**
   * @brief Check if a target has been acquired
   *
   * @return true
   * @return false
   */
  bool HasTargetLock(std::vector<DetectedObject> &);

  /**
   * @brief Get the pose of the given object
   *
   * @return std::optional<frc::Pose2d>
   */
  std::optional<frc::Pose2d> GetTargetPose(const DetectedObject &);

  /**
   * @brief From a list of detected objects, get the pose of the best one
   *
   * @return std::optional<frc::Pose2d>
   */
  std::optional<frc::Pose2d> GetBestTargetPose(std::vector<DetectedObject> &);

  /**
   * @brief Get the estimated distance to the target
   *
   * @return units::inch_t
   */
  units::inch_t GetDistanceToTarget(const DetectedObject &);

private:
  /**
   * Sort targets by ASC distance to camera
   */
  void SortTargetsByProximity(std::vector<DetectedObject> &objects);
  void PublishTrackedTarget(const TrackedTarget &target, int index);

  TargetTrackerConfig m_config;
  std::function<frc::Pose2d()> m_poseGetter;
  std::function<frc::Field2d *()> m_fieldGetter;
  std::vector<TrackedTarget> m_trackedTargets;
};
} // namespace subzero