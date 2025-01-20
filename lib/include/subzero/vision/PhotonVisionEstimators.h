#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace subzero {

/**
 * @brief Combines estimated poses from an arbitrary number of PhotonVision
 * cameras and applies them to a Holonomic pose estimator
 *
 */
class PhotonVisionEstimators {
public:
  /**
   * @brief Represents a single camera
   *
   */
  class PhotonCameraEstimator {
  public:
    explicit PhotonCameraEstimator(photon::PhotonPoseEstimator &est,
                                   photon::PhotonCamera &cam)
        : estimator{est}, camera{cam} {}

    photon::PhotonPoseEstimator &estimator;
    photon::PhotonCamera &camera; // Changed to reference
  };

  explicit PhotonVisionEstimators(std::vector<PhotonCameraEstimator> &estms,
                                  Eigen::Matrix<double, 3, 1> singleTagStdDevs,
                                  Eigen::Matrix<double, 3, 1> multiTagStdDevs)
      : m_cameraEstimators{estms}, m_singleTagStdDevs{singleTagStdDevs},
        m_multiTagStdDevs{multiTagStdDevs} {
    for (auto &est : m_cameraEstimators) {
      est.estimator.SetMultiTagFallbackStrategy(
          photon::PoseStrategy::LOWEST_AMBIGUITY);
    }
  }

  std::vector<photon::EstimatedRobotPose>
  GetPosesFromCamera(frc::Pose3d prevPose, photon::PhotonPoseEstimator &est,
                     photon::PhotonCamera &camera, double maxAmbiguity = 0.2) {
    est.SetReferencePose(prevPose);

    std::vector<photon::EstimatedRobotPose> validPoses;

    // Photon now returns all of the poses that we haven't integrated
    // rather than just the latest, so we must return all that are valid
    for (const auto &result : camera.GetAllUnreadResults()) {
      // Check if the result has valid targets
      if (result.HasTargets() &&
          (result.targets.size() > 1 ||
           result.targets[0].GetPoseAmbiguity() <= maxAmbiguity)) {
        // Attempt to update the pose estimator with the result
        std::optional<photon::EstimatedRobotPose> visionEst =
            est.Update(result);

        // If a valid pose is produced, check its boundaries
        if (visionEst.has_value()) {
          auto estimatedPose = visionEst.value().estimatedPose;
          if (estimatedPose.X() > 0_m && estimatedPose.X() <= 100_m &&
              estimatedPose.Y() > 0_m && estimatedPose.Y() <= 100_m) {
            // Add the valid pose to the vector
            validPoses.push_back(visionEst.value());
          }
        }
      }
    }

    return validPoses;
  }

  /**
   * @brief Call in the drivetrain's Periodic method to update the estimated
   * robot's position
   *
   * @param estimator
   * @param test
   */
  void UpdateEstimatedGlobalPose(frc::SwerveDrivePoseEstimator<4U> &estimator,
                                 bool test) {
    for (auto &est : m_cameraEstimators) {
      auto camPoses =
          GetPosesFromCamera(frc::Pose3d(estimator.GetEstimatedPosition()),
                             est.estimator, est.camera);
      // if (camPose.has_value()) {
      //   auto pose = camPose.value();
      //   AddVisionMeasurement(pose, estimator, est);
      // }

      if (camPoses.size() > 0) {
        for (auto &pose : camPoses) {
          AddVisionMeasurement(pose, estimator, est);
        }
      }
    }
  }

  Eigen::Matrix<double, 3, 1>
  GetEstimationStdDevs(photon::EstimatedRobotPose &pose,
                       PhotonCameraEstimator &photonEst) {
    // TODO:
    Eigen::Matrix<double, 3, 1> estStdDevs = m_singleTagStdDevs;
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto &tgt : pose.targetsUsed) {
      auto tagPose =
          photonEst.estimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(
            pose.estimatedPose.ToPose2d().Translation());
      }
    }

    if (numTags == 0) {
      return estStdDevs;
    }

    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = m_multiTagStdDevs;
    }

    if (numTags == 1 && avgDist > 4_m) {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    } else {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }

    return estStdDevs;
  }

private:
  void AddVisionMeasurement(photon::EstimatedRobotPose &estimate,
                            frc::SwerveDrivePoseEstimator<4U> &estimator,
                            PhotonCameraEstimator &photonEst) {
    auto stdDevs = GetEstimationStdDevs(estimate, photonEst);
    wpi::array<double, 3> newStdDevs{stdDevs(0), stdDevs(1), stdDevs(2)};
    estimator.AddVisionMeasurement(estimate.estimatedPose.ToPose2d(),
                                   estimate.timestamp, newStdDevs);
  }

  std::vector<PhotonCameraEstimator> &m_cameraEstimators;
  Eigen::Matrix<double, 3, 1> m_singleTagStdDevs;
  Eigen::Matrix<double, 3, 1> m_multiTagStdDevs;

  units::second_t lastEstTimestamp{0_s};
};

} // namespace subzero