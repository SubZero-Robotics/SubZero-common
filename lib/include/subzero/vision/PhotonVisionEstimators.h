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
 * @brief Combines estimated poses from an aritrary number of PhotonVision
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
    explicit PhotonCameraEstimator(photon::PhotonPoseEstimator &est)
        : estimator{est} {
      camera = estimator.GetCamera();
    }

    photon::PhotonPoseEstimator &estimator;
    std::shared_ptr<photon::PhotonCamera> camera;
  };

  explicit PhotonVisionEstimators(std::vector<PhotonCameraEstimator> &estms,
                                  Eigen::Matrix<double, 3, 1> singleTagStdDevs,
                                  Eigen::Matrix<double, 3, 1> multiTagStdDevs)
      : m_cameraEstimators{estms},
        m_singleTagStdDevs{singleTagStdDevs},
        m_multiTagStdDevs{multiTagStdDevs} {
    for (auto &est : m_cameraEstimators) {
      est.estimator.SetMultiTagFallbackStrategy(
          photon::PoseStrategy::LOWEST_AMBIGUITY);
    }
  }

  std::optional<photon::EstimatedRobotPose> GetPoseFromCamera(
      frc::Pose3d prevPose, photon::PhotonPoseEstimator &est,
      photon::PhotonCamera &camera, double maxAbmiguity = 0.2) {
    est.SetReferencePose(prevPose);
    auto camResult = camera.GetLatestResult();

    std::optional<photon::EstimatedRobotPose> visionEst;

    if (camResult.HasTargets() &&
        (camResult.targets.size() > 1 ||
         camResult.targets[0].GetPoseAmbiguity() <= maxAbmiguity)) {
      visionEst = est.Update(camResult);
    }

    if (visionEst.has_value()) {
      auto estimatedPose = visionEst.value().estimatedPose;
      // Replace 100_m with actual field dimensions
      if (estimatedPose.X() > 0_m && estimatedPose.X() <= 100_m &&
          estimatedPose.Y() > 0_m && estimatedPose.Y() <= 100_m) {
        return visionEst;
      }
    }

    return std::nullopt;
  }

  // See:
  // https://github.com/Hemlock5712/2023-Robot/blob/Joe-Test/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
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
      auto camPose =
          GetPoseFromCamera(frc::Pose3d(estimator.GetEstimatedPosition()),
                            est.estimator, *est.camera);
      if (camPose.has_value()) {
        auto pose = camPose.value();
        AddVisionMeasurement(pose, estimator, est);
      }
    }
  }

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(
      photon::EstimatedRobotPose &pose, PhotonCameraEstimator &photonEst) {
    // TODO:
    Eigen::Matrix<double, 3, 1> estStdDevs = m_singleTagStdDevs;
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    // ConsoleWriter.logVerbose("Vision", "targets used length
    // %d", pose.targetsUsed.size());
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
      // TODO:
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
}  // namespace subzero