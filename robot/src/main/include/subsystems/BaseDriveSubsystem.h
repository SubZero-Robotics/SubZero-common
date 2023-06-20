// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <frc/AnalogGyro.h>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class BaseDriveSubsystem : public frc2::SubsystemBase {
   public:
    DriveSubsystem(std::vector<Encoders>& encoders);

    void DisabledInit();

    void TeleopInit();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void SimulationPeriodic() override;

    // Subsystem methods go here.

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    void ResetEncoders();

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive
     * more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be
     * constrained
     */
    void SetMaxOutput(double maxOutput);

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    units::degree_t GetHeading();

    /**
     * Translate NavX into Rotation2D values.
     *
     * @return the robot's heading in degrees, coninuous vectorization from 360
     * to 361
     */
    units::degree_t Get2dAngle();

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    double GetTurnRate();

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The current pose of the robot
     */
    frc::Pose2d GetPose();

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    void ResetOdometry(frc::Pose2d pose);

    frc::Field2d& GetField();

   private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.

    /**
     * @brief Gets the Average of two encoder positions
     * @param encoders The encoders
     * @return double The average position
     */
    static double AverageEncoderPosition(Encoders);

    /**
     * @brief Get the Average of the encoders minus an offset
     *
     * @param encoders
     * @param offset The offset used to correct the average
     * @return double The corrected encoder position
     */
    static double GetEncoder(Encoders, double);

    /**
     * @brief Get the average velocity of the encodes
     *
     * @return double velocity in m/s
     */
    static units::meters_per_second_t AverageEncoderVelocity(Encoders);

    /**
     * @brief Set all motors to brake mode
     *
     */
    void SetBrakeMode();

    void SetCoastMode();

    frc::Field2d field;

    // Helper methods to convert between meters and native units
    static int DistanceToNativeUnits(units::meter_t position);
    static int VelocityToNativeUnits(units::meters_per_second_t velocity);
    static units::meter_t NativeUnitsToDistanceMeters(double sensorCounts);

    // Odometry should be declared in the actual class, but pose can exist here
    frc::Rotation2d currentrobotAngle;  // is zeroed by default
    frc::Pose2d currentRobotPose;       // is also zeroed by default

    // navx
    double gyroAngle = 0.0;  // What is the angle (degrees) from the gyro?
    double gyroRate = 0.0;   // What is angle change (deg/sec)
    AHRS ahrs{frc::SPI::Port::kMXP};

#ifdef IS_SIMULATION
    frc::AnalogGyro m_gyro{1};
    frc::sim::AnalogGyroSim m_gyroSim{m_gyro};
#endif

    // The drive's config for trajectory
    frc::TrajectoryConfig* trajectoryConfig;
};
