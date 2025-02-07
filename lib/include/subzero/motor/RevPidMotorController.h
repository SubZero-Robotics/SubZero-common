#pragma once 

#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/ClosedLoopSlot.h>
#include <rev/SparkBase.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "subzero/motor/IPidMotorController.h"

using namespace subzero;

namespace subzero {

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
class RevPidMotorController : public IPidMotorController {
public:
    explicit RevPidMotorController(std::string name,
                                TMotor& motor,
                                bool hasAbsoluteEncoder,
                                PidSettings settings,
                                units::angular_velocity::revolutions_per_minute_t maxRpm);

    void Set(double percentage) override;

    void Set(units::voltage::volt_t voltage) override;

    void SetPidTolerance(double tolerane) override;

    void Update() override;

    void RunWithVelocity(units::angular_velocity::revolutions_per_minute_t rpm) override;

    void RunWithVelocity(double percentage) override;

    void RunToPosition(double positoin) override;

    void ResetEncoder() override;

    double GetEncoderPosition() override;

    std::optional<double> GetAbsoluteEncoderPosition() override;

    void SetEncoderConversionFactor(double factor) override;

    void SetAbsoluteEncoderConversionFactor(double factor) override;

    void Stop() override;

    const PidSettings& GetPidSettings() override;

    void UpdatePidSettings(PidSettings settings) override;
private:
    TMotor &m_motor;
    TController m_controller;
    TRelativeEncoder m_relativeEncoder;
    std::unique_ptr<TAbsoluteEncoder> m_absoluteEncoder;
    TPidConfig m_config;
    PidSettings m_pidSettings;
    bool m_hasAbsoluteEncoder;
    bool m_absolutePositionEnabled;
    double m_absoluteTarget;
    const units::revolutions_per_minute_t m_maxRpm;
    bool m_isInitialized;
};

}