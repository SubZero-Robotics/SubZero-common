#include "subzero/motor/RevPidMotorController.h"

using namespace subzero;

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::RevPidMotorController(std::string name, 
                                TMotor& motor, 
                                bool hasAbsoluteEncoder,
                                PidSettings settings,
                                units::angular_velocity::revolutions_per_minute_t maxRpm) :
    IPidMotorController{name}, m_motor{motor}, m_hasAbsoluteEncoder{hasAbsoluteEncoder},
    m_relativeEncoder{m_motor.GetEncoder()}, 
    m_absoluteEncoder{m_hasAbsoluteEncoder ? &m_motor.GetAbsoluteEncoder() : nullptr},
    m_maxRpm{maxRpm}, m_isInitialized{false},
    m_pidSettings{settings} {}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::Set(double percentage) {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::Set(units::voltage::volt_t voltage) {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::SetPidTolerance(double tolerance) {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::Update() {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::RunWithVelocity(units::angular_velocity::revolutions_per_minute_t rpm) {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::RunWithVelocity(double percentage) {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::RunToPosition(double position) {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::ResetEncoder() {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
double RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::GetEncoderPosition() {
        return 0.0;
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
std::optional<double> RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::GetAbsoluteEncoderPosition() {
        return 0.0;
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::SetEncoderConversionFactor(double factor) {
    
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::SetAbsoluteEncoderConversionFactor(double factor) {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::Stop() {

}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
const PidSettings& RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::GetPidSettings() {
        return m_pidSettings;
}

template <typename TMotor, typename TController, typename TRelativeEncoder,
        typename TAbsoluteEncoder, typename TPidConfig>
void RevPidMotorController<TMotor, TController, TRelativeEncoder, TAbsoluteEncoder,
    TPidConfig>::UpdatePidSettings(PidSettings settings) {

}