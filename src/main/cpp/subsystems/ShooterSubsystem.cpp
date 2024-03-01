#include "subsystems/ShooterSubsystem.h"

#include "Constants.h"

using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem(void) : 
                m_shooterSparkMax1{kShooterLeftMotorCanId, rev::CANSparkMax::MotorType::kBrushless}, 
                m_shooterSparkMax2{kShooterRightMotorCanId, rev::CANSparkMax::MotorType::kBrushless} {
    m_shooterSparkMax1.RestoreFactoryDefaults();
    m_shooterSparkMax1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterSparkMax1.SetSmartCurrentLimit(kShooterMotor1CurrentLimit.value());
    m_shooterSparkMax1.BurnFlash();
    m_shooterSparkMax2.RestoreFactoryDefaults();
    m_shooterSparkMax2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterSparkMax2.SetSmartCurrentLimit(kShooterMotor2CurrentLimit.value());
    m_shooterSparkMax2.BurnFlash();
}

void ShooterSubsystem::Periodic() {
    this->error1 = this->setpoint - m_shootingEncoder1.GetVelocity();
    this->error2 = this->setpoint - m_shootingEncoder2.GetVelocity();
    this->integralError1 += this->error1*0.020;
    this->integralError2 += this->error2*0.020;
    m_shooterSparkMax1.SetVoltage((units::volt_t)(this->error1*kShooterP+this->integralError1*kShooterI));
    m_shooterSparkMax2.SetVoltage((units::volt_t)(this->error2*kShooterP+this->integralError2*kShooterI));
}

void ShooterSubsystem::setSpeed(double speed) {
    if (this->setpoint != speed) {
        this->integralError1 = 0;
        this->integralError2 = 0;
    }
    this->setpoint = speed;
}

bool ShooterSubsystem::isAtSpeed(void) {
    return m_shootingEncoder1.GetVelocity()-this->setpoint < kShooterSpeedThreshold && m_shootingEncoder2.GetVelocity()-this->setpoint < kShooterSpeedThreshold;
}