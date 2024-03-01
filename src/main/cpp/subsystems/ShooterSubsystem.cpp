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
}

void ShooterSubsystem::setSpeed(double speed) {
    m_shooterSparkMax1.Set(0.5);
}