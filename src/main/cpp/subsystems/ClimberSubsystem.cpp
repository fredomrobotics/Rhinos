#include "subsystems/ClimberSubsystem.h"

#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ClimberConstants;

ClimberSubsystem::ClimberSubsystem(void) : 
    m_climberMotor1{kClimberMotor1CanId, rev::CANSparkMax::MotorType::kBrushed}, 
    m_climberMotor2{kClimberMotor2CanId, rev::CANSparkMax::MotorType::kBrushed} {

        m_climberMotor1.RestoreFactoryDefaults();
        m_climberMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_climberMotor1.SetSmartCurrentLimit(kClimberMotor1CurrentLimit.value());
        m_climberMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_climberMotor1.BurnFlash();

        m_climberMotor2.RestoreFactoryDefaults();
        m_climberMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_climberMotor2.SetSmartCurrentLimit(kClimberMotor1CurrentLimit.value());
        m_climberMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_climberMotor2.BurnFlash();

}

void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::setVoltage(double voltage, double input) {

    m_climberMotor1.SetVoltage((units::volt_t)voltage * input);
    m_climberMotor2.SetVoltage((units::volt_t)voltage * input);

    frc::SmartDashboard::PutNumber("input:", input);    

}