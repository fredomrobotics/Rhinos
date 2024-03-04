#include "subsystems/ClimberSubsystem.h"

#include "Constants.h"

using namespace ClimberConstants;

ClimberSubsystem::ClimberSubsystem(void) : 
            m_climberMotor1{kClimberMotor1CanId}, 
            m_climberMotor2{kClimberMotor2CanId} {

    // Sets the arm motors to brake mode
    m_climberMotor1.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_climberMotor2.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void ClimberSubsystem::Periodic() {
}

void ClimberSubsystem::Move(double power) {
    if (power < 0) {
        m_climberMotor1.Set(power/2);
        m_climberMotor2.Set(-power/2);
    } else {
        m_climberMotor1.Set(0);
        m_climberMotor2.Set(0);
    }
}