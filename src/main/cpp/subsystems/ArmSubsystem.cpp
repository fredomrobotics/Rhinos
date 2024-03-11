#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

using namespace ArmConstants;

ArmSubsystem::ArmSubsystem(void) : 
            m_armMotor1{kArmMotor1CanId}, 
            m_armMotor2{kArmMotor2CanId},
            s_armAbsoluteEncoder{kArmAbsoluteEncoderPort} {

    // Sets the arm motors to brake mode
    m_armMotor1.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_armMotor2.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    // Sets the tolerance of the PID controller
    this->setpoint = kArmInitialAngle;
}

void ArmSubsystem::Periodic() {
    error = this->setpoint - s_armAbsoluteEncoder.GetAbsolutePosition();
    pidOutput = error*kArmP;
}

void ArmSubsystem::MoveToAngle(double desiredAngle) {
    this->setpoint = desiredAngle;
}

void ArmSubsystem::LockAngle(bool intakePosition, bool outakePosition, bool shootingPosition) {
    if (intakePosition) {
        this->setpoint = kArmIntakeAngle;
    } else if (outakePosition) {
        this->setpoint = kArmOutakeAngle;
    } else if (shootingPosition) {
        this->setpoint = kArmShootingAngle;
    }

    if (pidOutput > kMaxOutputArmVoltage) {
        motorsOutput = (units::volt_t)kMaxOutputArmVoltage;
    } else if (pidOutput < -kMaxOutputArmVoltage) {
        motorsOutput = -(units::volt_t)kMaxOutputArmVoltage;
    } else {
        motorsOutput = (units::volt_t)pidOutput;
    }

    m_armMotor1.SetVoltage(motorsOutput);
    m_armMotor2.SetVoltage(-motorsOutput);
}

double ArmSubsystem::GetAngle(void) {
  return s_armAbsoluteEncoder.GetAbsolutePosition();
}

void ArmSubsystem::waitForAngle(double angle) {
    while (abs(s_armAbsoluteEncoder.GetAbsolutePosition() - angle) > kArmAngleThreshold) {
        this->MoveToAngle(angle);
    }
}