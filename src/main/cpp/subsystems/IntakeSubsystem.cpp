#include "subsystems/IntakeSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace IntakeConstants;

IntakeSubsystem::IntakeSubsystem(void) : m_intakeSparkMax{kIntakeMotorCanId, rev::CANSparkMax::MotorType::kBrushed}, limitSensor{kIntakeLimitPort} {
    m_intakeSparkMax.RestoreFactoryDefaults();
    m_intakeSparkMax.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_intakeSparkMax.SetSmartCurrentLimit(kIntakeMotorCurrentLimit.value());
    m_intakeSparkMax.BurnFlash();
}

void IntakeSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Switch:", !limitSensor.Get());
}

void IntakeSubsystem::runUntilSensor(double power) {
    if (!limitSensor.Get()) {
        this->noteInside = true;
    }
    if (this->noteInside) {
        m_intakeSparkMax.Set(0);
    } else {
        m_intakeSparkMax.Set(power*0.75);
    }
}

void IntakeSubsystem::run(double power) {
    this->noteInside = false;
    m_intakeSparkMax.Set(power);
}

void IntakeSubsystem::stop(void) {
    m_intakeSparkMax.Set(0);
}