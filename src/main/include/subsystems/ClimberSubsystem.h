// Copyright (c) Paolo Reyes for Rhinos.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
    // Climber constructor
    ClimberSubsystem(void);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    // Subsystem methods go here.

    void setVoltage(double voltage, double input);

    /**
     * Moves the climber
     */
    void Move(double power);

 private:
   rev::CANSparkMax m_climberMotor1;
   rev::CANSparkMax m_climberMotor2;
};