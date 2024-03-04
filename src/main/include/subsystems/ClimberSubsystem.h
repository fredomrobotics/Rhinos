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

    /**
     * Moves the climber
     */
    void Move(double power);

 private:
   ctre::phoenix6::hardware::TalonFX m_climberMotor1;
   ctre::phoenix6::hardware::TalonFX m_climberMotor2;
};