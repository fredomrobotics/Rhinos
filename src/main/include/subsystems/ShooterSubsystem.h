// Copyright (c) Paolo Reyes for Rhinos.

#pragma once

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
    // Arm constructor
    ShooterSubsystem(void);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    // Subsystem methods go here.

     /**
     * Sets the speed of the shooter motors in RPM
     */
    void setSpeed(double speed);

    /**
     * Sets the speed of the shooter motors to outake the note
     */
    void setDefaultSpeed(void);

 private:
    rev::CANSparkMax m_shooterSparkMax1;
    rev::CANSparkMax m_shooterSparkMax2;

    double setpoint;
};