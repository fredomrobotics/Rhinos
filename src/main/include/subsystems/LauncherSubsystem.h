// Copyright (c) Paolo Reyes for Rhinos.

#pragma once

#include "ShooterSubsystem.h"
#include "IntakeSubsystem.h"
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class LauncherSubsystem : public frc2::SubsystemBase {
 public:
    // Launcher constructor
    LauncherSubsystem(void);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    // Subsystem methods go here.

     /**
     * Sets the speed of the shooter motors in RPM
     */
    void shootAtSpeed(double speed);

    /**
     * Reads the triggers and enables intaking and outaking
     */
    void intakeAndOutake(double speed1, double speed2, bool shoot);

    void resetIntegralError(void);

 private:
    ShooterSubsystem m_shooter;
    IntakeSubsystem m_intake;

    int waitCycles = 0;
    double waitTime = LauncherConstants::kDelayBeforeShooting;
};