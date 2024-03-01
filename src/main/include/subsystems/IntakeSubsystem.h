// Copyright (c) Paolo Reyes for Rhinos.

#pragma once

#include <rev/CANSparkMax.h>
#include "frc/DigitalInput.h"
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
    // Arm constructor
    IntakeSubsystem(void);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    // Subsystem methods go here.

     /**
     * Intakes/Outakes a note
     */
    void runUntilSensor(double power);

    /**
     * Intakes/Outakes a note
     */
    void run(double power);

 private:
    rev::CANSparkMax m_intakeSparkMax;
    double meanSpeed;
    frc::DigitalInput limitSensor;
    bool noteInside = false;
};