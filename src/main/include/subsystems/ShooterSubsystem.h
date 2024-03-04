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

    /**
     * Checks if the shooter is at speed
     */
    bool isAtSpeed(void);

    void resetIntegralError(void);

 private:
    rev::CANSparkMax m_shooterSparkMax1;
    rev::CANSparkMax m_shooterSparkMax2;
    rev::SparkRelativeEncoder m_shootingEncoder1 =
      m_shooterSparkMax1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkRelativeEncoder m_shootingEncoder2 =
      m_shooterSparkMax2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    double setpoint;
    double error1;
    double error2;
    double integralError1 = 0;
    double integralError2 = 0;
};