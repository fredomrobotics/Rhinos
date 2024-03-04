// Copyright (c) Paolo Reyes for Rhinos.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/DutyCycleEncoder.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class ArmSubsystem : public frc2::SubsystemBase {
 public:
    // Arm constructor
    ArmSubsystem(void);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    // Subsystem methods go here.

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear speeds have no effect on the angular speed.
     *
     * @param desiredAngle Angle to move the arm to.
     */
    void MoveToAngle(double desiredAngle);

    /**
    * Locks the arm at the current angle.
    */
    void LockAngle(bool intakePositon, bool outakePosition, bool shootingPosition);

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    double GetAngle(void);

 private:
   ctre::phoenix6::hardware::TalonFX m_armMotor1;
   ctre::phoenix6::hardware::TalonFX m_armMotor2;
   double setpoint;
   frc::DutyCycleEncoder s_armAbsoluteEncoder;

   double pidOutput;
   double error;
   units::volt_t motorsOutput;
};