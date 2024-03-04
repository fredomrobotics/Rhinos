// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/LauncherSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  // Subsystems
  // The drivetrain subsystem
  DriveSubsystem m_drive;

  // The Launcher subsystem
  LauncherSubsystem m_launcher;

  // Arm Subsystem
  ArmSubsystem m_arm;

  // Climber
  ClimberSubsystem m_climber;

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_driverController2{OIConstants::kDriverControllerPort2};

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
