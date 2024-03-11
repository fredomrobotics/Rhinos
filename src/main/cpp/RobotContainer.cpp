// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ArmSubsystem.h"

using namespace DriveConstants;


RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRawAxis(OIConstants::kLeftYStick), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRawAxis(OIConstants::kLeftXStick), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRawAxis(OIConstants::kRightXStick), OIConstants::kDriveDeadband)},
            true, true);
      },
      {&m_drive}));

    // Set up default intake command
    // The right trigger controls the intake in, and the left trigger controls the intake out
    m_launcher.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_launcher.intakeAndOutake(frc::ApplyDeadband(m_driverController2.GetRawAxis(OIConstants::kRigthTrigger), OIConstants::kIntakeDeadband), 
                                  frc::ApplyDeadband(m_driverController2.GetRawAxis(OIConstants::kLeftTrigger), OIConstants::kIntakeDeadband),
                                  m_driverController2.GetRawButton(OIConstants::kBButton));
      },
      {&m_launcher}));

    m_arm.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_arm.LockAngle(m_driverController2.GetRawButton(OIConstants::kAButton), 
                        m_driverController2.GetRawButton(OIConstants::kXButton), 
                        m_driverController2.GetRawButton(OIConstants::kYButton));
      },
      {&m_arm}));

      m_climber.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_climber.Move(frc::ApplyDeadband(m_driverController2.GetRawAxis(OIConstants::kLeftYStick), OIConstants::kClimberDeadband));
      },
      {&m_climber}));
}

void RobotContainer::ConfigureButtonBindings() {
    // Set X position to avoid the robot from moving when pressing the left bumper
    frc2::JoystickButton(&m_driverController, OIConstants::kLeftBumper)
        .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
    
    // Reset the gyro when pressing the right stick button
    frc2::JoystickButton(&m_driverController, OIConstants::kRightStickButton)
        .OnTrue(new frc2::RunCommand([this] { m_drive.ZeroHeading(); }, {&m_drive}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]() { m_arm.waitForAngle(ArmConstants::kArmShootingAngle); }, {&m_arm}),
      frc2::InstantCommand([this]() { m_launcher.autonomousShoot(); }, {&m_launcher}),
      frc2::InstantCommand([this]() { m_arm.waitForAngle(ArmConstants::kArmIntakeAngle); }, {&m_arm}),
      frc2::InstantCommand([this]() { m_drive.DriveForSeconds(-units::meters_per_second_t{-2}, 1.5); m_launcher.intakeForSeconds(2);}, {&m_drive, &m_launcher}),
      frc2::InstantCommand([this]() { m_drive.DriveForSeconds(-units::meters_per_second_t{2}, 1.5);}, {&m_drive}),
      frc2::InstantCommand([this]() { m_arm.waitForAngle(ArmConstants::kArmShootingAngle); }, {&m_arm}),
      frc2::InstantCommand([this]() { m_launcher.autonomousShoot(); }, {&m_launcher}),
      frc2::InstantCommand([this]() { m_drive.DriveForSeconds(-units::meters_per_second_t{-2}, 2.5);}, {&m_drive})
  );
}