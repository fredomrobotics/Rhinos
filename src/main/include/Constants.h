#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/acceleration.h>
#include <numbers>

#pragma once

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4.8_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
constexpr units::meter_t kTrackWidth =
    0.661_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.661_m;  // Distance between centers of front and back wheels on robot

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId = 6;
constexpr int kRearLeftDrivingCanId = 8;
constexpr int kFrontRightDrivingCanId = 3;
constexpr int kRearRightDrivingCanId = 5;

constexpr int kFrontLeftTurningCanId = 7;
constexpr int kRearLeftTurningCanId = 9;
constexpr int kFrontRightTurningCanId = 2;
constexpr int kRearRightTurningCanId = 4;
}  // namespace DriveConstants

namespace ModuleConstants {
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 14;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kDriverControllerPort2 = 1;
constexpr double kDriveDeadband = 0.05;
constexpr double kIntakeDeadband = 0.05;
constexpr double kShooterDeadband = 0.05;
constexpr double kClimberDeadband = 0.05;

constexpr int kLeftBumper = 5;
constexpr int kRightStickButton = 10;
constexpr int kLeftYStick = 1;
constexpr int kLeftXStick = 0;
constexpr int kRightXStick = 4;
constexpr int kAButton = 1;
constexpr int kBButton = 2;
constexpr int kXButton = 3;
constexpr int kYButton = 4;
constexpr int kRigthTrigger = 3;
constexpr int kLeftTrigger = 2;
}  // namespace OIConstants

namespace ArmConstants {
constexpr double kArmAngleThreshold = 0.05;
constexpr int kArmAbsoluteEncoderPort = 0;
constexpr int kArmMotor1CanId = 13;
constexpr int kArmMotor2CanId = 14;
constexpr double kArmP = 100;
constexpr double kArmI = 40;
constexpr double kArmD = 70;

constexpr double kMaxOutputArmVoltage = 12;

constexpr double kArmInitialAngle = 68; //68
constexpr double kArmIntakeAngle = 0;
constexpr double kArmOutakeAngle = 86;
constexpr double kArmShootingAngle = 18;

constexpr int kArmPIDPositionToleranceDegrees = 3;
constexpr int kArmPIDVelocityToleranceDegreesPerSec = 15;
}  // namespace ArmConstants

namespace IntakeConstants {
constexpr int kIntakeMotorCanId = 10;
constexpr int kIntakeLimitPort = 1;
constexpr units::ampere_t kIntakeMotorCurrentLimit = 30_A;
constexpr int kIntakeMotor2CanId = 15;
constexpr int kIntakeMotor3CanId = 16;
}  // namespace IntakeConstants

namespace ShooterConstants {
constexpr int kShooterLeftMotorCanId = 11;
constexpr int kShooterRightMotorCanId = 12;
constexpr units::ampere_t kShooterMotor1CurrentLimit = 40_A;
constexpr units::ampere_t kShooterMotor2CurrentLimit = 40_A;
constexpr double kShooterI = 0.0015;
constexpr double kShooterD = 0;
constexpr double kShooterSpeedThreshold = 50;
} // namespace ShooterConstants

namespace LauncherConstants {
constexpr double kIntakeAndOutakeThreshold = 0.3;
constexpr double kIntakeFeedingPower = 0.7;
constexpr double kOutakeSpeed = 1500;
constexpr double kShootingSpeed = 4500;
constexpr double kIdleShootingSpeed = 0;
constexpr double kDelayBeforeShooting = 1.5;
} // namespace LauncherConstants

namespace ClimberConstants {
constexpr int kClimberMotor1CanId = 17;
constexpr int kClimberMotor2CanId = 18;
constexpr units::ampere_t kClimberMotor1CurrentLimit = 3.8_A;
constexpr units::ampere_t kClimberMotor2CurrentLimit = 3.8_A;
} // namespace ClimberConstants

namespace LimelightConstants {
constexpr double kDistanceM = 1.115;
constexpr double kAlignmentP = 0.05;
constexpr double kAlignmentI = 0.04;
} // namespace LimelightConstants