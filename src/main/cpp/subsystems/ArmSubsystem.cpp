#include "subsystems/ArmSubsystem.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "Constants.h"

using namespace ArmConstants;

ArmSubsystem::ArmSubsystem(void): 

    //sets IDs

    m_armMotor1{kArmMotor1CanId}, 
    m_armMotor2{kArmMotor2CanId},
    s_armAbsoluteEncoder{kArmAbsoluteEncoderPort} 
    
    {

    // Sets the arm motors to brake mode
    m_armMotor1.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_armMotor2.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    // Sets the tolerance of the PID controller
    this->setpoint = transToUnitCircle(kArmInitialAngle);
    this->previous_time = (double)frc::Timer::GetFPGATimestamp();

}

void ArmSubsystem::Periodic() {

    //get error

    error = this->setpoint - s_armAbsoluteEncoder.GetAbsolutePosition();

    //sets integral error

    if (error < 0.025 && error > -0.025) {
        integral_error += error*((double)frc::Timer::GetFPGATimestamp()-previous_time);
    } else {
        integral_error = 0;
    }

    pidOutput = error*kArmP+integral_error*kArmI;

    previous_time = (double)frc::Timer::GetFPGATimestamp();

    //gets vertical angle 

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0) + 23;
    double tv = table->GetNumber("tv",0.0);

    if(tv == 1){

        double distanceToSpeaker = LimelightConstants::kDistanceM/tan(targetOffsetAngle_Vertical * M_PI/180);
        frc::SmartDashboard::PutNumber("ty:", targetOffsetAngle_Vertical);
        frc::SmartDashboard::PutNumber("distanceToSpeaker:", distanceToSpeaker);
        frc::SmartDashboard::PutNumber("Radians:", targetOffsetAngle_Vertical * M_PI/180);
        frc::SmartDashboard::PutNumber("Tan:", tan(targetOffsetAngle_Vertical * M_PI/180));

    }else{
        
    }
}

void ArmSubsystem::MoveToAngle(double desiredAngle) {
    this->setpoint = desiredAngle;
}

void ArmSubsystem::LockAngle(bool intakePosition, bool outakePosition, bool shootingPosition, bool bButton) {
    
    if(bButton){

            std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
            double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0) + 23;
            double tv = table->GetNumber("tv",0.0);

        if(tv == 1){

            double distanceToSpeaker = LimelightConstants::kDistanceM/tan(targetOffsetAngle_Vertical * M_PI/180);
            //double e = 67 - targetOffsetAngle_Vertical;
            double e = 10.4 * distanceToSpeaker;

            this->setpoint = transToUnitCircle(e);
        }

    }else{

        if (intakePosition) {this->setpoint = transToUnitCircle(kArmIntakeAngle);} 
        else if (outakePosition) {this->setpoint = transToUnitCircle(kArmOutakeAngle);} 
        else if (shootingPosition) {this->setpoint = transToUnitCircle(kArmShootingAngle);}

        if (pidOutput > kMaxOutputArmVoltage) {motorsOutput = (units::volt_t)kMaxOutputArmVoltage;} 
        else if (pidOutput < -kMaxOutputArmVoltage) {motorsOutput = -(units::volt_t)kMaxOutputArmVoltage;} 
        else {motorsOutput = (units::volt_t)pidOutput;}

        m_armMotor1.SetVoltage(motorsOutput);
        m_armMotor2.SetVoltage(-motorsOutput);

    }
}

double ArmSubsystem::GetAngle(void) {
  return s_armAbsoluteEncoder.GetAbsolutePosition();
}

void ArmSubsystem::waitForAngle(double angle) {
    while (abs(s_armAbsoluteEncoder.GetAbsolutePosition() - angle) > kArmAngleThreshold) {
        this->MoveToAngle(angle);
    }
}

void ArmSubsystem::resetIntegralError() {
    integral_error = 0;
}

double ArmSubsystem::transToUnitCircle(double angle) {
    return (-angle+320)/360.0;
}