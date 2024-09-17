#include "subsystems/ShooterSubsystem.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

#define FFMotor1(speed) speed*0.00221+0.138
#define FFMotor2(speed) speed*0.0022+0.107

using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem(void) : 
                m_shooterSparkMax1{kShooterLeftMotorCanId, rev::CANSparkMax::MotorType::kBrushless}, 
                m_shooterSparkMax2{kShooterRightMotorCanId, rev::CANSparkMax::MotorType::kBrushless} {
    m_shooterSparkMax1.RestoreFactoryDefaults();
    m_shooterSparkMax1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterSparkMax1.SetSmartCurrentLimit(kShooterMotor1CurrentLimit.value());
    m_shooterSparkMax1.BurnFlash();
    m_shooterSparkMax2.RestoreFactoryDefaults();
    m_shooterSparkMax2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_shooterSparkMax2.SetSmartCurrentLimit(kShooterMotor2CurrentLimit.value());
    m_shooterSparkMax2.BurnFlash();

    this->previous_time = (double)frc::Timer::GetFPGATimestamp();
}

void ShooterSubsystem::Periodic() {
    this->error1 = this->setpoint - m_shootingEncoder1.GetVelocity();
    this->error2 = this->setpoint - m_shootingEncoder2.GetVelocity();
    #define SPEEDINTEGRAL 300
    if (this->error1 < SPEEDINTEGRAL && this->error1 > -SPEEDINTEGRAL) {
        this->integralError1 += this->error1*((double)frc::Timer::GetFPGATimestamp()-previous_time);
    } else {
        this->integralError1 = 0;
    }
    if (this->error2 < SPEEDINTEGRAL && this->error2 > -SPEEDINTEGRAL) {
        this->integralError2 += this->error2*((double)frc::Timer::GetFPGATimestamp()-previous_time);
    } else {
        this->integralError2 = 0;
    }
    m_shooterSparkMax1.SetVoltage((units::volt_t)(FFMotor1(this->setpoint)+this->integralError1*kShooterI));
    m_shooterSparkMax2.SetVoltage((units::volt_t)(FFMotor2(this->setpoint)+this->integralError2*kShooterI));

    frc::SmartDashboard::PutNumber("Motor Speed 1:", m_shootingEncoder1.GetVelocity());
    frc::SmartDashboard::PutNumber("Motor Speed 2:", m_shootingEncoder2.GetVelocity());

    previous_time = (double)frc::Timer::GetFPGATimestamp();
}

void ShooterSubsystem::setSpeed(double speed) {
    
    this->setpoint = speed;
}

void ShooterSubsystem::setVoltage(double voltage) {
    m_shooterSparkMax1.SetVoltage((units::volt_t)voltage);
    m_shooterSparkMax2.SetVoltage((units::volt_t)voltage);
}

void ShooterSubsystem::resetIntegralError(void) { 
    this->integralError1 = 0;
    this->integralError2 = 0;
}

bool ShooterSubsystem::isAtSpeed(void) {
    return this->error1 < kShooterSpeedThreshold && this->error1 > -kShooterSpeedThreshold && this->error2 < kShooterSpeedThreshold && this->error2 > -kShooterSpeedThreshold;
}