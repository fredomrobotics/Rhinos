#include "subsystems/LauncherSubsystem.h"

#include "Constants.h"

using namespace LauncherConstants;

LauncherSubsystem::LauncherSubsystem(void) : m_shooter{}, m_intake{} {
}

void LauncherSubsystem::Periodic() {
}

void LauncherSubsystem::shootAtSpeed(double speed) {
    
}

void LauncherSubsystem::intakeAndOutake(double speed1, double speed2, bool shoot) {
    if (shoot) {
        m_shooter.setSpeed(kShootingSpeed);
        if (waitCycles > (int)waitTime*50) {
            m_intake.run(kIntakeFeedingPower);
        }
        waitCycles += 1;
    } else {
        if (speed1 - speed2 > kIntakeAndOutakeThreshold) {
        m_intake.runUntilSensor(speed1 - speed2);
        } else if (speed1 - speed2 < -kIntakeAndOutakeThreshold) {
            m_intake.run(-(speed1 - speed2));
            m_shooter.setSpeed(kOutakeSpeed);
        } else {
            m_shooter.setSpeed(kIdleShootingSpeed);
            m_intake.stop();
        }
        waitCycles = 0;
    }
}

void LauncherSubsystem::autonomousShoot() {
    m_shooter.setSpeed(kShootingSpeed);
    units::time::second_t startTime = frc::Timer::GetFPGATimestamp();
    while (frc::Timer::GetFPGATimestamp() - startTime < 1.5_s) {
        m_intake.stop();
    }
    m_intake.run(kIntakeFeedingPower);
    m_shooter.setSpeed(kIdleShootingSpeed);
    m_intake.stop();
}

void LauncherSubsystem::intakeForSeconds(double seconds) {
    m_intake.run(0.5);
    units::time::second_t startTime = frc::Timer::GetFPGATimestamp();
    while (frc::Timer::GetFPGATimestamp() - startTime < (units::time::second_t)seconds) {
        m_intake.run(0.5);
    }
    m_intake.stop();
}

void LauncherSubsystem::resetIntegralError(void) {
    m_shooter.resetIntegralError();
}