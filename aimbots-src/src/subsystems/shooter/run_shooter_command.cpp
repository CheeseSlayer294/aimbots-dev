#include "utils/robot_specific_inc.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/run_shooter_command.hpp"
#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {

RunShooterCommand::RunShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter, float shooterRPM)
    : drivers(drivers),
      shooter(shooter),
      shooterRPM(shooterRPM)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void RunShooterCommand::execute() {
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, shooterRPM);
    shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void RunShooterCommand::initialize() {}

void RunShooterCommand::end(bool interupted) {}

bool RunShooterCommand::isReady() { return true; }

bool RunShooterCommand::isFinished() const { return false; }

}

#endif