#include "subsystems/shooter/stop_shooter_command.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {

StopShooterCommand::StopShooterCommand(Drivers* drivers, ShooterSubsystem* shooter)
    : drivers(drivers), shooter(shooter)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void StopShooterCommand::initialize() {}

void StopShooterCommand::execute() {
    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, 0.0f);
    shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void StopShooterCommand::end(bool interrupted) {}

bool StopShooterCommand::isReady() { return true; }

bool StopShooterCommand::isFinished() const { return false; }

} // namespace src::Shooter

#endif // #ifdef SHOOTER_COMPATIBLE