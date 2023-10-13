#include "subsystems/feeder/stop_feeder_command.hpp"

#ifdef FEEDER_COMPATIBLE
namespace src::Feeder {

StopFeederCommand::StopFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder) 
    : drivers(drivers),
    feeder(feeder) 
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void StopFeederCommand::initialize() {}

void StopFeederCommand::execute() {
    feeder->setTargetRPM(0);
}

void StopFeederCommand::end(bool interrupted) {}

bool StopFeederCommand::isReady() { return true; }

bool StopFeederCommand::isFinished() const { return false; }

};

#endif // closes ifdef
