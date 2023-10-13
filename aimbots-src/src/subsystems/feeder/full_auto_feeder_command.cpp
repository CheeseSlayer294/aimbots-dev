#include "full_auto_feeder_command.hpp"

namespace src::Feeder {

RunFeederCommand::RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float feederRPM) 
    : drivers(drivers),
    feeder(feeder),
    feederRPM(feederRPM)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void RunFeederCommand::initialize() {}

void RunFeederCommand::execute() {
    feeder->setTargetRPM(feederRPM);
}

void RunFeederCommand::end(bool interrupted) {}

bool RunFeederCommand::isReady() { return true; }

bool RunFeederCommand::isFinished() const { return false; }

};