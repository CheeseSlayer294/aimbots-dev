#include "unjam_feeder_command.hpp"

namespace src::Feeder {
UnjamFeederCommand::UnjamFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers), feeder(feeder), speed(0) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void UnjamFeederCommand::initialize() {
    speed = 0.0f;
    feeder->setTargetRPM(0.0f);
}

void UnjamFeederCommand::execute() {
    speed = -3.0 * FEEDER_DEFAULT_RPM;
    feeder->setTargetRPM(speed);
}

void UnjamFeederCommand::end(bool) {}

bool UnjamFeederCommand::isReady() {
    return true;
}

bool UnjamFeederCommand::isFinished() const {
    return false;  // finished condition (button released) or their api is nice and we don't have to
}
}  // namespace src::Feeder