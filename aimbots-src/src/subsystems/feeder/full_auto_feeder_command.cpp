#include "full_auto_feeder_command.hpp"
#ifndef ENGINEER

namespace src::Feeder {

FullAutoFeederCommand::FullAutoFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, bool &barrelMovingFlag, float speed, float acceptableHeatThreshold)
    : drivers(drivers),
      feeder(feeder),
      barrelMovingFlag(barrelMovingFlag),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      unjamSpeed(-3000.0f)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FullAutoFeederCommand::initialize() {
    feeder->setTargetRPM(0.0f);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
}

void FullAutoFeederCommand::execute() {
    if (fabs(feeder->getCurrentRPM()) <= 10.0f && startupThreshold.execute()) {
        feeder->setTargetRPM(unjamSpeed);
        unjamTimer.restart(175);
    }

    if (unjamTimer.execute()) {
        feeder->setTargetRPM(speed);
        startupThreshold.restart(500);
    }
}

void FullAutoFeederCommand::end(bool) { feeder->setTargetRPM(0.0f); }

bool FullAutoFeederCommand::isReady() { return (feeder->isBarrelHeatAcceptable(acceptableHeatThreshold) && !barrelMovingFlag); }

bool FullAutoFeederCommand::isFinished() const { return (!feeder->isBarrelHeatAcceptable(acceptableHeatThreshold) || barrelMovingFlag); }

}  // namespace src::Feeder
#endif