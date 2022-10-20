#include "burst_feeder_command.hpp"
#ifndef TARGET_DART
namespace src::Feeder {
BurstFeederCommand::BurstFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold, int burstLength)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      startingTotalBallCount(0),
      burstLength(burstLength) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void BurstFeederCommand::initialize() {
    startingTotalBallCount = feeder->getTotalLimitCount();
}

void BurstFeederCommand::execute() {
    feeder->setTargetRPM(speed);
}

void BurstFeederCommand::end(bool) {
    feeder->setTargetRPM(0);
}

bool BurstFeederCommand::isReady() {
    return feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

bool BurstFeederCommand::isFinished() const {
    int elapsedTotal = feeder->getTotalLimitCount() - startingTotalBallCount;
    return (elapsedTotal >= burstLength) || !feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

}  // namespace src::Feeder
#endif