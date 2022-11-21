#include "test_feeder_command.hpp"

namespace src::Feeder {
TestFeederCommand::TestFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float speed, float acceptableHeatThreshold, int burstLength)
    : drivers(drivers),
      feeder(feeder),
      speed(speed),
      acceptableHeatThreshold(acceptableHeatThreshold),
      startingTotalBallCount(0),
      burstLength(burstLength) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void TestFeederCommand::initialize() { startingTotalBallCount = feeder->getTotalLimitCount(); }

void TestFeederCommand::execute() {
    int temp;
    if (drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP) {
        feeder->setTargetRPM(feeder_speed_array[2]);

    } else if (drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID) {
        feeder->setTargetRPM(feeder_speed_array[1]);

    } else {
        feeder->setTargetRPM(feeder_speed_array[0]);
    }
}

void TestFeederCommand::end(bool) { feeder->setTargetRPM(0); }

bool TestFeederCommand::isReady() { return feeder->isBarrelHeatAcceptable(acceptableHeatThreshold); }

bool TestFeederCommand::isFinished() const {
    int elapsedTotal = feeder->getTotalLimitCount() - startingTotalBallCount;
    return (elapsedTotal >= burstLength) || !feeder->isBarrelHeatAcceptable(acceptableHeatThreshold);
}

}  // namespace src::Feeder