#include "set_pos_throttle_command.hpp"

namespace src::PosTester {

SetPosThrottleCommand::SetPosThrottleCommand(Drivers* drivers, PosTesterSubsystem* posTester)
    : drivers(drivers), posTester(posTester)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(posTester));
}

void SetPosThrottleCommand::initialize() {}

void SetPosThrottleCommand::execute() {
    // [-1, 1]
    float x = drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
    float y = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);

    float angle = atan2f(y, x);
    
    // map [-pi, pi] to [0, 2pi]
    if (angle < 0)
        angle += 2*M_PI;

    posTester->setTargetPosition(angle);
    posTester->updateMotorPositionPID();
}

void SetPosThrottleCommand::end(bool interrupted) {}

bool SetPosThrottleCommand::isReady() { return true; }

bool SetPosThrottleCommand::isFinished() const { return false; }

};