#include "set_pos_command.hpp"

namespace src::PosTester {

SetPosCommand::SetPosCommand(Drivers* drivers, PosTesterSubsystem* posTester, float angle)
    : drivers(drivers), posTester(posTester), angle(angle)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(posTester));
}

void SetPosCommand::initialize() {}

void SetPosCommand::execute() {
    float x = drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
    float y = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);

    posTester->setTargetPosition(atan2f(x, y));
    posTester->updateMotorPositionPID();
}

void SetPosCommand::end(bool interrupted) {}

bool SetPosCommand::isReady() { return true; }

bool SetPosCommand::isFinished() const { return false; }

}