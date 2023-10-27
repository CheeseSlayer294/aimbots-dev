#include "set_pos_command.hpp"

namespace src::PosTester {

SetPosCommand::SetPosCommand(Drivers* drivers, PosTesterSubsystem* posTester, float angle)
    : drivers(drivers), posTester(posTester), angle(angle)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(posTester));
}

void SetPosCommand::initialize() {}

void SetPosCommand::execute() {
    posTester->setTargetPosition(angle);
    posTester->updateMotorPositionPID();
}

void SetPosCommand::end(bool interrupted) {}

bool SetPosCommand::isReady() { return true; }

bool SetPosCommand::isFinished() const { return false; }

}