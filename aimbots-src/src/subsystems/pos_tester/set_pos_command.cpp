#include "set_pos_command.hpp"

namespace src::PosTester {

SetPosCommand::SetPosCommand(Drivers* drivers, PosTesterSubsystem* posTester)
    : drivers(drivers), posTester(posTester)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(posTester));
}

void SetPosCommand::initialize() {}

float angle = 90;

void SetPosCommand::execute() {
    posTester->setTargetPosition(angle);
    posTester->updateMotorPositionPID();
}

void SetPosCommand::end(bool interrupted) {}

bool SetPosCommand::isReady() {}

bool SetPosCommand::isFinished() const {}

}