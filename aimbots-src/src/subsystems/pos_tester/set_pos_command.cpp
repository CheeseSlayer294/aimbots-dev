#include "set_pos_command.hpp"

namespace src::PosTester {

SetPosCommand::SetPosCommand(Drivers* drivers, PosTesterSubsystem* posTester)
    : drivers(drivers), posTester(posTester)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(posTester));
}

bool initialized_display = false;

void SetPosCommand::initialize() {
    initialized_display = true;
}

float angle = 90;
bool isCommandRunning_display = false;

void SetPosCommand::execute() {
    isCommandRunning_display = true;
    posTester->setTargetPosition(angle);
    posTester->updateMotorPositionPID();
}

void SetPosCommand::end(bool interrupted) {
    isCommandRunning_display = false;
}

bool SetPosCommand::isReady() { return true; }

bool SetPosCommand::isFinished() const { return false; }

}