#include "pos_tester.hpp"

namespace src::PosTester {

PosTesterSubsystem::PosTesterSubsystem(Drivers* drivers)
    : Subsystem(drivers),
    targetPos(0),
    desiredOutput(0),
    motor(drivers, POS_MOTOR_ID, POS_MOTOR_BUS, POS_MOTOR_DIRECTION, "Position Motor"),
    posPID(EMTESTBENCH_POSITION_PID_CONFIG)
{
}

void PosTesterSubsystem::initialize() {}

void PosTesterSubsystem::refresh() {
    setDesiredOutput();
}

void PosTesterSubsystem::updateMotorPositionPID() {
    float err = targetPos - getCurrentPos();
    posPID.runControllerDerivateError(err);
    desiredOutput = posPID.getOutput();
}

void PosTesterSubsystem::setDesiredOutput() {
    motor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
}

void PosTesterSubsystem::setTargetPosition(float targetPos) {
    this->targetPos = targetPos;
}

};