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

void PosTesterSubsystem::initialize() {
    motor.initialize();
    motor.setDesiredOutput(60);
}

float current_position_display = 0.0f;
float wrapped_display = 0.0f;

void PosTesterSubsystem::refresh() {
    setDesiredOutput();
    wrapped_display = motor.getEncoderWrapped();
    current_position_display = getCurrentPos();
}

float output_display = 0.0f;

void PosTesterSubsystem::updateMotorPositionPID() {
    float err = targetPos - getCurrentPos();
    posPID.runControllerDerivateError(err);
    desiredOutput = posPID.getOutput();
    output_display = desiredOutput;
}

void PosTesterSubsystem::setDesiredOutput() {
    motor.setDesiredOutput(static_cast<int32_t>(5000));
}

void PosTesterSubsystem::setTargetPosition(float targetPos) {
    this->targetPos = targetPos;
}

};