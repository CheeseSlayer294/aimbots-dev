#include "subsystems/vel_tester/vel_tester.hpp"

namespace src::vel_tester {
Velocity_Control::Velocity_Control(src::Drivers* drivers)
    : Subsystem(drivers),
      targetRPM(0),
      velPID(vel_PID_CONFIG),
      desiredOutput(0),
      testMotor(drivers, vel_ID, vel_BUS, vel_DIRECTION, "vel_tester Motor") {}

bool is_tester_CommandRunningDisplay = false;

void Velocity_Control::initialize() { 
    testMotor.initialize(); 
}

void Velocity_Control::refresh() {
    setDesiredOutput();
}


void Velocity_Control::updateMotorVelocityPID() {
    float err = targetRPM - getCurrentRPM();
    velPID.runControllerDerivateError(err);
    desiredOutput = velPID.getOutput();
}

void Velocity_Control::setDesiredOutput() { 
    testMotor.setDesiredOutput(static_cast<int32_t>(desiredOutput)); 
}

float Velocity_Control::setTargetRPM(float rpm) {
    this->targetRPM = rpm;
}
}  // namespace src::vel_tester
