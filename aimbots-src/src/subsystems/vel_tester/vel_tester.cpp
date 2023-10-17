#include "subsystems/vel_tester/vel_tester.hpp"

namespace src::Vel_Tester {

Vel_TesterSubsystem::Vel_TesterSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      targetRPM(0),
      desiredOutput(0),
      velTesterPID(FEEDER_VELOCITY_PID_CONFIG), //we have this reference right??
      vel_testerMotor(drivers, FEEDER_ID, FEED_BUS, FEEDER_DIRECTION, "Feeder Motor")

{
}


void Vel_TesterSubsystem::initialize() {
    vel_testerMotor.initialize();
}


void Vel_TesterSubsystem::refresh() {
    updateMotorVelocityPID();
    setDesiredOutput();
}

void Vel_TesterSubsystem::updateMotorVelocityPID() {
    float err = targetRPM - vel_testerMotor.getShaftRPM();
    vel_TestVelPID.runControllerDerivateError(err);
    desiredOutput = vel_TestVelPID.getOutput();
}

float Vel_TesterSubsystem::setTargetRPM(float rpm) {
    this->targetRPM = rpm;
    return targetRPM;
}

void Vel_TesterSubsystem::setDesiredOutput(){
    vel_testerMotor.setDesiredOutput(static_cast<int32_t>(DesiredOutput));
}



}