#include "subsystems/vel_tester/vel_tester.hpp"

namespace src::Vel_Tester {

 Vel_TesterSubsystem::Vel_TesterSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
      targetRPM(0),
      desiredOutput(0),
      velTesterPID(TESTBENCH_VELOCITY_PID_CONFIG), //we have this reference right??
      vel_testerMotor(drivers, VEL_MOTOR_ID, VEL_BUS, VEL_MOTOR_DIRECTION, "Vel Motor")

{
    BuildVelTestMotor();
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
    velTesterPID.runControllerDerivateError(err);
    desiredOutput = velTesterPID.getOutput();
}

void Vel_TesterSubsystem::setTargetRPM(float rpm) {
    this->targetRPM = rpm;
}

void Vel_TesterSubsystem::setDesiredOutput(){
    vel_testerMotor.setDesiredOutput(static_cast<int32_t>(desiredOutput));
}



}