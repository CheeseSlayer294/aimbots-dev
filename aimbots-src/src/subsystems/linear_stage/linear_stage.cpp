#include "subsystems/linear_stage/linear_stage.hpp"

#ifndef ENGINEER

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "utils/common_types.hpp"

namespace src::LinearStage {

LinearStageSubsystem::ShooterSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      x_motor(drivers, LINEAR_STAGE_X_MOTOR_ID, STAGE_BUS, X_STAGE_DIRECTION, "X-Motor"),
      y_motor(drivers, LINEAR_STAGE_Y_MOTOR_ID, STAGE_BUS, Y_STAGE_DIRECTION, "Y-Motor"),
      z_motor(drivers, LINEAR_STAGE_Z_MOTOR_ID, STAGE_BUS, Z_STAGE_DIRECTION, "Z-Motor"),
      x_motorPID(STAGE_VELOCITY_PID_CONFIG),  // FIX
      y_motorPID(STAGE_VELOCITY_PID_CONFIG),
      z_motorPID(STAGE_VELOCITY_PID_CONFIG),

      targetRPMs(Matrix<float, STAGE_MOTOR_COUNT, 1>::zeroMatrix()),
      desiredOutputs(Matrix<int32_t, STAGE_MOTOR_COUNT, 1>::zeroMatrix()),
      motors(Matrix<DJIMotor*, STAGE_MOTOR_COUNT, 1>::zeroMatrix()),
      positionPIDs(Matrix<SmoothPID*, STAGE_MOTOR_COUNT, 1>::zeroMatrix())
//
{
    motors[X][0] = &x_motor;  // X
    motors[Y][0] = &y_motor;  // Y
    motors[Z][0] = &z_motor;  // Z
    positionPIDs[X][0] = &x_motorPID;
    positionPIDs[Y][0] = &y_motorPID;
    positionPIDs[Z][0] = &z_motorPID;
}

void LinearStageSubsystem::initialize() {
    ForAllStageMotors(&DJIMotor::initialize);

    ForAllStageMotors(&DJIMotor::setDesiredOutput, static_cast<int32_t>(0.0f));
}

float PIDoutDisplay = 0.0f;
float shaftSpeedDisplay = 0.0f;

float xOut = 0.0f;  // RIGHT / TOP_RIGHT
float yOut = 0.0f;  // LEFT / BOT_LEFT
float xOut = 0.0f;

// Update the actual RPMs of the motors; the calculation is called from ShooterCommand
// WARCRIMES IN PROGRESS
void LinearStageSubsystem::refresh() {
    // Debug info
    if (x_motor.isMotorOnline()) {
        xOut = x_motor.getEncoderUnwrapped();
    }
    if (y_motor.isMotorOnline()) {
        yOut = y_motor.getEncoderUnwrapped();
    }
    if (z_motor.isMotorOnline()) {
        zOut = z_motor.getEncoderUnwrapped();
    }

    ForAllStageMotors(&LinearStageSubsystem::setDesiredOutputToMotor);
}

float LinearStageSubsystem::getMotorPosition(MotorIndex motorIdx) const {
    if (motors[motorIdx][0]->isMotorOnline()) {
        return motors[motorIdx][0]->getEncoderUnwrapped();
    }
    return 0.0f;
}

void LinearStageSubsystem::updateMotorPositionPID(MotorIndex motorIdx) {
    if (motors[motorIdx][0]->isMotorOnline()) {  // Check if motor is online when getting info from it
        float err = targetRPMs[motorIdx][0] - motors[motorIdx][0]->getEncoderUnwrapped();
        float PIDOut = positionPIDs[motorIdx][0]->runControllerDerivateError(err);
        setDesiredOutput(motorIdx, PIDOut);
    }
}

void LinearStageSubsystem::setTargetRPM(MotorIndex motorIdx, float targetRPM) { targetRPMs[motorIdx][0] = targetRPM; }

float powerDisplay = 0.0f;

void LinearStageSubsystem::setDesiredOutput(MotorIndex motorIdx, float desiredOutput) {
    desiredOutputs[motorIdx][0] = static_cast<int32_t>(desiredOutput);
}

void LinearStageSubsystem::setDesiredOutputToMotor(MotorIndex motorIdx) {
    if (motors[motorIdx][0]->isMotorOnline()) {  // Check if motor is online when setting to it
        motors[motorIdx][0]->setDesiredOutput(desiredOutputs[motorIdx][0]);
    }
}
};  // namespace src::LinearStage

#endif