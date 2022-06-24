#include "utils/robot_specific_inc.hpp"
#ifdef TOKYO_COMPATIBLE

#include <subsystems/chassis/chassis_rel_drive.hpp>

#include "chassis_tokyo_command.hpp"
#warning "tokyo compatible"

namespace src::Chassis {

ChassisTokyoCommand::ChassisTokyoCommand(src::Drivers* drivers, ChassisSubsystem* chassis, src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisTokyoCommand::initialize() {
    // picks a random direction to begin rotation
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? 1 : -1;
    rotationSpeedRamp.reset(chassis->getDesiredRotation());
}

float gimbalYawAngleDisplay = 0.0f;

void ChassisTokyoCommand::execute() {
    if (gimbal->isOnline()) {
        float gimbalYawAngle = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Radians);
        // this is wrapped between -PI and PI

        gimbalYawAngleDisplay = modm::toDegree(gimbalYawAngle);

        float x = 0.0f;
        float y = 0.0f;

        Movement::Independent::calculateUserDesiredMovement(drivers, chassis, &x, &y, 0.0f);

        x *= TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER;
        y *= TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER;

        const float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        const float translationalSpeedThreshold = maxWheelSpeed * TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER * TOKYO_TRANSLATION_THRESHOLD_TO_DECREASE_ROTATION_SPEED;

        float rampTarget = maxWheelSpeed * rotationDirection * TOKYO_ROTATIONAL_SPEED_FRACTION_OF_MAX;

        // reduces rotation speed when translation speed is high
        if (fabsf(x) > translationalSpeedThreshold || fabsf(y) > translationalSpeedThreshold) {
            rampTarget *= TOKYO_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING;
        }

        rotationSpeedRamp.setTarget(rampTarget);
        rotationSpeedRamp.update(TOKYO_ROTATIONAL_SPEED_INCREMENT);

        float r = rotationSpeedRamp.getValue();

        rotateVector(&x, &y, -gimbalYawAngle);

        chassis->setTargetRPMs(x, y, r);
    } else {
        Movement::Independent::onExecute(drivers, chassis);
    }
}

void ChassisTokyoCommand::end(bool) {
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisTokyoCommand::isReady() {
    return true;
}

bool ChassisTokyoCommand::isFinished() const {
    return false;
}

}  // namespace src::Chassis

#endif