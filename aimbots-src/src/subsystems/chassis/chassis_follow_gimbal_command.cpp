#include "utils/robot_specific_inc.hpp"
#ifdef TOKYO_COMPATIBLE

#include <subsystems/chassis/chassis_rel_drive.hpp>

#include "chassis_follow_gimbal_command.hpp"

#define RADPS_TO_RPM 9.549297f

namespace src::Chassis {

ChassisFollowGimbalCommand::ChassisFollowGimbalCommand(src::Drivers* drivers, ChassisSubsystem* chassis, src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      rotationController(ROTATION_POSITION_PID_CONFIG)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisFollowGimbalCommand::initialize() {
}

float gimbalYawFieldRelativeDisplay = 0.0f;
float gimbalYawAngleDisplay2 = 0.0f;
float chassisYawDisplay = 0.0f;
float rotationControllerOutputDisplay = 0.0f;

float rotationLimitedMaxTranslationalSpeedDisplay = 0.0f;

void ChassisFollowGimbalCommand::execute() {
    if (gimbal->isOnline()) {
        float gimbalYawAngle = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Radians);

        float rotationControllerError = gimbalYawAngle;
        gimbalYawAngleDisplay2 = modm::toDegree(gimbalYawAngle);

        rotationController.runController(rotationControllerError, (RADPS_TO_RPM * drivers->fieldRelativeInformant.getGz()));

        rotationControllerOutputDisplay = rotationController.getOutput();

        const float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        float rotationLimitedMaxTranslationalSpeed =
            maxWheelSpeed * chassis->calculateRotationTranslationalGain(rotationController.getOutput());

        rotationLimitedMaxTranslationalSpeedDisplay = rotationLimitedMaxTranslationalSpeed;

        float chassisXDesiredWheelspeed = limitVal(
            maxWheelSpeed * drivers->controlOperatorInterface.getChassisXInput(),
            -rotationLimitedMaxTranslationalSpeed,
            rotationLimitedMaxTranslationalSpeed);

        float chassisYDesiredWheelspeed = limitVal(
            maxWheelSpeed * drivers->controlOperatorInterface.getChassisYInput(),
            -rotationLimitedMaxTranslationalSpeed,
            rotationLimitedMaxTranslationalSpeed);

        rotateVector(&chassisXDesiredWheelspeed, &chassisYDesiredWheelspeed, -gimbalYawAngle);

        chassis->setTargetRPMs(chassisXDesiredWheelspeed, chassisYDesiredWheelspeed, rotationController.getOutput());
    } else {
        Movement::Independent::onExecute(drivers, chassis);
    }
}

void ChassisFollowGimbalCommand::end(bool) {
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisFollowGimbalCommand::isReady() {
    return true;
}

bool ChassisFollowGimbalCommand::isFinished() const {
    return false;
}

}  // namespace src::Chassis

#endif