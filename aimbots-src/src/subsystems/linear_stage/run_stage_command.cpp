#include "subsystems/linear_stage/run_stage_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp" a

namespace src::LinearStage {

RunStageCommand::RunStageCommand(
    src::Drivers* drivers,
    LinearStageSubsystem* linear_stage,
    src::Utils::RefereeHelper* refHelper)
    : drivers(drivers),
      linear_stage(linear_stage),
      refHelper(refHelper) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(linear_stage));
}

void RunStageCommand::initialize() {
    // No initialization needed
}

tap::communication::serial::RefSerialData::Rx::TurretData refSysRobotTurretDataDisplay;

uint16_t flywheelRPMDisplay = 0;
uint16_t flywheelCurrentRPMDisplay = 0;

void RunStageCommand::execute() {
    // defaults to slowest usable speed for robot
    uint16_t flywheelRPM = SHOOTER_SPEED_MATRIX[0][1];
    uint16_t refSpeedLimit = refHelper->getProjectileSpeedLimit();

    flywheelRPMDisplay = flywheelRPM;
    flywheelCurrentRPMDisplay = linear_stage->getMotorSpeed(src::LinearStageSubsystem::MotorIndex::LEFT);

    linear_stage->ForAllStageMotors(&LinearStageSubsystem::setTargetRPM, static_cast<float>(flywheelRPM));
    linear_stage->ForAllStageMotors(&LinearStageSubsystem::updateMotorPositionPID);
}

void RunStageCommand::end(bool) {
    // No cleanup needed
}

bool RunStageCommand::isReady() { return true; }

bool RunStageCommand::isFinished() const { return false; }
}  // namespace src::LinearStage
