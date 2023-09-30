#include "subsystems/shooter/run_shooter_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {

RunShooterCommand::RunShooterCommand(
    src::Drivers* drivers,
    ShooterSubsystem* shooter,
    src::Utils::RefereeHelperTurreted* refHelper)
    : drivers(drivers),
      shooter(shooter),
      refHelper(refHelper) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void RunShooterCommand::initialize() {
    // No initialization needed
}

tap::communication::serial::RefSerialData::Rx::TurretData refSysRobotTurretDataDisplay;

uint16_t flywheelRPMDisplay = 0;
uint16_t flywheelCurrentRPMDisplay = 0;

bool commandIsRunning = false;

float currentHeatDisplay = 0.0f;
float heatLimitDisplay = 0.0f;

void RunShooterCommand::execute() {

    commandIsRunning = true;

    currentHeatDisplay = refHelper->getCurrBarrelHeat();
    heatLimitDisplay = refHelper->getCurrBarrelLimit();

    

    // defaults to slowest usable speed for robot
    uint16_t flywheelRPM = SHOOTER_SPEED_MATRIX[0][1];
    uint16_t refSpeedLimit = refHelper->getCurrBarrelProjectileSpeedLimit().value_or(SHOOTER_SPEED_MATRIX[0][0]);

    flywheelRPMDisplay = flywheelRPM;
    flywheelCurrentRPMDisplay = shooter->getMotorSpeed(src::Shooter::MotorIndex::LEFT);

    for (int i = 0; i < SHOOTER_SPEED_MATRIX.getNumberOfRows(); i++) {
        if (SHOOTER_SPEED_MATRIX[i][0] == refSpeedLimit) {
            flywheelRPM = SHOOTER_SPEED_MATRIX[i][1];
            break;
        }
    }

    if (drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID) {
        flywheelRPM = SHOOTER_SPEED_MATRIX[0][1];
    }
    else {
        flywheelRPM = SHOOTER_SPEED_MATRIX[1][1];
    }

    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, static_cast<float>(flywheelRPM));
    shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void RunShooterCommand::end(bool) {
    // No cleanup needed
    commandIsRunning = false;
}

bool RunShooterCommand::isReady() { return true; }

bool RunShooterCommand::isFinished() const { return false; }
}  // namespace src::Shooter

#endif //#ifdef SHOOTER_COMPATIBLE