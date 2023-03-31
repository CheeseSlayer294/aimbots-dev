#include "subsystems/shooter/run_shooter_command.hpp"
#ifndef ENGINEER

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

#include "drivers.hpp"

// #ifndef TARGET_ENGINEER

namespace src::Shooter {

RunShooterCommand::RunShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void RunShooterCommand::initialize() {
    // No initialization needed
}

tap::communication::serial::RefSerialData::Rx::TurretData refSysRobotTurretDataDisplay;

void RunShooterCommand::execute() {
    // defaults to slowest usable speed for robot
    uint16_t flywheelRPM = SHOOTER_SPEED_MATRIX[0][1];

#if defined(TARGET_STANDARD) || defined(TARGET_SENTRY)
    using RefSerialRxData = tap::communication::serial::RefSerialData::Rx;
    uint16_t refSpeedLimit = 0;
    auto refSysRobotTurretData = drivers->refSerial.getRobotData().turret;
    refSysRobotTurretDataDisplay = refSysRobotTurretData;

    auto launcherID = refSysRobotTurretData.launchMechanismID;
    switch (launcherID) {  // gets launcher ID from ref serial, sets speed limit accordingly
                           // #if defined(TARGET_STANDARD) || defined(TARGET_SENTRY)
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            refSpeedLimit = refSysRobotTurretData.barrelSpeedLimit17ID1;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            refSpeedLimit = refSysRobotTurretData.barrelSpeedLimit17ID2;
            break;
        }

        case RefSerialRxData::MechanismID::TURRET_42MM: {
            refSpeedLimit = refSysRobotTurretData.barrelSpeedLimit42;
            break;
        }
        default:
            break;
    }

    for (int i = 0; i < SHOOTER_SPEED_MATRIX.getNumberOfRows(); i++) {
        if (SHOOTER_SPEED_MATRIX[i][0] == refSpeedLimit) {
            flywheelRPM = SHOOTER_SPEED_MATRIX[i][1];
            break;
        }
    }
#endif

    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, static_cast<float>(flywheelRPM));
    shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void RunShooterCommand::end(bool) {
    // No cleanup needed
}

bool RunShooterCommand::isReady() { return true; }

bool RunShooterCommand::isFinished() const { return false; }
}  // namespace src::Shooter

#endif
