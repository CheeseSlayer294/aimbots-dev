#include "subsystems/shooter/run_shooter_command.hpp"

#include "drivers.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

//#ifndef TARGET_ENGINEER

namespace src::Shooter {

RunShooterCommand::RunShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void RunShooterCommand::initialize() {
    // No initialization needed
}

float rpmTarget = 0.0f;

void RunShooterCommand::execute() {
    switch (drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH)) {
        case Remote::SwitchState::DOWN:
            rpmTarget = 7500.0f;
            break;

        case Remote::SwitchState::MID:
            rpmTarget = 8750.0f;
            break;

        case Remote::SwitchState::UP:
            rpmTarget = 10000.0f;
            break;

        default:
            rpmTarget = 150.0f;
            break;
    }

    // if (drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) < -0.9f) {
    //     rpmTarget =
    // }

    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, rpmTarget);

    shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void RunShooterCommand::end(bool) {
    // No cleanup needed
}

bool RunShooterCommand::isReady() {
    return true;
}

bool RunShooterCommand::isFinished() const {
    return false;
}
}  // namespace src::Shooter