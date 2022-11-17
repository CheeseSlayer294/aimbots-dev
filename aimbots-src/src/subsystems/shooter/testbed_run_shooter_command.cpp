#include "subsystems/shooter/testbed_run_shooter_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

#include "drivers.hpp"

//#ifndef TARGET_ENGINEER

// compile to standard specifically for 17mm shooter testbed

namespace src::Shooter {

TestbedRunShooterCommand::TestbedRunShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void TestbedRunShooterCommand::initialize() {
    // No initialization needed
}

// tap::communication::serial::RefSerialData::Rx::TurretData refSysRobotTurretDataDisplay;

void TestbedRunShooterCommand::execute() {
    using RefSerialRxData = tap::communication::serial::RefSerialData::Rx;

    // defaults to slowest usable speed for robot
    uint16_t flywheelRPMTop, flywheelRPMBottom;
    int temp = -1;
    if (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP) {
        temp = 2;

    } else if (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::MID) {
        temp = 1;
    } else if (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::DOWN) {
        temp = 0;
    }
    Matrix<uint16_t, 3, 3> CURRENT_SPEED_MATRIX(
        temp == 0 ? testbed_standard_speed_array : (temp == 1 ? testbed_backspin_speed_array : testbed_topspin_speed_array));
    flywheelRPMTop = CURRENT_SPEED_MATRIX[0][1];
    flywheelRPMBottom = CURRENT_SPEED_MATRIX[0][2];

    float analogY = shooter->getRightYJoystick(drivers);

    if (analogY > 0.3) {z
        flywheelRPMTop = CURRENT_SPEED_MATRIX[2][1];
        flywheelRPMBottom = CURRENT_SPEED_MATRIX[2][2];
    } else if (analogY < -0.3) {
        flywheelRPMTop = CURRENT_SPEED_MATRIX[0][1];
        flywheelRPMBottom = CURRENT_SPEED_MATRIX[0][2];
    } else {
        flywheelRPMTop = CURRENT_SPEED_MATRIX[1][1];
        flywheelRPMBottom = CURRENT_SPEED_MATRIX[1][2];
    }

    // shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, static_cast<float>(flywheelRPM));
    shooter->setTargetRPM(MotorIndex::RIGHT, flywheelRPMTop);
    shooter->setTargetRPM(MotorIndex::LEFT, flywheelRPMBottom);
    shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void TestbedRunShooterCommand::end(bool) {
    // No cleanup needed
}

bool TestbedRunShooterCommand::isReady() { return true; }

bool TestbedRunShooterCommand::isFinished() const { return false; }
}  // namespace src::Shooter