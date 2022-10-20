#include "run_feeder_command.hpp"
#ifndef TARGET_DART
namespace src::Feeder {
RunFeederCommand::RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder)
    : drivers(drivers), feeder(feeder), speed(0) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void RunFeederCommand::initialize() {
    speed = 0.0f;
    feeder->setTargetRPM(speed);
}

void RunFeederCommand::execute() {
    // drivers->leds.set(tap::gpio::Leds::A, true);
    // drivers->leds.set(tap::gpio::Leds::B, false);
    // drivers->leds.set(tap::gpio::Leds::C, true);
    // drivers->leds.set(tap::gpio::Leds::D, false);
    // drivers->leds.set(tap::gpio::Leds::E, true);
    // drivers->leds.set(tap::gpio::Leds::F, false);
    // drivers->leds.set(tap::gpio::Leds::G, true);
    // drivers->leds.set(tap::gpio::Leds::H, false);
    speed = 3000.0f * FEEDER_MOTOR_DIRECTION;
    feeder->setTargetRPM(speed);
}

void RunFeederCommand::end(bool) {}

bool RunFeederCommand::isReady() {
    return true;
}

bool RunFeederCommand::isFinished() const {
    return false;  // finished condition (button released) or their api is nice and we don't have to
}
}  // namespace src::Feeder
#endif