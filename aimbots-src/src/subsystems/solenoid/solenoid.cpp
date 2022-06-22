#include "subsystems/solenoid/solenoid.hpp"

// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{

SolenoidSubsytem::SolenoidSubsytem(tap::Drivers* driver)  :
    Subsystem(drivers) {};

void SolenoidSubsytem::initialize() {
    C1::configure(modm::platform::Gpio::InputType::PullDown);
    C2::configure(modm::platform::Gpio::InputType::PullDown);
    C3::configure(modm::platform::Gpio::InputType::PullDown);

}

void SolenoidSubsytem::refresh() {
    // updateSolenoid();
}

void SolenoidSubsytem::solenoidWrite(bool value, std::string pin) {
    if (pin == "C1") {
        C1::set(value);
    } else if (pin == "C2") {
        C2::set(value);
    } else if (pin == "C3") {
        C3::set(value);
    }
    // C1::set(value);
}

void SolenoidSubsytem::solenoidRead(){

}

}
// #endif
