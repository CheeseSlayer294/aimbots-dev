#include "subsystems/solenoid/solenoid.hpp"

// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{

SolenoidSubsytem::SolenoidSubsytem(src::Drivers* driver)  :
    Subsystem(drivers) {};

void SolenoidSubsytem::initialize() {
    C1::configure(modm::platform::Gpio::InputType::PullDown);
    C2::configure(modm::platform::Gpio::InputType::PullDown);
    C3::configure(modm::platform::Gpio::InputType::PullDown);
    C1State = false;
    C2State = false;
    C3State = false;

}

void SolenoidSubsytem::refresh() {
    solenoidWrite(C1State, "C1");
    solenoidWrite(C2State, "C2");
    solenoidWrite(C3State, "C3");
}

void SolenoidSubsytem::solenoidWrite(bool value, std::string pin) {
    if (pin == "C1") {
        C1::set(value);
        C1State = value;
    } else if (pin == "C2") {
        C2::set(value);
        C2State = value;
    } else if (pin == "C3") {
        C3::set(value);
        C3State = value;
    }
    // C1::set(value);
}

void SolenoidSubsytem::solenoidRead(){

}

}
// #endif
