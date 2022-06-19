#include "subsystems/solenoid/solenoid.hpp"

#if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{

SolenoidSubsytem::SolenoidSubsytem(tap::Drivers* driver)  :
    Subsystem(drivers) {};

void SolenoidSubsytem::initialize() {
    // C6::configure(modm::platform::Gpio::InputType::PullDown);
    // C7::configure(modm::platform::Gpio::InputType::PullDown);
}

void SolenoidSubsytem::refresh() {
    updateSolenoid();
}

void SolenoidSubsytem::updateSolenoid(){

}

void SolenoidSubsytem::readSolenoid(){

}

}
#endif
