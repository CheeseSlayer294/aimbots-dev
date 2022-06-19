#include "subsystems/solonoid/solonoid.hpp"

#if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solonoid{

SolonoidSubsytem::SolonoidSubsytem(tap::Drivers* driver)  :
    Subsystem(drivers) {};

void SolonoidSubsytem::initialize() {
    // C6::configure(modm::platform::Gpio::InputType::PullDown);
    // C7::configure(modm::platform::Gpio::InputType::PullDown);
}

void SolonoidSubsytem::refresh() {
    updateSolonoid();
}

void SolonoidSubsytem::updateSolonoid(){

}

void SolonoidSubsytem::readSolonoid(){

}

}
#endif