// #ifdef ENGINEER

#include "subsystems/grabber/toggle_grabber_command.hpp"

namespace src::Grabber {
ToggleGrabberCommand::ToggleGrabberCommand(src::Drivers* drivers, GrabberSubsystem* grabber) {
    this->drivers = drivers;
    this->grabber = grabber;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(grabber));
}

void ToggleGrabberCommand::initialize() {
    uint8_t state = grabber->getGrabberState();
    if (state == UNKNOWN) {  // default 'unknown' action can be open/closed
        grabber->setGrabberAngle(GRABBER_CLOSED_ANGLE);
        grabber->setGrabberState(CLOSED);
    } else {
        grabber->setGrabberAngle(state ? GRABBER_CLOSED_ANGLE : GRABBER_OPEN_ANGLE);
        grabber->setGrabberState(!state);
    }
}

void ToggleGrabberCommand::execute() {
}

void ToggleGrabberCommand::end(bool) {
}

bool ToggleGrabberCommand::isReady() {
    return true;
}

bool ToggleGrabberCommand::isFinished() const {
    return grabber->isGrabberReady();
}
};  // namespace src::Grabber

// #endif