// #ifdef ENGINEER

#include "subsystems/grabber/close_grabber_command.hpp"

namespace src::Grabber {

CloseGrabberCommand::CloseGrabberCommand(src::Drivers* drivers, GrabberSubsystem* grabber) {
    this->drivers = drivers;
    this->grabber = grabber;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(grabber));
}

void CloseGrabberCommand::initialize() {
    grabber->setGrabberAngle(GRABBER_CLOSED_ANGLE);
}

void CloseGrabberCommand::execute() {
}

void CloseGrabberCommand::end(bool) {
}

bool CloseGrabberCommand::isReady() {
    return true;
}

bool CloseGrabberCommand::isFinished() const {
    return grabber->isGrabberReady();
}

};  // namespace src::grabber
// #endif