// #ifdef ENGINEER

#include "subsystems/grabber/open_grabber_command.hpp"

namespace src::Grabber {
OpenGrabberCommand::OpenGrabberCommand(src::Drivers* drivers, GrabberSubsystem* grabber) {
    this->drivers = drivers;
    this->grabber = grabber;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(grabber));
}

void OpenGrabberCommand::initialize() {
    grabber->setGrabberAngle(GRABBER_CLOSED_ANGLE);
}

void OpenGrabberCommand::execute() {
}

void OpenGrabberCommand::end(bool) {
}

bool OpenGrabberCommand::isReady() {
    return true;
}

bool OpenGrabberCommand::isFinished() const {
    return grabber->isGrabberReady();
}

};  // namespace src::grabber

// #endif