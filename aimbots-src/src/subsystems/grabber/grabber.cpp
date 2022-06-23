// #ifdef ENGINEER

#include "subsystems/grabber/grabber.hpp"

#include "drivers.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "utils/common_types.hpp"

#define REMAP_GRABBER(x) (REMAP(x, GRABBER_MIN_ANGLE, GRABBER_MAX_ANGLE, GRABBER_MIN_PWM, GRABBER_MAX_PWM))

namespace src::Grabber {

GrabberSubsystem::GrabberSubsystem(tap::Drivers* drivers) : Subsystem(drivers),
                                                          drivers(drivers),
                                                          grabberMotorRight(drivers, GRABBER_RIGHT_PIN, GRABBER_MAX_PWM, GRABBER_MIN_PWM, GRABBER_PWM_RAMP_SPEED),
                                                          grabberMotorLeft(drivers, GRABBER_LEFT_PIN, GRABBER_MAX_PWM, GRABBER_MIN_PWM, GRABBER_PWM_RAMP_SPEED),
                                                          grabber_state(2) {}

void GrabberSubsystem::initialize() {
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 330);  // Timer 1 for C1 Pin
}

float testVal = 0.85f;

void GrabberSubsystem::refresh() {
    grabberMotorLeft.updateSendPwmRamp();
    grabberMotorRight.updateSendPwmRamp();
}

float grabberAngleSetDisplay = 0.0f;

void GrabberSubsystem::setGrabberAngle(float desiredAngle) {
    desiredAngle = tap::algorithms::limitVal<float>(desiredAngle, GRABBER_MIN_ANGLE, GRABBER_MAX_ANGLE);  // Limit inputs to min/max of motor
    grabberAngleSetDisplay = desiredAngle;
    grabberMotorLeft.setTargetPwm(REMAP_GRABBER(desiredAngle));
    grabberMotorRight.setTargetPwm(REMAP_GRABBER(-desiredAngle));
    actionStartTime = tap::arch::clock::getTimeMilliseconds();
}

bool GrabberSubsystem::isGrabberReady() const {
    return (grabberMotorRight.isRampTargetMet() && (tap::arch::clock::getTimeMilliseconds() - actionStartTime) > GRABBER_MIN_ACTION_DELAY) && (grabberMotorLeft.isRampTargetMet() && (tap::arch::clock::getTimeMilliseconds() - actionStartTime) > GRABBER_MIN_ACTION_DELAY);
    // return true;
    // the delay is mostly just to keep commands from ending b4 they should, bc isRampTargetMet() is based on pwm ramp finishing
}

uint8_t GrabberSubsystem::getGrabberState() const {
    return grabber_state;
}

void GrabberSubsystem::setGrabberState(uint8_t new_state) {
    grabber_state = new_state;
}
};  // namespace src::grabber

// #endif