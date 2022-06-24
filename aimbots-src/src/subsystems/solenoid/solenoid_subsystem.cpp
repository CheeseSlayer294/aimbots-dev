#include "subsystems/solenoid/solenoid_subsystem.hpp"
#include "robots/engineer/swerve_engineer_constants.hpp"

// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)
#define REMAP_SOLENOID(x) (REMAP(x, SOLENOID_MIN_ANGLE, SOLENOID_MAX_ANGLE, SOLENOID_MIN_PWM, SOLENOID_MAX_PWM))


namespace src::Solenoid{

SolenoidSubsytem::SolenoidSubsytem(src::Drivers* driver)  :
    Subsystem(drivers),
    drivers(driver),
    solenoidHorizontal(drivers, SOLENOID_HORIZONTAL_PIN, SOLENOID_MAX_PWM, SOLENOID_MIN_PWM, SOLENOID_PWM_RAMP_SPEED),
    solenoidGrabber(drivers, SOLENOID_GRABBER_PIN, SOLENOID_MAX_PWM, SOLENOID_MIN_PWM, SOLENOID_PWM_RAMP_SPEED)
    {}

void SolenoidSubsytem::initialize() {
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER1, 330);
}

void SolenoidSubsytem::refresh() {
    solenoidGrabber.updateSendPwmRamp();
    solenoidHorizontal.updateSendPwmRamp();
}

bool SolenoidSubsytem::isSolenoidReady(){
    return (solenoidGrabber.isRampTargetMet() && (tap::arch::clock::getTimeMilliseconds() - actionStartTime) > SOLENOID_MIN_ACTION_DELAY) &&
    (solenoidHorizontal.isRampTargetMet() && (tap::arch::clock::getTimeMilliseconds() - actionStartTime) > SOLENOID_MIN_ACTION_DELAY);
}

void SolenoidSubsytem::setSolenoidPostion(float desiredAngle, std::string solenoidName){    
    desiredAngle = tap::algorithms::limitVal<float>(desiredAngle, SOLENOID_MIN_ANGLE, SOLENOID_MAX_ANGLE);  // Limit inputs to min/max of motor
    // hopperAngleSetDisplay = desiredAngle;
    if(solenoidName == "horizontal"){
        solenoidHorizontal.setTargetPwm(REMAP_SOLENOID(desiredAngle));
    }else if(solenoidName == "grabber"){
        solenoidGrabber.setTargetPwm(REMAP_SOLENOID(desiredAngle));
    }
    actionStartTime = tap::arch::clock::getTimeMilliseconds();
}

void SolenoidSubsytem::setSolenoidState(uint8_t state, std::string solenoidName){
     if(solenoidName == "horizontal"){
        solenoidHorizontalState = state;
    }else if(solenoidName == "grabber"){
        solenoidGrabberState = state;
    }
}

uint8_t SolenoidSubsytem::getSolenoidState(std::string solenoidName) const{
    if(solenoidName == "horizontal"){
        return solenoidHorizontalState;
    }else if(solenoidName == "grabber"){
        return solenoidGrabberState;
    }
}


}; // namespace src::Solenoid
// #endif