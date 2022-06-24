#include "subsystems/solenoid/solenoid.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

 namespace src::Solenoid{
    Solenoid::Solenoid(
    Drivers *drivers,
    tap::gpio::Pwm::Pin pwmPin,
    float maximumPwm,
    float minimumPwm,
    float pwmRampSpeed)
    : drivers(drivers),
      pwmOutputRamp(0.0f),
      maxPwm(tap::algorithms::limitVal<float>(maximumPwm, 0.0f, 1.0f)),
      minPwm(tap::algorithms::limitVal<float>(minimumPwm, 0.0f, 1.0f)),
      pwmRampSpeed(pwmRampSpeed),
      prevTime(0),
      solenoidPin(pwmPin)
{
    if (maxPwm < minPwm)
    {
        minPwm = 0.0f;
        maxPwm = 1.0f;
        RAISE_ERROR(drivers, "min Solenoid PWM > max Solenoid PWM");
    }
}

void Solenoid::setTargetPwm(float pwm)
{
    pwmOutputRamp.setTarget(tap::algorithms::limitVal<float>(pwm, minPwm, maxPwm));
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void Solenoid::updateSendPwmRamp()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    pwmOutputRamp.update(pwmRampSpeed * (currTime - prevTime));
    prevTime = currTime;
    currentPwm = pwmOutputRamp.getValue();
    drivers->pwm.write(pwmOutputRamp.getValue(), solenoidPin);
}

float Solenoid::getPWM() const { return currentPwm; }

float Solenoid::getMinPWM() const { return minPwm; }

float Solenoid::getMaxPWM() const { return maxPwm; }

bool Solenoid::isRampTargetMet() const { return pwmOutputRamp.isTargetReached();
    
 }

 }; // namespace src::Solenoid
