#pragma once

#include  "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "utils/common_types.hpp"
#include <string>

// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)


namespace src::Solenoid{

class Solenoid{
public:
Solenoid(
        Drivers *drivers,
        tap::gpio::Pwm::Pin currpwmPinPort,
        float maximumPwm,
        float minimumPwm,
        float pwmRampSpeed);

    /**
     * Limits `pwmOutputRamp` to `minPwm` and `maxPwm`, then sets ramp output
     * to the limited value. Do not repeatedly call (i.e. only call in a `Command`'s
     * `initialize` function, for example).
     */
    void setTargetPwm(float PWM);

    /**
     * Updates the `pwmOutputRamp` object and then sets the output PWM to the updated
     * ramp value.
     */
    void updateSendPwmRamp();

    /**
     * @return The current PWM output to the servo.
     */
    float getPWM() const;

    /**
     * @return The minimum PWM output (as a duty cycle).
     */
    float getMinPWM() const;

    /**
     * @return The maximum PWM output (as a duty cycle).
     */
    float getMaxPWM() const;

    /**
     * @return `true` if the ramp has met the desired PWM value (set with `setTargetPwm`).
     *      Use this to estimate when a servo movement is complete.
     */
    bool isRampTargetMet() const;

private:
    Drivers *drivers;

    /// Used to change servo speed. See construtctor for detail.
    tap::algorithms::Ramp pwmOutputRamp;

    /// The max PWM the servo can handle.
    float maxPwm;

    /// The min PWM the servo can handle.
    float minPwm;

    /// Current PWM output.
    float currentPwm;

    /// Desired speed of the ramp in PWM / ms
    float pwmRampSpeed;

    /// Used to calculate the ramp dt.
    uint32_t prevTime = 0;

    /// The PWM pin that the solenoid is attached to.
    tap::gpio::Pwm::Pin solenoidPin;
};  // class Solenoid

};//namespace src::Solonoid

// #endif
