#pragma once

#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/gpio/digital.hpp"
#include "utils/common_types.hpp"

namespace src {
class Drivers;
}

namespace utils {

class UltrasonicDistanceSensor {
   public:
    using LeftEchoPin = modm::platform::GpioC6;    // using port C5, echo for C5 is gpio_C6
    using RightEchoPin = modm::platform::GpioE13;  // using port C3, echo for C3 is gpio_E13

    static void handleLeftEchoEnd();
    static void handleRightEchoEnd();

    UltrasonicDistanceSensor(src::Drivers* drivers);

    void initialize();
    void update();

   private:
    src::Drivers* drivers;

    static float distanceLeft;
    static float distanceRight;
    static float echoStartTimeMS;
    static tap::arch::PeriodicMilliTimer echoTimer;
    static tap::arch::MilliTimeout pulseTimer;

    // FIXME: Set these right somehow. (Probably have to add the pins to the "project.xml")
    static constexpr tap::gpio::Digital::OutputPin LEFT_TRIGGER_PIN = tap::gpio::Digital::OutputPin::Laser;
    static constexpr tap::gpio::Digital::OutputPin RIGHT_TRIGGER_PIN = tap::gpio::Digital::OutputPin::Laser;
    static constexpr uint16_t CM_PER_uS = 1 / 58.0f;
};

}  // namespace utils