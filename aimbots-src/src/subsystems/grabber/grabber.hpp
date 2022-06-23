#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"

// #define ENGINEER

namespace src::Grabber {

enum GrabberState : uint8_t {
    CLOSED = 0,
    OPEN = 1,
    UNKNOWN = 2
};

class GrabberSubsystem : public tap::control::Subsystem {
   public:
    GrabberSubsystem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;

    // motor control commands here, specific open/close commands separate?
    // need to know more about servo class

    // taproot takes 0-1 which I'm assuming maps to 0-360 degrees
    // want to take 0-360 here and map to taproot 0-1

    /**
     * @brief Sets angle for hopper servo to turn to and maintain (don't call continuously!!!)
     *
     */
    void setGrabberAngle(float desiredAngle);

    /**
     * @brief Returns true if Grabber PWM ramp is finished AND minimum delay (declared in standard_constants) has passed
     *
     */
    bool isGrabberReady() const;

    /**
     * @brief Returns the current state of the Grabber as a uint8_t (check GrabberState enum)
     *
     */
    uint8_t getGrabberState() const;

    /**
     * @brief setter for Grabber state integer
     *
     */
    void setGrabberState(uint8_t new_state);

   private:
    tap::Drivers* drivers;

    Servo grabberMotorRight, grabberMotorLeft;
    uint8_t grabber_state;
    uint32_t actionStartTime;  // milliseconds
};
};  // namespace src::Grabber

// #endif