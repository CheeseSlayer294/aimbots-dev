#pragma once

#include  "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "utils/common_types.hpp"
#include <string>

#include "subsystems/solenoid/solenoid.hpp"

namespace src::Solenoid{

enum SolenoidState : uint8_t {
    CLOSED = 0,
    EXTENDED = 1,
    UNKNOWN = 2
};

// using C5 = Board::PWMOutPinC5;
// using C4 = Board::PWMOutPinC4;
// using C3 = Board::PWMOutPinC3;


class SolenoidSubsytem : public tap::control::Subsystem{
    private:
        src::Drivers* drivers;
        // std::string rxPin;

        Solenoid solenoidHorizontal;
        Solenoid solenoidGrabber;

        uint8_t solenoidHorizontalState;
        uint8_t solenoidGrabberState;

        uint32_t actionStartTime;  // milliseconds

    public:
        SolenoidSubsytem(src::Drivers* driver);
     

        void initialize();
        void refresh();

        // void setSolenoidHorizontalState(SolenoidState state);
        // void setSolenoidGrabberState(SolenoidState state);
        // uint8_t getSolenoidHorizontalState() const;
        // uint8_t getSolenoidGrabberState() const;
        void setSolenoidPostion(float desiredAngle, std::string solenoidName);

        void setSolenoidState(uint8_t state, std::string solenoidName);
        uint8_t getSolenoidState(std::string solenoidName) const;

        bool isSolenoidReady();

        // const char* getName() const override { return "open hopper command"; }

        // void solenoidRead();
        // void solenoidWrite(bool value, std::string pin);


}; 
}// namespace src::Solenoid