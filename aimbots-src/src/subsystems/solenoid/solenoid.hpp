#pragma once

#include  "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "utils/common_types.hpp"
#include <string>

// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

using C1 = Board::PWMOutPinC1;
using C2 = Board::PWMOutPinC2;
using C3 = Board::PWMOutPinC3;


namespace src::Solenoid{

class SolenoidSubsytem : public tap::control::Subsystem{
    private:
        tap::Drivers* drivers;
        // std::string rxPin;

        bool C1State;
        bool C2State;
        bool C3State;        

    public:
        SolenoidSubsytem(src::Drivers* driver);

        /**
         * Allows user to call a DJIMotor member function on all shooter motors
         *
         * @param function pointer to a member function of DJIMotor
         * @param args arguments to pass to the member function
        */
        // template <class... Args>
        //     void ForAllSolenoids(void (DJIMotor::*func)(Args...), Args... args) {
        //         for (auto i = 0; i < SOLeNOID_COUNT; i++) {
        //             (solenoid[i][0]->*func)(args...);
        //         }
        //     }       

        void initialize();
        void refresh();

        void solenoidRead();
        void solenoidWrite(bool value, std::string pin);



};
}//namespace src::Solonoid

// #endif
