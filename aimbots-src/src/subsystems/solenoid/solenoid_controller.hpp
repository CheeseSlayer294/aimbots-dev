#pragma once

#include "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "utils/common_types.hpp"
#include "subsystems/solenoid/solenoid.hpp"

#if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{
    class SolenoidController : public TapCommand{
        private:
            src::Drivers* drivers;
            Solenoid* solenoid;


        public:
            SolenoidController(tap::Drivers* drivers, Solenoid* solenoid);
            
            void initialize() override;

            void execute() override;
            void end(bool interrupted) override;
            bool isReady() override;

            bool isFinished() const override;


    };

}//namespace src::Solenoid

#endif