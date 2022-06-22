#pragma once

#include "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "utils/common_types.hpp"
#include "subsystems/solenoid/solenoid.hpp"

// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{
    class SolenoidController : public TapCommand{
        private:
            src::Drivers* drivers;
            SolenoidSubsytem* solenoid;


        public:
            SolenoidController(src::Drivers* drivers, SolenoidSubsytem* solenoid);
            
            void initialize() override;

            void execute() override;
            void end(bool interrupted) override;
            bool isReady() override;

            bool isFinished() const override;


    };

}//namespace src::Solenoid

// #endif