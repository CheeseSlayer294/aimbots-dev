#pragma once

#include "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "utils/common_types.hpp"
#include "subsystems/solenoid/solenoid_subsystem.hpp"

// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{
    class ToggleSolenoidCommand : public TapCommand{
        private:
            src::Drivers* drivers;
            SolenoidSubsytem* solenoid;
            std::string solenoidName;

        public:
            ToggleSolenoidCommand(src::Drivers* drivers, SolenoidSubsytem* solenoid, std::string solenoidName);
            
            void initialize() override;

            void execute() override;
            void end(bool interrupted) override;
            bool isReady() override;

            bool isFinished() const override;
            const char* getName() const override { return "toggle solenoid command"; }


    };

}//namespace src::Solenoid

// #endif