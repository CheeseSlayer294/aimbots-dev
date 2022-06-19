#pragma once

#include "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"  //maybe not
#include "utils/common_types.hpp"
#include "subsystems/solonoid/solonoid.hpp"

#if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solonoid{
    class SolonoidController : public tap::control::Subsystem{
        private:
            src::Drivers* drivers;
            Solonoid* solonoid;


        public:
            SolonoidController(tap::Drivers drivers*);

            
           

            mockable void initialize() override;
            void refresh() override;





    };

}//namespace src::Solonoid

#endif