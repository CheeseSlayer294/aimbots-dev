#pragma once

#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"
#include "drivers.hpp"

namespace src::HUD_Display {

class HUD_DisplaySubsystem : public tap::control::Subsystem {

    //Virtual sub-system, exists only to run commands on it.
    public:
    HUD_DisplaySubsystem(src::Drivers* drivers);
};

};