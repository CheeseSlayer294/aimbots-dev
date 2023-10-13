#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "subsystems/feeder/feeder.hpp"

#ifdef FEEDER_COMPATIBLE        // checks for the existence of macro definitions

namespace src::Feeder {

class StopFeederCommand : public TapCommand {
public:
    StopFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "stop feeder command"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
};

};


#endif      // close ifdef