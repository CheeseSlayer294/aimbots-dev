#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/control/command.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#include "subsystems/pos_tester/pos_tester.hpp"

namespace src::PosTester {

class SetPosThrottleCommand : public TapCommand {
public:
    SetPosThrottleCommand(Drivers*, PosTesterSubsystem*) {}

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "set pos throttle command"; }

private:
    Drivers* drivers;
    PosTesterSubsystem* posTester;
};

}