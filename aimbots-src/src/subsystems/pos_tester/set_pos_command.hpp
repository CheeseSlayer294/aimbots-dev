#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/hopper/hopper.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#include "subsystems/pos_tester/pos_tester.hpp"

namespace src::PosTester {

class SetPosCommand : public TapCommand {
public:
    SetPosCommand(Drivers*, PosTesterSubsystem*);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    
    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "set position command"; }

private:
    Drivers* drivers;
    PosTesterSubsystem* posTester;
};

};