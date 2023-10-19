#pragma once

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/vel_tester/vel_tester.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::vel_tester{

class Velocity_Control_Command : public TapCommand {
public:
    Velocity_Control_Command(
        src::Drivers*,
        Velocity_Control*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "run vel_tester";}

private:
    src::Drivers* drivers;
    Velocity_Control* vel_tester;
};
}
