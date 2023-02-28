#ifdef CHASSIS
#pragma once

#include "tap/control/command.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Chassis {

class ChassisManualDriveCommand : public TapCommand {
public:
    ChassisManualDriveCommand(src::Drivers*, ChassisSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Chassis Manual Drive"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
};

}  // namespace src::Chassis
#endif