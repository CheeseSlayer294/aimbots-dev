#pragma once

#if defined(CHASSIS) && defined(GIMBAL)

#include "tap/control/command.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Chassis {

class ChassisFollowGimbalCommand : public TapCommand {
public:
    ChassisFollowGimbalCommand(src::Drivers*, ChassisSubsystem*, src::Gimbal::GimbalSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Chassis Follow Gimbal"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    SmoothPID rotationController;
};

}  // namespace src::Chassis

#endif