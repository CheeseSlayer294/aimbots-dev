#pragma once
#ifndef TARGET_DART
#include "drivers.hpp"
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_follow_gimbal_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"

namespace src::Chassis {

class ChassisToggleDriveCommand : public TapComprisedCommand {
   public:
    ChassisToggleDriveCommand(src::Drivers*, ChassisSubsystem*, Gimbal::GimbalSubsystem*);

    void initialize() override;
    void execute() override;

    void end(bool interupted) override;
    bool isReady() override;
    bool isFinished() const override;

    char const* getName() const override { return "Chassis Toggle Drive Command"; }

   private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    ChassisFollowGimbalCommand followGimbalCommand;
    ChassisTokyoCommand tokyoCommand;

    bool wasFPressed = false;
};

}  // namespace src::Chassis
#endif