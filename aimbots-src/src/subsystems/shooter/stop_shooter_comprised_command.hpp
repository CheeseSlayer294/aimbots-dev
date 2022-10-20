#pragma once

#include "drivers.hpp"
#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

// #ifndef TARGET_DART
//#ifndef TARGET_ENGINEER

namespace src::Shooter {

class StopShooterComprisedCommand : public TapComprisedCommand {
   public:
    StopShooterComprisedCommand(src::Drivers* drivers, ShooterSubsystem* shooter);
    void initialize() override;

    void execute() override;
    
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Slow To Stop Command"; }

   private:
    src::Drivers* drivers;
    ShooterSubsystem* shooter;

    BrakeShooterCommand brake_command;
    StopShooterCommand stop_command;
    bool brakeFinished;
};

}  // namespace src::Shooter

// #endif /