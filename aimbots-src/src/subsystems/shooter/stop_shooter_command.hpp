#pragma once

#include "subsystems/shooter/shooter.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {

class StopShooterCommand : public TapCommand {
public:
    StopShooterCommand(Drivers* drivers, ShooterSubsystem* shooter);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    
    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "stop shooter command"; }

private:
    Drivers* drivers;
    ShooterSubsystem* shooter;
};

} // namespace src::Shooter

#endif // #ifdef SHOOTER_COMPATIBLE