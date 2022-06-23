#pragma once

// #ifdef ENGINEER

#include "drivers.hpp"
#include "subsystems/grabber/grabber.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Grabber {

class CloseGrabberCommand : public TapCommand {
   public:
    CloseGrabberCommand(src::Drivers* drivers, GrabberSubsystem* grabber);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "close grabber command"; }

   private:
    src::Drivers* drivers;
    GrabberSubsystem* grabber;
};
};  // namespace src::grabber

// #endif