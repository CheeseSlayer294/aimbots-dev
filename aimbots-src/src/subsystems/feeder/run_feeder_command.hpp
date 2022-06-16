#pragma once

#include "drivers.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Feeder {

class RunFeederCommand : public TapCommand {
   public:
    RunFeederCommand(src::Drivers*, FeederSubsystem*, float speed = FEEDER_DEFAULT_RPM, float acceptableHeatThreshold = 0.90f);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "run feeder"; }

   private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;

    float speed;
    float acceptableHeatThreshold;
    bool canShoot;
};

}  // namespace src::Feeder