#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/linear_stage/linear_stage.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_helper.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::LinearStage {

class RunStageCommand : public TapCommand {
public:
    RunStageCommand(src::Drivers* drivers, LinearStageSubsystem* linear_stage, src::Utils::RefereeHelper* refHelper);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "linear stage subsystem"; }

private:
    src::Drivers* drivers;
    LinearStageSubsystem* linear_stage;

    src::Utils::RefereeHelper* refHelper;
};

}  // namespace src::LinearStage