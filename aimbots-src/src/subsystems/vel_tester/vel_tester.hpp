#pragma once

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Vel_Tester {

class Vel_TesterSubsystem : public tap::control::subsystem {
public:
    Vel_TesterSubsystem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;
    void setTargetRPM(MotorIndex motorIdx, float targetRPM);
}
}