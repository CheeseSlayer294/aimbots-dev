#pragma once
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "informants/limit_switch.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::vel_tester {
class Velocity_Control : public tap::control::Subsystem {
public:
    Velocity_Control(src::Drivers* drivers);
    void initialize() override;
    void refresh() override;

    void updateMotorVelocityPID();
    void setDesiredOutput();
    float setTargetRPM(float);
    float getCurrentRPM() const { return testMotor.getShaftRPM(); }

private:
    float targetRPM;
    float desiredOutput;

    SmoothPID velPID;
    DJIMotor testMotor;
};
}  // namespace src::vel_tester
