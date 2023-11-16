#pragma once
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "informants/limit_switch.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {
class FeederSubsystem : public tap::control::Subsystem {
public:
    FeederSubsystem(src::Drivers* drivers);

    mockable void initialize() override;
    mockable void refresh() override;

    void updateMotorVelocityPID();

    mockable void setDesiredOutput();

    mockable float setTargetRPM(float rpm);

    float getTargetRPM() const { return targetRPM; }

    bool getPressed();

    float getCurrentRPM() const { return feederMotor.getShaftRPM(); }

    


private:

    float targetRPM;
    float desiredOutput;

    SmoothPID feederVelPID;
    DJIMotor feederMotor;
    src::Drivers* drivers;

    src::Informants::LimitSwitch limitSwitch;  // for single-barreled robots
//#endif

    // commands
};

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE