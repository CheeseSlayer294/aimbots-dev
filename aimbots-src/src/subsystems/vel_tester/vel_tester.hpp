#pragma once

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Vel_Tester {

class Vel_TesterSubsystem : public tap::control::Subsystem {
public:
    Vel_TesterSubsystem(src::Drivers* drivers);

    void initialize() override;

    void BuildVelTestMotor() {
    }
    void refresh() override;
    void setTargetRPM(float targetRPM);
    void setDesiredOutput();

    void updateMotorVelocityPID();


    float getTargetRPM() const {return targetRPM; }

    float getCurrentRPM() const {return vel_testerMotor.getShaftRPM();}

    public: 
        float desiredOutput;
        DJIMotor vel_testerMotor;
        float targetRPM;
        SmoothPID velTesterPID;
};



}