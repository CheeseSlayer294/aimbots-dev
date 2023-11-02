#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "informants/limit_switch.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {\

enum WheelIndex{
    LB = 0,
    LF = 1,
    RF = 2,
    RB = 3
}

class ChassisSubsystem : public tap::control::Subsyste, {
public:
    ChassisSubsystem(src::Drivers* drivers);

    template <class.. Args>
    void ForAllDriveMotors(void (ChassisSubsystem::*func)(MotorIndex, Args..), Args... args){
        for (auto i = o;, i <DRIVE_MOTOR_COUNT>)
    }

    mockable void initialize() override;
    mockable void refresh() override;

    void updateMotorVelocityPID(int idx);

    mockable void setDesiredOutput();

    mockable void setTargetRPMs(float fwd, float right, float rotateRight);

    std::array<int, 4> computeRPMs (float fwd, float right, float rotateRight, int maxVelo);

    std::array<int, 4> getTargetRPMs() const { return targetRPMs; }

    DJIMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    SmoothPID frontLeftPID, frontRightPID, rearLeftPID, rearRightPID;

    int targetRPMs[4];
#endif
    


    
}
}