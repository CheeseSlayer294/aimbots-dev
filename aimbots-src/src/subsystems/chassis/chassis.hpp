#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "informants/limit_switch.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

enum WheelIndex{
    LB = 0,
    LF = 1,
    RF = 2,
    RB = 3
};

enum ChassisVelIndex {
    X = 0,
    Y = 1,
    R = 2, //why is this comma here?
};

class ChassisSubsystem : public tap::control::ChassisSubsystemInterface {
public:
    ChassisSubsystem(src::Drivers* drivers);

    //function to apply a Chassis function to all motors
    template <class... Args> // n arguments of type generic type Args, template keyword denotes arbitrary typing
    void ForAllDriveMotors(void (ChassisSubsystem::*func)(WheelIndex, Args..), Args... args){ // "ChassisSubsystem::*func" denotes a function that exists in "this", function must take a MotorIndex first, then Args... 
        for (auto i = 0; i < DRIVE_MOTOR_COUNT; i++) {//auto keyword automatically detirmines tpye
            WheelIndex mi = static_cast<WheelIndex>(i); //integer to enum stuff
             (this->*func)(mi, DRIVER, args...); //gets func from this, runs it with mi as an argument
    }
    }

    //function to apply a DJIMotor function to all motors
    template <class... Args> // n arguments of type generic type Args, template keyword denotes arbitrary typing
    void ForAllChassisMotors(void (DJIMotor::*func)(Args...), Args... args) { //DJIMotor::*func denotes a function that exists in DJIMotor, no motor index this time because this function acts directly on DJIMotor objects 
        for (auto i = 0; i < DRIVEN_WHEEL_COUNT; i++) { //auto keyword automatically detirmines type
            (motors[i][DRIVER]->*func)(args...); // ->* calls 'func' with 'args...' on motors[i]

        }
    }

    mockable void initialize() override;
    mockable void refresh() override;

    void updateMotorVelocityPID(WheelIndex idx);

    void setTargetRPMs(float x, float y, float r);

    mockable void setDesiredOutput(WheelIndex idx);

    void calculateHolonomic(float x, float y, float r, float maxWheelSpeed);  // normal 4wd mecanum robots

    std::array<int, 4> computeRPMs (float fwd, float right, float rotateRight, int maxVelo);

    std::array<int, 4> getTargetRPMs() const { return targetRPMs; }

    inline int getNumChassisMotors() const override { return DRIVEN_WHEEL_COUNT * MOTORS_PER_WHEEL; }

    inline float getDesiredRotation() const { return desiredRotation; }

    void limitChassisPower();


    DJIMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    SmoothPID frontLeftPID, frontRightPID, rearLeftPID, rearRightPID;

    int targetRPMs[4];
#endif
    


    
}
}