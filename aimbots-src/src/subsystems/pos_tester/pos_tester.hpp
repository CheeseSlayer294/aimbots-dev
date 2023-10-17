#pragma once

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

static inline float DJIEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

namespace src::PosTester {

class PosTesterSubsystem : public tap::control::Subsystem {
public:
    //reference canbus and motor ID later
    PosTesterSubsystem(src::Drivers* drivers);
    
    mockable void initialize() override;
    mockable void refresh() override;

    void updateMotorPositionPID();
    
    mockable void setDesiredOutput();
    mockable void setTargetPosition(float);
    
    float getTargetPos() const { return targetPos; };
    float getCurrentPos() const { return DJIEncoderValueToRadians(motor.getEncoderWrapped()); };

private:
    float targetPos;
    float desiredOutput;
    
    DJIMotor motor;
    SmoothPID posPID;
};

};