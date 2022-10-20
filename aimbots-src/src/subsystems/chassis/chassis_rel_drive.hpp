#pragma once
#ifndef TARGET_DART

#include <drivers.hpp>
#include <subsystems/chassis/chassis.hpp>

namespace src::Chassis::Movement::Relative {

void calculateUserDesiredMovement(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    float* desiredXSpeed,
    float* desiredYSpeed,
    float desiredChassisRotation);

void onExecute(src::Drivers* drivers, ChassisSubsystem* chassis);
}  // namespace src::Chassis::Movement::Relative
#endif