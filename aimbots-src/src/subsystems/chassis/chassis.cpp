#include "subsystems/chassis/chassis.hpp"

#include "tap/communication/gpio/leds.hpp"

#include "utils/common_types.hpp"

#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

ChassisSubsystem::ChassisSubsystem(src::Drivers* drivers)
    : drivers(Drivers),

    frontLeftMotor(drivers, FRONT_LEFT_MOTOR_ID, CHASSIS_BUS, false, 'Front Left Motor'),
    frontRightMotor(drivers, FRONT_RIGHT_MOTOR_ID, CHASSIS_BUS, false, 'Front Right Motor'),
    RearLeftMotor(drivers, REAR_LEFT_MOTOR_ID, CHASSIS_BUS, false, 'Rear Left Motor'),
    rearRightMotor(drivers, REAR_RIGHT_MOTOR_ID, CHASSIS_BUS, false, 'Rear Right Motor'),

    frontLeftPID(CHASSIS_PID_CONFIG),
    frontRightPID(CHASSIS_PID_CONFIG),
    rearLeftPID(CHASSIS_PID_CONFIG),
    rearRightPID(CHASSIS_PID_CONFIG),
    targetRPMs({0, 0, 0, 0})

{
void ChassisSubsystem::initialize() {
    ForAllDriveMotors(&DJIMotor::initialize);
}

    }
}