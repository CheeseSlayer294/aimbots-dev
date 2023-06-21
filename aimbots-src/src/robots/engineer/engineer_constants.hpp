#pragma once
#include "utils/common_types.hpp"

#define ENGINEER

static constexpr uint8_t DRIVEN_WHEEL_COUNT = 4;
static constexpr uint8_t MOTORS_PER_WHEEL = 1;

/**
 * @brief Velocity PID constants
 */
static constexpr float VELOCITY_PID_KP = 20.0f;
static constexpr float VELOCITY_PID_KI = 0.2f;
static constexpr float VELOCITY_PID_KD = 0.0f;
static constexpr float VELOCITY_PID_MAX_ERROR_SUM = 5000.0f;

/**
 * @brief Position PID constants
 */
static constexpr float POSITION_PID_KP = 20.0f;
static constexpr float POSITION_PID_KI = 0.2f;
static constexpr float POSITION_PID_KD = 0.0f;
static constexpr float POSITION_PID_MAX_ERROR_SUM = 5000.0f;

// Mechanical chassis constants, all in m
/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.076;

static constexpr float WHEELBASE_WIDTH = 0.366f;

static constexpr float WHEELBASE_LENGTH = 0.366f;

static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;

static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

static constexpr float YAW_OFFSET_ANGLE = M_PI_2;
static constexpr float PITCH_OFFSET_ANGLE = M_PI_2;
// #error "DM Richard on Discord if you see this (or just calculate the pitch stop limits yourself idc)"
static constexpr float PITCH_AXIS_SOFTSTOP_LOW = 0.0f;
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = 0.0f;

static constexpr MotorID LEFT_BACK_WHEEL_ID = MotorID::MOTOR1;
static constexpr MotorID LEFT_FRONT_WHEEL_ID = MotorID::MOTOR2;
static constexpr MotorID RIGHT_FRONT_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID RIGHT_BACK_WHEEL_ID = MotorID::MOTOR4;
static constexpr MotorID YAW_MOTOR_ID = MotorID::MOTOR5;
static constexpr MotorID PITCH_MOTOR_ID = MotorID::MOTOR6;

static constexpr CANBus LINEAR_STAGE_BUS = CANBus::CAN_BUS2;

static const int STAGE_MOTOR_COUNT = 3;

static const bool X_STAGE_DIRECTION = true;
static const bool Y_STAGE_DIRECTION = true;
static const bool Z_STAGE_DIRECTION = true;

static constexpr MotorID LINEAR_STAGE_X_MOTOR_ID = MotorID::MOTOR1;
static constexpr MotorID LINEAR_STAGE_Y_MOTOR_ID = MotorID::MOTOR2;
static constexpr MotorID LINEAR_STAGE_Z_MOTOR_ID = MotorID::MOTOR3;

enum WheelRPMIndex {  // index used to easily navigate wheel matrices
    LB = 0,
    LF = 1,
    RF = 2,
    RB = 3,
};

static constexpr SmoothPIDConfig STAGE_VELOCITY_PID_CONFIG = {
    .kp = 18.0f,
    .ki = 0.0f,
    .kd = 2.0f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

/**
 * Max wheel speed, measured in RPM of the 3508 motor shaft.
 */
static constexpr int MAX_3508_ENC_RPM = 7000;

// Power limiting constants, will explain later
static constexpr float MAX_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 40.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 5;
static constexpr uint16_t POWER_CONSUMPTION_THRESHOLD = 20;
static constexpr float CURRENT_ALLOCATED_FOR_ENERGY_BUFFER_LIMITING = 30000;


//allows for engineer to compule easily 
#warning "this should be changed but it is an easy fix for now"

static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;
static constexpr float FOLLOW_GIMBAL_ANGLE_THRESHOLD = modm::toRadian(20.0f);
static constexpr SmoothPIDConfig ROTATION_POSITION_PID_CONFIG = {
    .kp = 1.25f,
    .ki = 0.0f,
    .kd = 0.03f,
    .maxICumulative = 10.0f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};
static constexpr float PITCH_AXIS_START_ANGLE = modm::toRadian(0.0f);
static constexpr float GIMBAL_PITCH_GEAR_RATIO = (30.0f / 102.0f);  // for 2023 Standard
static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(-23.0f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(22.0f);
static constexpr float YAW_AXIS_START_ANGLE = modm::toRadian(0.0f);
static constexpr float GIMBAL_YAW_GEAR_RATIO = (1.0f / 2.0f);  // for 2023 Standard

